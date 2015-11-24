#include "Physics2012Mt.h"

Physics2012Mt::Physics2012Mt() {

}

Physics2012Mt::~Physics2012Mt() {

}

void Physics2012Mt::initHk() {
	memoryRouter = hkMemoryInitUtil::initDefault(hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(1024 * 1024));
	hkBaseSystem::init(memoryRouter, errorReport);

	initThreads();
}

void Physics2012Mt::runHk() {
	initPhysics();

	vdb.setVisualDebugger(m_world, true);

	//Simulate the world for 1 minute
	hkStopwatch stopWatch;
	stopWatch.start();
	hkReal lastTime = stopWatch.getElapsedSeconds();
	hkReal timeStep = 1.f / 60.f;
	int numSteps = int(10.f / timeStep);
	for (int i = 0; i < numSteps; i++) {
		m_world->stepMultithreaded(jobQueue, threadPool, timeStep);

		vdb.getVDBContext()->syncTimers(threadPool);
		vdb.step();

		hkMonitorStream::getInstance().reset();
		threadPool->clearTimerData();

		if (i % 60 == 0) {
			hkVector4 pos = m_ball->getPosition();
			hkStringBuf msg;
			msg.printf("[%f, %f, %f]\n", pos(0), pos(1), pos(2));
			errorReport(msg.cString(), HK_NULL);
		}

		while (stopWatch.getElapsedSeconds() < lastTime + timeStep);
		lastTime += timeStep;
	}
}

void Physics2012Mt::quitHk() {
	m_world->markForWrite();
	m_world->removeReference();
	vdb.quitVdb();
	delete jobQueue;
	threadPool->removeReference();

	hkBaseSystem::quit();
	hkMemoryInitUtil::quit();
}

void Physics2012Mt::initThreads() {
	//get the number of physical threads available on system
	int totalNumThreadsUsed = hkHardwareInfo::getNumHardwareThreads();

	//we use num_threads - 1 because we must also use a thread for simulation
	hkCpuThreadPoolCinfo threadPoolCInfo;
	threadPoolCInfo.m_numThreads = totalNumThreadsUsed - 1;
	//this enables timers collection, by allocating 200 kb per thread
	threadPoolCInfo.m_timerBufferPerThreadAllocation = 200000;
	threadPool = new hkCpuThreadPool(threadPoolCInfo);

	//this job queue is used to run multithreaded work
	hkJobQueueCinfo info;
	info.m_jobQueueHwSetup.m_numCpuThreads = totalNumThreadsUsed;
	jobQueue = new hkJobQueue(info);

	//Enable monitors for this thread
	hkMonitorStream::getInstance().resize(200000);
}

void Physics2012Mt::initPhysics() {
	//world simulation parameters
	hkpWorldCinfo worldInfo;
	//simulation type = multithreaded
	worldInfo.m_simulationType = hkpWorldCinfo::SIMULATION_TYPE_MULTITHREADED;
	//flag objects that fall out of the world to be automatically removed
	worldInfo.m_broadPhaseBorderBehaviour = hkpWorldCinfo::BROADPHASE_BORDER_REMOVE_ENTITY;

	m_world = new hkpWorld(worldInfo);
	//you can view timers in the VDB. !!!(this should not be done in your game)!!!
	m_world->m_wantDeactivation = false;

	//each thread must call markForRead/markForWrite before it modifies the world
	m_world->markForWrite();

	hkpAgentRegisterUtil::registerAllAgents(m_world->getCollisionDispatcher());

	m_world->registerWithJobQueue(jobQueue);

	createGroundPh();
	hkVector4 posy(0, 0, 0);
	createWallsPh(posy);
	createBallPh(posy);
}

void Physics2012Mt::createGroundPh() {
	//create the ground box
	hkVector4 groundRadii(70.0f, 2.0f, 140.0f);
	hkpConvexShape* shape = new hkpBoxShape(groundRadii, 0);

	hkpRigidBodyCinfo ci;
	ci.m_shape = shape;
	ci.m_motionType = hkpMotion::MOTION_FIXED;
	ci.m_position = hkVector4(0.0f, -2.0f, 0.0f);
	ci.m_qualityType = HK_COLLIDABLE_QUALITY_FIXED;

	hkpRigidBody* rigidBody = new hkpRigidBody(ci);
	m_world->addEntity(rigidBody);
	rigidBody->removeReference();
	shape->removeReference();
}

void Physics2012Mt::createWallsPh(hkVector4& posy) {
	int wallHeight = 8, wallWidth = 8, numWalls = 6;
	hkVector4 boxSize(1.0f, 0.5f, 0.5f);
	hkpBoxShape* box = new hkpBoxShape(boxSize, 0);

	hkReal deltaZ = 25.0f;
	posy(2) = -deltaZ * numWalls * 0.5f;

	for (int y = 0; y < numWalls; y++) {
		//create a wall
		createSingleWallPh(wallHeight, wallWidth, posy, 0.2f, box, boxSize);
		posy(2) += deltaZ;
	}
	box->removeReference();
}

void Physics2012Mt::createSingleWallPh(const int height, const int length, const hkVector4& position, const hkReal gapWidth, hkpConvexShape* box, hkVector4Parameter halfExtents) {
	hkVector4 posx = position;

	//do a raycast to place the wall
	hkpWorldRayCastInput ray;
	ray.m_from = posx;
	ray.m_to = posx;

	ray.m_from(1) += 20.0f;
	ray.m_to(1) -= 20.0f;

	hkpWorldRayCastOutput result;
	m_world->castRay(ray, result);
	posx.setInterpolate4(ray.m_from, ray.m_to, result.m_hitFraction);

	//move the start point
	posx(0) -= (gapWidth + 2.0f * halfExtents(0)) * length * 0.5f;
	posx(1) -= halfExtents(1) + box->getRadius();

	hkArray<hkpEntity*> entitiesToAdd;
	for (int x = 0; x < length; x++) {
		hkVector4 pos = posx;
		for (int ii = 0; ii < height; ii++) {
			pos(1) += (halfExtents(1) + box->getRadius()) * 2.0f;

			hkpRigidBodyCinfo boxInfo;
			hkMassProperties massProperties;
			hkpInertiaTensorComputer::computeBoxVolumeMassProperties(halfExtents, 10.0f, massProperties);

			boxInfo.setMassProperties(massProperties);
			boxInfo.m_solverDeactivation = boxInfo.SOLVER_DEACTIVATION_MEDIUM;
			boxInfo.m_shape = box;
			boxInfo.m_restitution = 0.0f;
			boxInfo.m_motionType = hkpMotion::MOTION_BOX_INERTIA;

			{
				boxInfo.m_position = pos;
				hkpRigidBody* boxRigidBody = new hkpRigidBody(boxInfo);
				m_world->addEntity(boxRigidBody);
				boxRigidBody->removeReference();
			}

			pos(1) += (halfExtents(1) + box->getRadius()) * 2.0f;
			pos(0) += halfExtents(0) * 0.6f;
			boxInfo.m_position = pos;
			hkpRigidBody* boxRigidBody = new hkpRigidBody(boxInfo);
			entitiesToAdd.pushBack(boxRigidBody);
			pos(0) -= halfExtents(0) * 0.6f;
		}
		posx(0) += halfExtents(0) * 2.0f + gapWidth;
	}
	m_world->addEntityBatch(entitiesToAdd.begin(), entitiesToAdd.getSize());

	for (int i = 0; i < entitiesToAdd.getSize(); i++) {
		entitiesToAdd[i]->removeReference();
	}
}

void Physics2012Mt::createBallPh(hkVector4& posy) {
	const hkReal radius = 1.5f;
	const hkReal sphereMass = 150.0f;

	hkVector4 relPos(0.0f, radius + 0.0f, 50.0f);

	hkpRigidBodyCinfo info;
	hkMassProperties massProperties;
	hkpInertiaTensorComputer::computeSphereVolumeMassProperties(radius, sphereMass, massProperties);

	info.setMassProperties(massProperties);
	info.m_shape = new hkpSphereShape(radius);
	info.m_position.setAdd4(posy, relPos);
	info.m_motionType = hkpMotion::MOTION_BOX_INERTIA;
	info.m_qualityType = HK_COLLIDABLE_QUALITY_BULLET;

	hkpRigidBody* sphereRigidBody = new hkpRigidBody(info);
	m_ball = sphereRigidBody;

	m_world->addEntity(sphereRigidBody);
	sphereRigidBody->removeReference();
	info.m_shape->removeReference();

	hkVector4 vel(0.0f, 4.9f, -100.0f);
	sphereRigidBody->setLinearVelocity(vel);
}