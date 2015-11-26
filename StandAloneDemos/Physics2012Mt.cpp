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

	add_cameraIrr();
	add_gui_elementsIrr();
	add_scene_nodesIrr();

	vdb.setVisualDebugger(m_world, true);

	//Simulate the world for 1 minute
	hkStopwatch stopWatch;
	stopWatch.start();
	hkReal lastTime = stopWatch.getElapsedSeconds();
	hkReal timeStep = 1.f / 60.f;
	int numSteps = int(10.f / timeStep);
	for (int i = 0; i < numSteps; i++) {
		m_world->stepMultithreaded(jobQueue, threadPool, timeStep);

		runIrr();

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

	quit_Irr();
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

	scene::IMeshSceneNode* g_ground = scene_manager->addCubeSceneNode(1.f, 0, -1, core::vector3df(0.f, -2.f, 0.f), core::vector3df(), core::vector3df(140.f, 4.f, 280.f));
	//g_ground->setMaterialFlag(video::EMF_LIGHTING, false);
	g_ground->addShadowVolumeSceneNode();
	g_ground->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
	scene_manager->getMeshManipulator()->setVertexColors(g_ground->getMesh(), video::SColor(255, 255, 0, 0));
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
				m_walls.pushBack(boxRigidBody);
				boxRigidBody->removeReference();

				scene::IMeshSceneNode* g_wall = scene_manager->addCubeSceneNode(1.f, 0, -1, core::vector3df(pos(0), pos(1), pos(2)), core::vector3df(), core::vector3df(2.f, 1.f, 1.f));
				g_wall->addShadowVolumeSceneNode();
				g_wall->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
				scene_manager->getMeshManipulator()->setVertexColors(g_wall->getMesh(), video::SColor(255, 142, 22, 175));
				g_walls.push_back(g_wall);
			}

			pos(1) += (halfExtents(1) + box->getRadius()) * 2.0f;
			pos(0) += halfExtents(0) * 0.6f;
			boxInfo.m_position = pos;
			hkpRigidBody* boxRigidBody = new hkpRigidBody(boxInfo);
			entitiesToAdd.pushBack(boxRigidBody);
			m_walls.pushBack(boxRigidBody);

			scene::IMeshSceneNode* g_wall = scene_manager->addCubeSceneNode(1.f, 0, -1, core::vector3df(pos(0), pos(1), pos(2)), core::vector3df(), core::vector3df(2.f, 1.f, 1.f));
			g_wall->addShadowVolumeSceneNode();
			g_wall->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
			scene_manager->getMeshManipulator()->setVertexColors(g_wall->getMesh(), video::SColor(255, 142, 22, 175));
			g_walls.push_back(g_wall);

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

	g_ball = scene_manager->addSphereSceneNode(radius, 16);
	g_ball->setPosition(core::vector3df(m_ball->getPosition()(0), m_ball->getPosition()(1), m_ball->getPosition()(2)));
	//g_ball->setMaterialFlag(video::EMF_LIGHTING, false);
	g_ball->addShadowVolumeSceneNode();
	g_ball->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
	scene_manager->getMeshManipulator()->setVertexColors(g_ball->getMesh(), video::SColor(255, 0, 255, 0));
}

//---------------------------Irrlicht-------------------------------//

const int Physics2012Mt::add_cameraIrr() {
	scene_manager->addCameraSceneNode(0, core::vector3df(10.f, 19.f, -20.f), core::vector3df());
	//scene_manager->addCameraSceneNodeFPS();

	return 0;
}

const int Physics2012Mt::add_gui_elementsIrr() {
	device->setWindowCaption(L"Physics2012Mt Demo - Havok+Irrlicht Engine");

	return 0;
}

const int Physics2012Mt::add_scene_nodesIrr() {
	scene_manager->addLightSceneNode(0, core::vector3df(0, 100, 20), video::SColorf(1.0f, 1.f, 1.f, 1.0f), 800.0f);
	return 0;
}

const int Physics2012Mt::runIrr() {
	device->run();

	driver->beginScene();

	scene_manager->drawAll();
	gui_env->drawAll();

	driver->setTransform(video::ETS_WORLD, core::matrix4());

	//ball position & rotation
	hkVector4 newpos = m_ball->getPosition();
	g_ball->setPosition(core::vector3df(newpos(0), newpos(1), newpos(2)));

	hkQuaternion newrot_hk = m_ball->getRotation();
	if (newrot_hk.hasValidAxis()) {
		hkVector4f axis;
		newrot_hk.getAxis(axis);
		float angle = core::radToDeg(newrot_hk.getAngle());
		g_ball->setRotation(core::vector3df(axis(0), axis(1), axis(2)) * angle);
	}

	//walls positions & rotations
	for (int i = 0; i < m_walls.getSize(); i++) {
		newpos = m_walls[i]->getPosition();
		g_walls[i]->setPosition(core::vector3df(newpos(0), newpos(1), newpos(2)));

		newrot_hk = m_walls[i]->getRotation();
		if (newrot_hk.hasValidAxis()) {
			hkVector4f axis;
			newrot_hk.getAxis(axis);
			float angle = core::radToDeg(newrot_hk.getAngle());
			g_walls[i]->setRotation(core::vector3df(axis(0), axis(1), axis(2)) * angle);
		}
	}

	driver->endScene();

	return 0;
}