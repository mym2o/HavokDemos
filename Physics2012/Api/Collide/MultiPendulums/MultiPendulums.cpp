#include "MultiPendulums.h"

void MultiPendulums::initHk() {
	memoryRouter = hkMemoryInitUtil::initDefault(hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(1024 * 1024));
	hkBaseSystem::init(memoryRouter, errorReport);

	hkpWorldCinfo info;
	info.setupSolverInfo(hkpWorldCinfo::SOLVER_TYPE_4ITERS_MEDIUM);
	info.m_gravity.set(0.f, -9.81f, 0.f);
	info.m_broadPhaseWorldAabb.m_min.set(-100.f, -100.f, -100.f);
	info.m_broadPhaseWorldAabb.m_max.setNeg4(info.m_broadPhaseWorldAabb.m_min);
	info.m_enableDeactivation = false;

	m_world = new hkpWorld(info);
	m_world->lock();

	//register all agents
	hkpAgentRegisterUtil::registerAllAgents(m_world->getCollisionDispatcher());

	createWorld();

	m_world->unlock();
}

void MultiPendulums::quitHk() {
	//hkError::getInstance().setEnabled(0xaf55adde, true);
	m_marbleAction->removeReference();

	vdb.quitVdb();

	hkBaseSystem::quit();
	hkMemoryInitUtil::quit();

	quit_Irr();
}

MultiPendulums::MultiPendulums() : IrrInterface() {
	//disable warnings
	//hkError::getInstance().setEnabled(0xaf55adde, false);
	device->setEventReceiver(&receiver);
}

void MultiPendulums::createWorld() {
	//create the base
	const float xdim = 25.f;
	const float zdim = 25.f;
	const float bW = 0.5f;	//borderWidth
	const float bH = 0.5f;	//borderHeight

	GameUtils::createStaticBox(m_world, 0.f, -0.5f, 0.f, xdim, 1.f, zdim);

	GameUtils::createStaticBox(m_world, -xdim, bH, 0.f, bW, bH, zdim);
	GameUtils::createStaticBox(m_world, xdim, bH, 0.f, bW, bH, zdim);
	GameUtils::createStaticBox(m_world, 0.f, bH, zdim, xdim, bH, bW);
	GameUtils::createStaticBox(m_world, 0.f, bH, -zdim, xdim, bH, bW);

	createMultiPendulumField();

	//create the moving block
	hkVector4 characterPos(0.f, 3.f, 5.f);
	hkVector4 bSize(.7f, .7f, .7f);
	hkpRigidBody* characterRigidBody = GameUtils::createBox(bSize, 300.f, characterPos);
	m_world->addEntity(characterRigidBody);
	characterRigidBody->removeReference();

	//create the "marble" action to drive him around
	hkVector4 forward(1.f, 0.f, 0.f);
	m_marbleAction = new MarbleAction(characterRigidBody, forward, characterRigidBody->getPosition(), 0.5f);
	m_world->addAction(m_marbleAction);
}

void MultiPendulums::createMultiPendulumField() {
	const int shift = 0;

	const int numBodies = hkMath::clamp(1000, 100, 1000) >> shift;
	int size = (int)hkMath::sqrt(float(numBodies));

	int row = -size / 2;
	int column = -size / 2;

	hkVector4 boxSize(.2f, .2f, .2f);
	hkpShape* boxShape = new hkpBoxShape(boxSize, 0);

	hkpRigidBody* fixedBody = m_world->getFixedRigidBody();

	hkPseudoRandomGenerator generator(324);

	for (int i = 1; i < numBodies; i++) {
		if (i % size == 0) {
			row = -(size >> 1);
			column++;
		}
		else {
			row++;
		}

		hkVector4 boxPos;
		boxPos.set(hkReal(row), 1.f, hkReal(column));

		hkpRigidBodyCinfo boxInfo;
		boxInfo.m_shape = boxShape;
		boxInfo.m_motionType = hkpMotion::MOTION_SPHERE_INERTIA;
		boxInfo.m_position = boxPos;
		boxInfo.m_mass = 1.f;
		boxInfo.m_angularDamping = 1.f;

		hkpInertiaTensorComputer::setShapeVolumeMassProperties(boxInfo.m_shape, boxInfo.m_mass, boxInfo);

		hkpRigidBody* body = new hkpRigidBody(boxInfo);
		hkVector4 velocity(0.f, 0.f, generator.getRandRange(0.f, 0.2f));
		body->setLinearVelocity(velocity);
		m_world->addEntity(body);
		body->removeReference();

		body->getMaterial().setFriction(0.f);

		hkVector4 pivot, pivotOffset;
		pivotOffset.set(0.f, 1.f, 0.f);
		pivot.setAdd4(boxPos, pivotOffset);

		hkpBallAndSocketConstraintData* bas = new hkpBallAndSocketConstraintData();
		bas->setInWorldSpace(fixedBody->getTransform(), body->getTransform(), pivot);

		m_world->createAndAddConstraintInstance(fixedBody, body, bas)->removeReference();
		bas->removeReference();
	}
	boxShape->removeReference();
}

void MultiPendulums::step() {
	m_world->lock();

	//check if action is still in the world : it should always be, but it's better to check here to make sure
	if (m_marbleAction->getWorld() != HK_NULL) {
		if (receiver.IsAnyDown()) {
			m_marbleAction->getEntity()->activate();
		}

		m_marbleAction->setLeftPressed(receiver.IsKeyDown(EKEY_CODE::KEY_LEFT));
		m_marbleAction->setRightPressed(receiver.IsKeyDown(EKEY_CODE::KEY_RIGHT));
		m_marbleAction->setBrakePressed(receiver.IsKeyDown(EKEY_CODE::KEY_LCONTROL));
		m_marbleAction->setBackwardPressed(receiver.IsKeyDown(EKEY_CODE::KEY_DOWN));
		m_marbleAction->setForwardPressed(receiver.IsKeyDown(EKEY_CODE::KEY_UP));
		m_marbleAction->setJumpPressed(receiver.IsKeyDown(EKEY_CODE::KEY_SPACE));

		if (receiver.IsKeyDown(KEY_KEY_R)) {
			m_marbleAction->reset();
		}
		if (receiver.IsKeyDown(KEY_KEY_Y)) {
			m_marbleAction->getEntity()->deactivate();
		}
	}

	m_world->unlock();
}

void MultiPendulums::runHk() {
	vdb.setVisualDebugger(m_world);

	hkStopwatch stopWatch;
	stopWatch.start();
	hkReal lastTime = stopWatch.getElapsedSeconds();
	hkReal timeStep = 1.f / 60.f;
	int numSteps = int(10.f / timeStep);
	while (!receiver.IsKeyDown(EKEY_CODE::KEY_ESCAPE)) {
		m_world->stepDeltaTime(timeStep);
		
		vdb.step();
		
		runIrr();

		step();

		while (stopWatch.getElapsedSeconds() < lastTime + timeStep);
		lastTime += timeStep;
	}
}

MultiPendulums::~MultiPendulums() {

}

const int MultiPendulums::add_cameraIrr() {
	scene_manager->addCameraSceneNode();
	return 0;
}

const int MultiPendulums::runIrr() {
	device->run();

	driver->beginScene();

	scene_manager->drawAll();
	gui_env->drawAll();

	driver->endScene();

	return 0;
}