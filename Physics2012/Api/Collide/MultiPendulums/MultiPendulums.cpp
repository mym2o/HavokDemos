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

	m_bodies.push_back(characterRigidBody);
	scene::IMeshSceneNode* g_character = scene_manager->addCubeSceneNode(.7f, 0, -1, core::vector3df(0.f, 3.f, 5.f));
	setColorAndShadow(g_character, video::SColor(255, 0, 255, 0));
	g_bodies.push_back(g_character);

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
		
		m_bodies.push_back(body);
		scene::IMeshSceneNode* g_pend = scene_manager->addCubeSceneNode(.4f, 0, -1, core::vector3df(boxPos(0), boxPos(1), boxPos(2)));
		setColorAndShadow(g_pend, video::SColor(255, 0, 0, 255));
		g_bodies.push_back(g_pend);
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
	add_scene_nodesIrr();
	add_cameraIrr();

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
	scene_manager->addCameraSceneNode(0, core::vector3df(33.f, 21.f, -24.f), core::vector3df());
	//scene_manager->addCameraSceneNodeFPS(0, 100, 0.01f);
	return 0;
}

const int MultiPendulums::runIrr() {
	device->run();

	driver->beginScene();

	scene_manager->drawAll();
	gui_env->drawAll();

	hkVector4 start_pos = m_marbleAction->getStartVector(), end_pos = m_marbleAction->getEndVector();
	//FPS customized camera
	scene_manager->getActiveCamera()->setPosition(core::vector3df(start_pos(0), start_pos(1), start_pos(2)));
	scene_manager->getActiveCamera()->setTarget(core::vector3df(end_pos(0), end_pos(1), end_pos(2)));
	//---

	for (int i = 0; i < m_bodies.size(); i++) {
		hkVector4f bodyPos = m_bodies[i]->getPosition();
		g_bodies[i]->setPosition(core::vector3df(bodyPos(0), bodyPos(1), bodyPos(2)));

		hkQuaternion new_hk_rot = m_bodies[i]->getRotation();
		if (new_hk_rot.hasValidAxis()) {
			hkVector4f axis;
			new_hk_rot.getAxis(axis);
			float angle = core::radToDeg(new_hk_rot.getAngle());
			g_bodies[i]->setRotation(core::vector3df(axis(0), axis(1), axis(2)) * angle);
		}
	}

	driver->endScene();

	return 0;
}

const int MultiPendulums::add_scene_nodesIrr() {
	//add light
	scene_manager->addLightSceneNode(0, core::vector3df(0, 100, 0), video::SColor(255, 255, 255, 255), 800.f);

	//draw fixed ground
	scene::IMeshSceneNode* ground = scene_manager->addCubeSceneNode(1.f, 0, -1, core::vector3df(0.f, -0.5f, 0.f), core::vector3df(), core::vector3df(25.f, 1.f, 25.f) * 2.f);
	setColorAndShadow(ground, video::SColor(255, 255, 0, 0));

	ground = scene_manager->addCubeSceneNode(1.f, 0, -1, core::vector3df(-25.f, 0.5f, 0.f), core::vector3df(), core::vector3df(0.5f, 0.5f, 25.f) * 2.f);
	setColorAndShadow(ground, video::SColor(255, 255, 0, 0));

	ground = scene_manager->addCubeSceneNode(1.f, 0, -1, core::vector3df(25.f, 0.5f, 0.f), core::vector3df(), core::vector3df(0.5f, 0.5f, 25.f) * 2.f);
	setColorAndShadow(ground, video::SColor(255, 255, 0, 0));

	ground = scene_manager->addCubeSceneNode(1.f, 0, -1, core::vector3df(0.f, 0.5f, 25.f), core::vector3df(), core::vector3df(25.f, 0.5f, 0.5f) * 2.f);
	setColorAndShadow(ground, video::SColor(255, 255, 0, 0));

	ground = scene_manager->addCubeSceneNode(1.f, 0, -1, core::vector3df(0.f, 0.5f, -25.f), core::vector3df(), core::vector3df(25.f, 0.5f, 0.5f) * 2.f);
	setColorAndShadow(ground, video::SColor(255, 255, 0, 0));
	//end fixed ground

	return 0;
}

const int MultiPendulums::add_gui_elementsIrr() {
	device->setWindowCaption(L"MultiPendulums Demo - Havok+Irrlicht Engine");
	return 0;
}