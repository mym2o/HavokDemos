#include "PairCollisionFilter.h"

#include <Physics2012\Collide\Dispatch\hkpAgentRegisterUtil.h>
#include <Physics2012\Collide\Shape\Convex\Box\hkpBoxShape.h>
#include <Physics2012\Dynamics\Entity\hkpRigidBody.h>
#include <Physics2012\Utilities\Dynamics\Inertia\hkpInertiaTensorComputer.h>

PairCollisionFilter::PairCollisionFilter() : IrrInterface() {
	device->setEventReceiver(&receiver);
}

PairCollisionFilter::~PairCollisionFilter() {
}

void PairCollisionFilter::initHk() {
	memoryRouter = hkMemoryInitUtil::initDefault(hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(1024 * 1024));
	hkBaseSystem::init(memoryRouter, errorReport);

	createWorld();
	m_world->lock();

	hkpAgentRegisterUtil::registerAllAgents(m_world->getCollisionDispatcher());

	hkpPairCollisionFilter* filter = createPairCollisionFilter();
	createFloor();
	hkpRigidBody* floor = create2ndFloor();
	movingBox = createMovingBox();

	//disable collisions between moving box and the upper floor
	filter->disableCollisionsBetween(floor, movingBox);

	m_world->unlock();
}

void PairCollisionFilter::quitHk() {
	m_world->removeReference();

	vdb.quitVdb();

	hkBaseSystem::quit();
	hkMemoryInitUtil::quit();

	quit_Irr();
}

void PairCollisionFilter::runHk() {
	vdb.setVisualDebugger(m_world);

	add_cameraIrr();
	add_gui_elementsIrr();
	add_scene_nodesIrr();

	hkStopwatch stopWatch;
	stopWatch.start();
	hkReal lastTime = stopWatch.getElapsedSeconds();
	hkReal timeStep = 1.f / 60.f;
	int numSteps = int(10.f / timeStep);
	while(!receiver.IsKeyDown(EKEY_CODE::KEY_ESCAPE)) {
		m_world->stepDeltaTime(timeStep);

		vdb.step();

		runIrr();

		while(stopWatch.getElapsedSeconds() <= lastTime + timeStep);
		lastTime += timeStep;
	}
}

void PairCollisionFilter::createWorld() {
	hkpWorldCinfo info;
	m_world = new hkpWorld(info);
}

/**
 * Creates a default hkpPairCollisionFilter
 */
hkpPairCollisionFilter* PairCollisionFilter::createPairCollisionFilter() {
	hkpPairCollisionFilter* filter = new hkpPairCollisionFilter();
	m_world->setCollisionFilter(filter);
	filter->removeReference();
	return filter;
}

void PairCollisionFilter::createFloor() {
	hkVector4 fixedBoxSize(5.f, .5f, 5.f);
	hkpBoxShape* fixedBoxShape = new hkpBoxShape(fixedBoxSize, 0);

	hkpRigidBodyCinfo info;
	info.m_shape = fixedBoxShape;
	info.m_motionType = hkpMotion::MOTION_FIXED;
	info.m_position.set(0.f, -1.f, 0.f);

	hkpRigidBody* lowerFloor = new hkpRigidBody(info);
	m_world->addEntity(lowerFloor);
	lowerFloor->removeReference();
	fixedBoxShape->removeReference();
}

hkpRigidBody* PairCollisionFilter::create2ndFloor() {
	hkVector4 fixedBoxSize(3.f, .5f, 3.f);
	hkpBoxShape* fixedBoxShape = new hkpBoxShape(fixedBoxSize, 0);

	hkpRigidBodyCinfo info;
	info.m_shape = fixedBoxShape;
	info.m_motionType = hkpMotion::MOTION_FIXED;
	info.m_position.set(0.f, 4.f, 0.f);

	hkpRigidBody* upperFloor = new hkpRigidBody(info);
	m_world->addEntity(upperFloor);
	upperFloor->removeReference();
	fixedBoxShape->removeReference();

	return upperFloor;
}

hkpRigidBody* PairCollisionFilter::createMovingBox() {
	hkVector4 boxSize(.5f, .5f, .5f);
	hkpShape* boxShape = new hkpBoxShape(boxSize, 0);

	hkpRigidBodyCinfo info;
	info.m_shape = boxShape;
	info.m_qualityType = HK_COLLIDABLE_QUALITY_MOVING;

	hkReal boxMass = 1.f;
	hkMassProperties massProperties;
	hkpInertiaTensorComputer::computeBoxVolumeMassProperties(boxSize, boxMass, massProperties);
	info.m_motionType = hkpMotion::MOTION_BOX_INERTIA;
	info.m_mass = boxMass;
	info.m_inertiaTensor = massProperties.m_inertiaTensor;
	info.m_position.set(0.f, 7.f, 0.f);

	hkpRigidBody* boxBody = new hkpRigidBody(info);
	m_world->addEntity(boxBody);

	boxBody->removeReference();
	boxShape->removeReference();

	return boxBody;
}

const int PairCollisionFilter::add_cameraIrr() {
	scene_manager->addCameraSceneNode(0, core::vector3df(10.f, 10.f, 15.f), core::vector3df());

	return 0;
}

const int PairCollisionFilter::add_gui_elementsIrr() {
	device->setWindowCaption(L"PairCollisionFilter Demo - Havok+Irrlicht Engine");

	return 0;
}

const int PairCollisionFilter::add_scene_nodesIrr() {
	scene_manager->addLightSceneNode(0, core::vector3df(0.f, 100.f, 100.f), video::SColorf(1.f, 1.f, 1.f), 800.f);

	scene::IMeshSceneNode* lowerFloor = scene_manager->addCubeSceneNode(1.f, 0, -1, core::vector3df(0.f, -1.f, 0.f), core::vector3df(), core::vector3df(5.f, .5f, 5.f) * 2.f);
	setColorAndShadow(lowerFloor, video::SColor(255, 255, 0, 0));

	scene::IMeshSceneNode* upperFloor = scene_manager->addCubeSceneNode(1.f, 0, -1, core::vector3df(0.f, 4.f, 0.f), core::vector3df(), core::vector3df(3.f, .5f, 3.f) * 2.f);
	setColorAndShadow(upperFloor, video::SColor(255, 0, 255, 0));

	scene_manager->addCubeSceneNode(1.f, 0, 1, core::vector3df(0.f, 7.f, 0.f), core::vector3df());
	setColorAndShadow((scene::IMeshSceneNode*)scene_manager->getSceneNodeFromId(1), video::SColor(255, 0, 0, 255));

	return 0;
}

const int PairCollisionFilter::runIrr() {
	device->run();

	driver->beginScene();
	
	scene_manager->drawAll();
	gui_env->drawAll();

	hkVector4 new_pos = movingBox->getPosition();
	scene::IMeshSceneNode* g_box = (scene::IMeshSceneNode*) scene_manager->getSceneNodeFromId(1);
	g_box->setPosition(core::vector3df(new_pos(0), new_pos(1), new_pos(2)));

	//reset moving box position
	if(receiver.IsKeyDown(EKEY_CODE::KEY_KEY_R)) {
		movingBox->setPosition(hkVector4(0.f, 7.f, 0.f));
	}
	//press space to make flying moving box
	if(receiver.IsKeyDown(EKEY_CODE::KEY_SPACE)) {
		movingBox->setLinearVelocity(hkVector4(0.f, 3.f, 0.f));
	}

	driver->endScene();

	return 0;
}