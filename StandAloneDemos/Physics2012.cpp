#include "Physics2012.h"

Physics2012::Physics2012() : IrrInterface() {

}

Physics2012::~Physics2012() {

}

void Physics2012::initHk() {
	hkMemoryRouter* memoryRouter = hkMemoryInitUtil::initDefault(hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(1024 * 1024));
	hkBaseSystem::init(memoryRouter, errorReport);
}

void Physics2012::initPhysics() {
	world = new hkpWorld(hkpWorldCinfo());
	hkpAgentRegisterUtil::registerAllAgents(world->getCollisionDispatcher());
	
	hkVector4 halfExtents(0.5f, 1.0f, 1.5f);
	hkpBoxShape* boxShape = new hkpBoxShape(halfExtents);

	hkpRigidBodyCinfo bodyCInfo;
	bodyCInfo.m_shape = boxShape;

	const hkReal boxMass = 10.0f;
	hkMassProperties massProperties;
	hkpInertiaTensorComputer::computeShapeVolumeMassProperties(boxShape, boxMass, massProperties);

	bodyCInfo.setMassProperties(massProperties);

	rigidBody = new hkpRigidBody(bodyCInfo);
	boxShape->removeReference();
	world->addEntity(rigidBody);

	cube = scene_manager->addCubeSceneNode(1.f, 0, -1, core::vector3df(), core::vector3df(), core::vector3df(1.f, 2.f, 3.f));
	cube->setMaterialFlag(video::EMF_LIGHTING, false);
	scene_manager->getMeshManipulator()->setVertexColors(cube->getMesh(), video::SColor(255, 255, 0, 0));
}

void Physics2012::stepPhysics() {
	hkStopwatch frameTimer;
	frameTimer.start();

	const hkReal updateFrequency = 1.0f / 60.0f;
	for (int i = 0; i < 100; i++) {
		world->stepDeltaTime(updateFrequency);

		runIrr();

		vdb.step();

		while (frameTimer.getElapsedSeconds() < updateFrequency);

		frameTimer.reset();
		frameTimer.start();
	}
}

void Physics2012::quitPhysics() {
	world->removeReference();
}

void Physics2012::quitHk() {
	quitPhysics();
	vdb.quitVdb();
	hkBaseSystem::quit();
	hkMemoryInitUtil::quit();

	quit_Irr();
}

void Physics2012::runHk() {
	initPhysics();

	add_gui_elementsIrr();
	add_cameraIrr();

	vdb.setVisualDebugger(world);
	stepPhysics();
}

//---------------------------Irrlicht-------------------------------//

const int Physics2012::add_cameraIrr() {
	scene_manager->addCameraSceneNode(0, core::vector3df(10.f, 19.f, -20.f), core::vector3df());

	return 0;
}

const int Physics2012::add_gui_elementsIrr() {
	device->setWindowCaption(L"Physics2012 Demo - Havok+Irrlicht Engine");

	return 0;
}

const int Physics2012::add_scene_nodesIrr() {
	return 0;
}

const int Physics2012::runIrr() {
	device->run();

	driver->beginScene();

	scene_manager->drawAll();
	gui_env->drawAll();

	hkVector4 newpos_hk = rigidBody->getPosition();
	cube->setPosition(core::vector3df(newpos_hk(0), newpos_hk(1), newpos_hk(2)));

	driver->endScene();

	return 0;
}