#include "Visualize.h"

Visualize::Visualize() : IrrInterface() {
	
}

Visualize::~Visualize() {

}

void Visualize::initPhysicWorld() {
	m_transform.setIdentity();

	//Make a new physics world
	m_world = new hkpWorld(hkpWorldCinfo());

	//Register all collision agents
	hkpAgentRegisterUtil::registerAllAgents(m_world->getCollisionDispatcher());

	createGround();
	addDynamicBody();
}

//Create a fixed ground body
void Visualize::createGround() {
	hkpRigidBodyCinfo bodyInfo;
	hkVector4 halfExtents(20, 0.5f, 20);
	bodyInfo.m_shape = new hkpBoxShape(halfExtents);
	bodyInfo.m_position.set(0, -0.5f, 0);
	bodyInfo.m_motionType = hkpMotion::MOTION_FIXED;

	//we don't need to save this node, because it is fixed (no physics applyied on it)
	scene::IMeshSceneNode* ground_node = scene_manager->addCubeSceneNode(1.0f, 0, -1, core::vector3df(0, -0.5f, 0), core::vector3df(0, 0, 0), core::vector3df(40, 1, 40));
	if (ground_node) {
		ground_node->addShadowVolumeSceneNode();
		//we have no texture, than we need the light to see our ground
		//ground_node->setMaterialFlag(video::EMF_LIGHTING, false);
		ground_node->setMaterialFlag(video::EMF_ANTI_ALIASING, true);
		ground_node->setMaterialType(video::EMT_SOLID);
		ground_node->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);
		scene_manager->getMeshManipulator()->setVertexColors(ground_node->getMesh(), video::SColor(0, 255, 0, 0));
	}

	hkpRigidBody* body = new hkpRigidBody(bodyInfo);
	bodyInfo.m_shape->removeReference();
	m_world->addEntity(body);
	body->removeReference();
}

//Create a dynamic falling body
void Visualize::addDynamicBody() {
	hkpRigidBodyCinfo bodyInfo;
	hkVector4 halfExtents(0.5f, 0.5f, 0.5f);
	bodyInfo.m_shape = new hkpBoxShape(halfExtents);
	hkMassProperties massProperties;
	hkpInertiaTensorComputer::computeShapeVolumeMassProperties(bodyInfo.m_shape, 1, massProperties);
	bodyInfo.setMassProperties(massProperties);
	bodyInfo.m_position.set(0, 10, 0);
	bodyInfo.m_angularVelocity.set(-2, -2, -2);
	bodyInfo.m_linearVelocity.set(5, -5, 5);

	//add a 3d dynamic falling body
	dynamicBody_irr = scene_manager->addCubeSceneNode(1.0f, 0, -1, core::vector3df(0, 10, 0), core::vector3df(0, 0, 0), core::vector3df(1, 1, 1));
	scene_manager->getMeshManipulator()->setVertexColors(dynamicBody_irr->getMesh(), video::SColor(0, 0, 255, 0));
	//dynamicBody_irr->setMaterialFlag(video::EMF_LIGHTING, false);
	dynamicBody_irr->addShadowVolumeSceneNode();
	dynamicBody_irr->setMaterialFlag(video::EMF_ANTI_ALIASING, true);
	dynamicBody_irr->setMaterialType(video::EMT_SOLID);
	dynamicBody_irr->setMaterialFlag(video::EMF_NORMALIZE_NORMALS, true);

	dynamicBody_hk = new hkpRigidBody(bodyInfo);
	bodyInfo.m_shape->removeReference();
	m_world->addEntity(dynamicBody_hk);
}

void Visualize::destroyPhysics() {
	m_world->removeReference();
}

//Step physics world
void Visualize::step(hkReal frameTime) {
	m_world->stepDeltaTime(frameTime);

	runIrr();
}

void Visualize::initHk() {
	hkMemoryRouter* memoryRouter = hkMemoryInitUtil::initDefault(hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(1024 * 1024));
	hkBaseSystem::init(memoryRouter, errorReport);

	const int monitorSize = 128 * 1024;
	hkMonitorStream& stream = hkMonitorStream::getInstance();
	stream.resize(monitorSize);
	stream.reset();
}

void Visualize::runHk() {
	//---Irrlicht initialization---//
	add_gui_elementsIrr();
	add_cameraIrr();
	add_scene_nodesIrr();
	//---end---//

	initPhysicWorld();
	
	vdb.setVisualDebugger(m_world);

	hkStopwatch frameTimer;
	frameTimer.start();

	const hkReal frameTime = 1 / 60.0f;
	const hkReal numFrames = 600;
	for (int i = 0; i < numFrames; i++) {
		step(frameTime);
		
		vdb.step();

		HK_DISPLAY_FRAME(m_transform, 5);

		while (frameTimer.getElapsedSeconds() < frameTime);

		frameTimer.reset();
		frameTimer.start();
	}
}

void Visualize::quitHk() {
	vdb.quitVdb();

	destroyPhysics();

	hkBaseSystem::quit();
	hkMemoryInitUtil::quit();

	quit_Irr();
}

//---------------------------Irrlicht-------------------------------//

const int Visualize::add_cameraIrr() {
	scene_manager->addCameraSceneNode(0, core::vector3df(10, 30, 10), core::vector3df(0, 0, 0));

	return 0;
}

const int Visualize::add_gui_elementsIrr() {
	device->setWindowCaption(L"Visualize - Havok+Irrlicht Engines");

	return 0;
}

const int Visualize::add_scene_nodesIrr() {
	scene_manager->addLightSceneNode(0, core::vector3df(0, 30, 0), video::SColorf(1.0f, 0.6f, 0.7f, 1.0f), 800.0f);
	return 0;
}

const int Visualize::runIrr() {
	device->run();

	driver->beginScene(true, true, video::SColor(0, 100, 100, 100));

	//drawing x, y, z axis
	driver->setTransform(video::ETS_WORLD, core::matrix4());
	driver->draw3DLine(core::vector3df(0, 0, 0), core::vector3df(10, 0, 0), video::SColor(255, 255, 0, 255));
	driver->draw3DLine(core::vector3df(0, 0, 0), core::vector3df(0, 10, 0), video::SColor(255, 0, 255, 0));
	driver->draw3DLine(core::vector3df(0, 0, 0), core::vector3df(0, 0, 10), video::SColor(255, 0, 0, 255));
	//---

	scene_manager->drawAll();
	gui_env->drawAll();

	//update 3d body position & rotation (i'm not be able to update the rotation)
	hkVector4 newpos_hk = dynamicBody_hk->getPosition();
	core::vector3df newpos_irr((float)newpos_hk(0), (float)newpos_hk(1), (float)newpos_hk(2));
	dynamicBody_irr->setPosition(newpos_irr);

	hkQuaternion newrot_hk = dynamicBody_hk->getRotation();
	if (newrot_hk.hasValidAxis()) {
		hkVector4f axis;
		newrot_hk.getAxis(axis);
		float angle = newrot_hk.getAngle() * core::RADTODEG;
		dynamicBody_irr->setRotation(core::vector3df(axis(0), axis(1), axis(2)) * angle);
	}

	driver->endScene();

	return 0;
}