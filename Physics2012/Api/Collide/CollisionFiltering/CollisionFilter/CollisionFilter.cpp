#include "CollisionFilter.h"

#include <Physics2012\Collide\Dispatch\hkpAgentRegisterUtil.h>
#include <Physics2012\Collide\Filter\Group\hkpGroupFilter.h>
#include <Physics2012\Collide\Shape\Convex\Box\hkpBoxShape.h>
#include <Physics2012\Dynamics\Entity\hkpRigidBody.h>
#include <Physics2012\Utilities\Dynamics\Inertia\hkpInertiaTensorComputer.h>

#include <Common\Base\Math\Matrix\hkMatrix3Util.h>

CollisionFilter::CollisionFilter() : IrrInterface(), id_camera(0) {
	device->setEventReceiver(&receiver);
	add_cameraIrr();
	add_gui_elementsIrr();
	add_scene_nodesIrr();
	bodies_color = video::SColor(255, 0, 150, 0);
}

CollisionFilter::~CollisionFilter() {
	g_bodies.clear();
	m_bodies.clear();
}

void CollisionFilter::initHk() {
	memoryRouter = hkMemoryInitUtil::initDefault(hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(1024 * 1024));
	hkBaseSystem::init(memoryRouter, errorReport);

	createWorld();

	m_world->lock();
		hkpAgentRegisterUtil::registerAllAgents(m_world->getCollisionDispatcher());
		createCollisionFilter();
		createBase();
		createGrids();
	m_world->unlock();
}

void CollisionFilter::runHk() {
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

		while (stopWatch.getElapsedSeconds() < lastTime + timeStep);
		lastTime += timeStep;
	}
}

void CollisionFilter::quitHk() {
	m_world->removeReference();
	
	vdb.quitVdb();

	hkBaseSystem::quit();
	hkMemoryInitUtil::quit();

	quit_Irr();
}

void CollisionFilter::createWorld() {
	hkpWorldCinfo info;
	info.setBroadPhaseWorldSize(100.f);
	info.setupSolverInfo(hkpWorldCinfo::SOLVER_TYPE_4ITERS_MEDIUM);
	m_world = new hkpWorld(info);
}

void CollisionFilter::createCollisionFilter() {
	hkpGroupFilter* filter = new hkpGroupFilter();

	//disable all collisions by default
	filter->disableCollisionsUsingBitfield(HK_INT32_MAX, HK_INT32_MAX);

	//enable group 2 against group 4 and 5
	filter->enableCollisionsUsingBitfield(1 << 2, (1 << 4) | (1 << 5));

	//enable collision of the base with everything else
	filter->enableCollisionsUsingBitfield(1U << 31, HK_INT32_MAX);

	//enable group 15 against group 10, 11, 12, 13
	filter->enableCollisionsUsingBitfield(1 << 15, (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13));
	
	m_world->setCollisionFilter(filter);
	filter->removeReference();
}

/**
 * It creates a base which catches the boxes which fall through
 */
void CollisionFilter::createBase() {
	hkVector4 baseRadii(16.f, 0.5f, 16.f);
	hkpBoxShape* baseShape = new hkpBoxShape(baseRadii, 0);

	hkpRigidBodyCinfo boxInfo;
	boxInfo.m_shape = baseShape;
	boxInfo.m_motionType = hkpMotion::MOTION_FIXED;
	boxInfo.m_position = hkVector4(0.f, -2.f, 0.f);
	boxInfo.m_collisionFilterInfo = hkpGroupFilter::calcFilterInfo(31);
	hkpRigidBody* baseRigidBody = new hkpRigidBody(boxInfo);
	m_world->addEntity(baseRigidBody);
	baseRigidBody->removeReference();
	baseShape->removeReference();
}

/**
 * It creates two grids of bodies one above the other (16x16)
 */
void CollisionFilter::createGrids() {
	//create shape
	hkVector4 boxRadii(0.4f, 0.4f, 0.4f);
	hkpBoxShape* cubeShape = new hkpBoxShape(boxRadii, 0);

	//create an array of pairs of boxes: the top box is moveable, and the top two boxes uses the collision filter
	for (int x = 0; x < 16; x++) {
		for (int y = 0; y < 16; y++) {
			//create the bottom static box, using the hkpGroupFilter collision filter
			hkpRigidBodyCinfo boxInfo;
			boxInfo.m_shape = cubeShape;
			boxInfo.m_motionType = hkpMotion::MOTION_FIXED;
			hkMatrix3Util::_setDiagonal(5.f, 5.f, 5.f, boxInfo.m_inertiaTensor);
			boxInfo.m_mass = 5.f;
			boxInfo.m_position.set(x - 7.5f, 2.5f, y - 7.5f);
			boxInfo.m_collisionFilterInfo = hkpGroupFilter::calcFilterInfo(x);
			hkpRigidBody* boxRigidBody = new hkpRigidBody(boxInfo);
			m_world->addEntity(boxRigidBody);
			m_bodies.push_back(boxRigidBody);
			boxRigidBody->removeReference();

			boxInfo.m_position.set(x - 7.5f, 5.f, y - 7.5f);
			boxInfo.m_motionType = hkpMotion::MOTION_SPHERE_INERTIA;
			boxInfo.m_collisionFilterInfo = hkpGroupFilter::calcFilterInfo(y);

			hkpInertiaTensorComputer::setShapeVolumeMassProperties(boxInfo.m_shape, boxInfo.m_mass, boxInfo);
			boxRigidBody = new hkpRigidBody(boxInfo);
			m_world->addEntity(boxRigidBody);
			m_bodies.push_back(boxRigidBody);
			boxRigidBody->removeReference();
		}
	}

	cubeShape->removeReference();

	for (int i = 0; i < m_bodies.size(); i++) {
		scene::IMeshSceneNode* node = scene_manager->addCubeSceneNode(1.f, 0, -1, core::vector3df(), core::vector3df(), core::vector3df(0.4f, 0.4f, 0.4f) * 2.f);
		setColorAndShadow(node, video::SColor(255, 255, 255, 0));
		g_bodies.push_back(node);
	}
}

const int CollisionFilter::runIrr() {
	device->run();

	driver->beginScene();

	for (int i = 0; i < m_bodies.size(); i++) {
		hkVector4f new_pos = m_bodies[i]->getPosition();
		g_bodies[i]->setPosition(core::vector3df(new_pos(0), new_pos(1), new_pos(2)));

		hkQuaternion new_rot = m_bodies[i]->getRotation();
		if (new_rot.hasValidAxis()) {
			hkVector4f axis;
			new_rot.getAxis(axis);
			float angle = core::radToDeg(new_rot.getAngle());
			g_bodies[i]->setRotation(core::vector3df(axis(0), axis(1), axis(2)) * angle);
		}
	}

	//this code allows to change camera finding ICameraSceneNode from id
	if (receiver.IsKeyDown(EKEY_CODE::KEY_KEY_1)) {
		scene_manager->setActiveCamera((scene::ICameraSceneNode*) scene_manager->getSceneNodeFromId(0));
	}
	if (receiver.IsKeyDown(EKEY_CODE::KEY_KEY_2)) {
		scene_manager->setActiveCamera((scene::ICameraSceneNode*) scene_manager->getSceneNodeFromId(1));
	}
	//or finding ICameraSceneNode from array
	if (receiver.IsKeyDown(EKEY_CODE::KEY_KEY_3)) {
		scene_manager->setActiveCamera(cameras[2]);
	}
	if (receiver.IsKeyDown(EKEY_CODE::KEY_KEY_4)) {
		scene_manager->setActiveCamera(cameras[3]);
	}

	if (receiver.IsKeyDown(EKEY_CODE::KEY_KEY_R)) {
		bodies_color.setRed(bodies_color.getRed() + 1 % 255);
		printf("[%d, %d, %d]\n", bodies_color.getRed(), bodies_color.getGreen(), bodies_color.getBlue());
		for (int i = 0; i < g_bodies.size(); i++) {
			scene_manager->getMeshManipulator()->setVertexColors(g_bodies[i]->getMesh(), bodies_color);
		}
	}
	if (receiver.IsKeyDown(EKEY_CODE::KEY_KEY_G)) {
		bodies_color.setGreen(bodies_color.getGreen() + 1 % 255);
		bodies_color.setBlue(bodies_color.getRed());
		printf("[%d, %d, %d]\n", bodies_color.getRed(), bodies_color.getGreen(), bodies_color.getBlue()); 
		for (int i = 0; i < g_bodies.size(); i++) {
			scene_manager->getMeshManipulator()->setVertexColors(g_bodies[i]->getMesh(), bodies_color);
		}
	}
	if (receiver.IsKeyDown(EKEY_CODE::KEY_KEY_B)) {
		bodies_color.setBlue(bodies_color.getBlue() + 1 % 255);
		printf("[%d, %d, %d]\n", bodies_color.getRed(), bodies_color.getGreen(), bodies_color.getBlue());
		for (int i = 0; i < g_bodies.size(); i++) {
			scene_manager->getMeshManipulator()->setVertexColors(g_bodies[i]->getMesh(), bodies_color);
		}
	}

	scene_manager->drawAll();
	gui_env->drawAll();
	driver->endScene();

	return 0;
}

const int CollisionFilter::add_cameraIrr() {
	cameras.push_back(scene_manager->addCameraSceneNode(0, core::vector3df(0.f, 10.f, 30.f), core::vector3df(), 0));
	cameras.push_back(scene_manager->addCameraSceneNode(0, core::vector3df(0.f, 10.f, -30.f), core::vector3df(), 1, false));
	cameras.push_back(scene_manager->addCameraSceneNode(0, core::vector3df(30.f, 10.f, 0.f), core::vector3df(), 2, false));
	cameras.push_back(scene_manager->addCameraSceneNode(0, core::vector3df(-30.f, 10.f, 0.f), core::vector3df(), 3, false));

	return 0;
}

const int CollisionFilter::add_gui_elementsIrr() {
	device->setWindowCaption(L"CollisionFilter Demo - Havok+Irrlicht Engine");
	
	return 0;
}

const int CollisionFilter::add_scene_nodesIrr() {
	//add light
	scene_manager->addLightSceneNode(0, core::vector3df(0, 100, 0), video::SColor(255, 255, 255, 255), 800.f);

	//draw fixed ground
	scene::IMeshSceneNode* ground = scene_manager->addCubeSceneNode(1.f, 0, -1, core::vector3df(0.f, -2.f, 0.f), core::vector3df(), core::vector3df(16.f, .5f, 16.f) * 2.f);
	setColorAndShadow(ground, video::SColor(255, 255, 0, 0));

	return 0;
}