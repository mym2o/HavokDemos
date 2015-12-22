#include "ConstraintCollisionFilter.h"

#include <Physics2012\Collide\Dispatch\hkpAgentRegisterUtil.h>
#include <Physics2012\Dynamics\Collide\Filter\Constraint\hkpConstraintCollisionFilter.h>
#include <Physics2012\Collide\Shape\Convex\Box\hkpBoxShape.h>
#include <Physics2012\Dynamics\Entity\hkpRigidBody.h>
#include <Physics2012\Utilities\Dynamics\Inertia\hkpInertiaTensorComputer.h>
#include <Physics/Constraint/Data/Hinge/hkpHingeConstraintData.h>

ConstraintCollisionFilter::ConstraintCollisionFilter() : IrrInterface() {
	device->setEventReceiver(&receiver);
}

void ConstraintCollisionFilter::initHk() {
	memoryRouter = hkMemoryInitUtil::initDefault(hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(1024 * 1024));
	hkBaseSystem::init(memoryRouter, errorReport);

	createWorld();
	
	m_world->lock();
		hkpAgentRegisterUtil::registerAllAgents(m_world->getCollisionDispatcher());
		createCollisionFilter();
		createRigidBodies();
		createHingeConstraint();
	m_world->unlock();
}

void ConstraintCollisionFilter::runHk() {
	add_cameraIrr();
	add_gui_elementsIrr();
	add_scene_nodesIrr();

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

void ConstraintCollisionFilter::quitHk() {
	m_world->removeReference();

	vdb.quitVdb();

	hkBaseSystem::quit();
	hkMemoryInitUtil::quit();

	quit_Irr();
}

void ConstraintCollisionFilter::createWorld() {
	hkpWorldCinfo info;
	info.setupSolverInfo(hkpWorldCinfo::SOLVER_TYPE_4ITERS_MEDIUM);
	info.setBroadPhaseWorldSize(100.f);

	m_world = new hkpWorld(info);
}

/**
 * Creates constrained-system collision filter
 */
void ConstraintCollisionFilter::createCollisionFilter() {
	hkpConstraintCollisionFilter* filter = new hkpConstraintCollisionFilter();
	filter->updateFromWorld(m_world);
	m_world->setCollisionFilter(filter);
	filter->removeReference();
}

void ConstraintCollisionFilter::createRigidBodies() {
	hkVector4 halfSize(0.5f, 0.5f, 0.5f);
	hkVector4 size;
	size.setMul4(2.f, halfSize);
	hkVector4 position(size(0), -size(1) - 0.1f, 0.f);

	hkpBoxShape* boxShape = new hkpBoxShape(halfSize, 0);

	//fixed body
	hkpRigidBodyCinfo info;
	info.m_position.set(0.f, 0.f, 0.f);
	info.m_shape = boxShape;
	info.m_motionType = hkpMotion::MOTION_FIXED;
	m_bodies.push_back(new hkpRigidBody(info));
	m_world->addEntity(m_bodies[0]);
	m_bodies[0]->removeReference();

	g_bodies.push_back(scene_manager->addCubeSceneNode(1.f, 0, -1, core::vector3df(), core::vector3df(), core::vector3df(0.5f, 0.5f, 0.5f) * 2.f));
	setColorAndShadow(g_bodies[0], video::SColor(255, 255, 0, 0));

	//movable body
	info.m_position = position;
	info.m_mass = 10.f;
	hkMassProperties massProperties;
	hkpInertiaTensorComputer::computeBoxVolumeMassProperties(halfSize, info.m_mass, massProperties);
	info.m_inertiaTensor = massProperties.m_inertiaTensor;
	info.m_centerOfMass = massProperties.m_centerOfMass;
	info.m_motionType = hkpMotion::MOTION_BOX_INERTIA;

	m_bodies.push_back(new hkpRigidBody(info));
	m_world->addEntity(m_bodies[1]);
	m_bodies[1]->removeReference();

	g_bodies.push_back(scene_manager->addCubeSceneNode(1.f, 0, -1, core::vector3df(), core::vector3df(), core::vector3df(0.5f, 0.5f, 0.5f) * 2.f));
	setColorAndShadow(g_bodies[1], video::SColor(255, 0, 255, 0));

	boxShape->removeReference();
}

void ConstraintCollisionFilter::createHingeConstraint() {
	hkpHingeConstraintData* hc = new hkpHingeConstraintData();
	hkVector4 halfSize(0.5f, 0.5f, 0.5f), size, pivot;
	size.setMul4(2.f, halfSize);
	hkVector4 position(size(0), -size(1) - 0.1f, 0.f);

	pivot.setAdd4(position, halfSize);

	//move pivot to center of side on cube
	pivot(0) -= size(0);
	pivot(2) -= halfSize(2);

	hkVector4 axis(0.f, 0.f, 1.f);

	//create constraint
	hc->setInWorldSpace(m_bodies[0]->getTransform(), m_bodies[1]->getTransform(), pivot, axis);

	hkpConstraintInstance* constraint = new hkpConstraintInstance(m_bodies[0], m_bodies[1], hc);
	m_world->addConstraint(constraint);
	constraint->removeReference();

	hc->removeReference();
}

const int ConstraintCollisionFilter::add_cameraIrr() {
	scene_manager->addCameraSceneNode(0, core::vector3df(0.f, 0.f, -10.f), core::vector3df());

	return 0;
}

const int ConstraintCollisionFilter::add_gui_elementsIrr() {
	device->setWindowCaption(L"ConstraintCollisionFilter Demo - Havok+Irrlicht Engine");

	return 0;
}

const int ConstraintCollisionFilter::add_scene_nodesIrr() {
	//add light
	scene_manager->addLightSceneNode(0, core::vector3df(0, 100, 0), video::SColor(255, 255, 255, 255), 800.f);

	return 0;
}

const int ConstraintCollisionFilter::runIrr() {
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

	scene_manager->drawAll();
	gui_env->drawAll();
	driver->endScene();

	return 0;
}