#include "Visualize.h"

Visualize::Visualize() {

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

	hkpRigidBody* body = new hkpRigidBody(bodyInfo);
	bodyInfo.m_shape->removeReference();
	m_world->addEntity(body);
	body->removeReference();
}

void Visualize::destroyPhysics() {
	m_world->removeReference();
}

//Step physics world
void Visualize::step(hkReal frameTime) {
	m_world->stepDeltaTime(frameTime);
}

void Visualize::initHk() {
	hkMemoryRouter* memoryRouter = hkMemoryInitUtil::initDefault(hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(1024 * 1024));
	hkBaseSystem::init(memoryRouter, errorReport);

	const int monitorSize = 128 * 1024;
	hkMonitorStream& stream = hkMonitorStream::getInstance();
	stream.resize(monitorSize);
	stream.reset();
}

void Visualize::run() {
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
}