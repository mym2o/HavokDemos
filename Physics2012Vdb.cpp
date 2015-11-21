#include "Physics2012Vdb.h"

Physics2012Vdb::Physics2012Vdb() {

}

Physics2012Vdb::~Physics2012Vdb() {

}

void Physics2012Vdb::initHk() {
	memoryRouter = hkMemoryInitUtil::initDefault(hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(1024 * 1024));
	hkBaseSystem::init(memoryRouter, errorReport);

	// Initialize the monitor stream so that we can send timing information to the Visual Debugger.
	const int monitorSize = 128 * 1024;
	hkMonitorStream& stream = hkMonitorStream::getInstance();
	stream.resize(monitorSize);
	stream.reset();
}

void Physics2012Vdb::run() {
	initPhysics();

	vdb.setVisualDebugger(m_world);

	hkStopwatch stopWatch;
	stopWatch.start();

	hkReal lastTime = stopWatch.getElapsedSeconds();

	const int numStepsPerSecond = 60;
	const hkReal timeStep = 1.0f / hkReal(numStepsPerSecond);

	for (int i = 0; i < numStepsPerSecond * 10; i++) {
		m_world->stepDeltaTime(timeStep);

		vdb.step();

		hkMonitorStream::getInstance().reset();

		//display the position of the rigid body every second
		if (i % 60 == 0) {
			hkVector4 pos = rigidBody->getPosition();
			printf("[%f, %f, %f]\n", pos(0), pos(1), pos(2));
		}

		while (stopWatch.getElapsedSeconds() < lastTime + timeStep);

		lastTime += timeStep;
	}
}

void Physics2012Vdb::quitHk() {
	rigidBody->removeReference();
	m_world->removeReference();

	vdb.quitVdb();

	hkBaseSystem::quit();
	hkMemoryInitUtil::quit();
}

void Physics2012Vdb::initPhysics() {
	m_world = new hkpWorld(hkpWorldCinfo());
	//disable world gravity
	m_world->setGravity(hkVector4::getZero());

	hkpAgentRegisterUtil::registerAllAgents(m_world->getCollisionDispatcher());
	
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

	m_world->addEntity(rigidBody);
}