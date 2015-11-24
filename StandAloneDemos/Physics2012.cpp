#include "Physics2012.h"

Physics2012::Physics2012() {

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
	
	hkpRigidBody* rigidBody;
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
	rigidBody->removeReference();
}

void Physics2012::stepPhysics() {
	hkStopwatch frameTimer;
	frameTimer.start();

	const hkReal updateFrequency = 1.0f / 60.0f;
	for (int i = 0; i < 100; i++) {
		world->stepDeltaTime(updateFrequency);

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
}

void Physics2012::runHk() {
	initPhysics();
	vdb.setVisualDebugger(world);
	stepPhysics();
}