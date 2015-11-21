#include "Physics2012Monitor.h"

Physics2012Monitor::Physics2012Monitor() {

}

Physics2012Monitor::~Physics2012Monitor() {

}

void Physics2012Monitor::initHk() {
	memoryRouter = hkMemoryInitUtil::initDefault(hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(1024 * 1024));
	hkBaseSystem::init(memoryRouter, errorReport);
}

void Physics2012Monitor::run() {
	hkMonitorStreamAnalyzer monitorAnalyzer(1024 * 1024);
	hkMonitorStream& monitorStream = hkMonitorStream::getInstance();
	monitorStream.resize(500 * 1024);	//500K for timer info
	monitorStream.reset();

	initPhysics();

	vdb.setVisualDebugger(world);

	hkStopwatch frameTimer;
	frameTimer.start();
	const hkReal updateFrequency = 1 / 60.0f;
	for (int i = 0; i < 100; i++) {
		world->stepDeltaTime(updateFrequency);

		vdb.step();

		hkMonitorStreamFrameInfo frameInfo;
		frameInfo.m_indexOfTimer0 = 0;
		frameInfo.m_indexOfTimer1 = -1;
		frameInfo.m_timerFactor0 = 1e6f / float(hkStopwatch::getTicksPerSecond());
		frameInfo.m_timerFactor1 = 1;
		frameInfo.m_absoluteTimeCounter = hkMonitorStreamFrameInfo::ABSOLUTE_TIME_TIMER_0;

		monitorAnalyzer.captureFrameDetails(monitorStream.getStart(), monitorStream.getEnd(), frameInfo);
		monitorStream.reset();

		while (frameTimer.getElapsedSeconds() < updateFrequency);
		frameTimer.reset();
		frameTimer.start();
	}

	hkOstream stream("monitorAnalyzerOutput.txt");
	monitorAnalyzer.writeStatistics(stream);
}

void Physics2012Monitor::quitHk() {
	world->removeReference();
	vdb.quitVdb();
	hkBaseSystem::quit();
	hkMemoryInitUtil::quit();
}

void Physics2012Monitor::initPhysics() {
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