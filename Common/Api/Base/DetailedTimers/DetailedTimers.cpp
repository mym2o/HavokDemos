#include "DetailedTimers.h"

DetailedTimers::DetailedTimers() {
	m_step = 0;
}

DetailedTimers::~DetailedTimers() {

}

void DetailedTimers::initHk() {
	memoryRouter = hkMemoryInitUtil::initDefault(hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(1024 * 1024));
	hkBaseSystem::init(memoryRouter, errorReport);
}

void DetailedTimers::run() {
	timeDetailedTimers(100, "DetailedTimers_stats.txt");
}

void DetailedTimers::quitHk() {
	hkBaseSystem::quit();
	hkMemoryInitUtil::quit();
}

void DetailedTimers::timeDetailedTimers(const int iterations, const char* fileName) {
	hkMonitorStreamFrameInfo frameInfo;
	frameInfo.m_indexOfTimer0 = 0;
	frameInfo.m_indexOfTimer1 = -1;
	frameInfo.m_heading = "usecs";
	frameInfo.m_timerFactor0 = 1e6f / hkReal(hkStopwatch::getTicksPerSecond());

	int numThreads = 1;
	
	//--------------------------------------------------------//
	//I don't want a multithreaded demo
	bool isPhysicsDemo = false;
	if (isPhysicsDemo) {
		//use a method like "initThreads" in Physics2012Mt
	}
	//--------------------------------------------------------//

	//ten millions
	hkMonitorStreamAnalyzer streamAnalyzer(10000000, numThreads);

	for (int i = 0; i < iterations; i++) {
		//setup the timerinfo and memory on a per frame bases
		hkMonitorStream& stream = hkMonitorStream::getInstance();
		stream.resize(2 * 1024 * 1024);	//2MB
		stream.reset();

		//step the demo (nothing to step)

		if (numThreads > 1) {
			//manage this case
		}

		frameInfo.m_threadId = 0;
		streamAnalyzer.captureFrameDetails(stream.getStart(), stream.getEnd(), frameInfo);
	}

	//write the results to a file
	hkStringBuf msg;
	msg.printf("Locking to write...\n");
	errorReport(msg.cString(), HK_NULL);

	hkReferencedObject::lockAll();

	hkOstream ostr(fileName);
	ostr << "DetailedTimers\t Timers: \n";
	streamAnalyzer.writeStatistics(ostr);

	msg.printf("Writing stats...\n");
	errorReport(msg.cString(), HK_NULL);

	hkReferencedObject::unlockAll();

	msg.printf("Done!\n");
	errorReport(msg.cString());
}