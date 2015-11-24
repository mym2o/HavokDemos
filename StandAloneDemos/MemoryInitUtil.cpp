#include "MemoryInitUtil.h"

MemoryInitUtil::MemoryInitUtil() {
	
}

MemoryInitUtil::~MemoryInitUtil() {

}

void MemoryInitUtil::initHk() {
	memoryRouter = hkMemoryInitUtil::initDefault(hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(0));
	hkBaseSystem::init(memoryRouter, errorReport);
}

void MemoryInitUtil::quitHk() {
	hkBaseSystem::quit();
	hkMemoryInitUtil::quit();
}

void MemoryInitUtil::runHk() {
	HK_WARN_ALWAYS(0x47625fd12, "Hello World!");
}