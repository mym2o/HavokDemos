#pragma once

#include "../HavokDefinitions.h"

// Also we're not using any serialization/versioning so we don't need any of these.
#define HK_EXCLUDE_FEATURE_SerializeDeprecatedPre700
#define HK_EXCLUDE_FEATURE_RegisterVersionPatches
#define HK_EXCLUDE_FEATURE_RegisterReflectedClasses
#define HK_EXCLUDE_FEATURE_MemoryTracker

class MemoryInitUtil : public HavokInterface {
	public:
		MemoryInitUtil();
		~MemoryInitUtil();

		void initHk();
		void quitHk();
		void run();
};