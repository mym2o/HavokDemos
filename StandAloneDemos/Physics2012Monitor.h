#pragma once

#include "../HavokDefinitions.h"

#include <Common/Base/Monitor/hkMonitorStream.h>
#include <Common/Base/Monitor/MonitorStreamAnalyzer/hkMonitorStreamAnalyzer.h>

// Physics
#include <Physics2012/Dynamics/World/hkpWorld.h>
#include <Physics2012/Collide/Dispatch/hkpAgentRegisterUtil.h>
#include <Physics2012/Dynamics/Entity/hkpRigidBody.h>
#include <Physics2012/Collide/Shape/Convex/Box/hkpBoxShape.h>
#include <Physics2012/Utilities/Dynamics/Inertia/hkpInertiaTensorComputer.h>

#include <Common/Base/System/Stopwatch/hkStopwatch.h>

#define HK_EXCLUDE_FEATURE_SerializeDeprecatedPre700
#define HK_EXCLUDE_FEATURE_RegisterVersionPatches
#define HK_EXCLUDE_FEATURE_MemoryTracker

class Physics2012Monitor : public HavokInterface {
private:
	VisualDebuggerHk vdb;
	hkpWorld* world;

	void initPhysics();
public:
	Physics2012Monitor();
	virtual ~Physics2012Monitor();

	void initHk();
	void runHk();
	void quitHk();
};