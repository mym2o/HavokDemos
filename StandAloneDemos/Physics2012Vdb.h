#pragma once

#include "HavokDefinitions.h"

// Physics
#include <Physics2012/Dynamics/World/hkpWorld.h>
#include <Physics2012/Collide/Dispatch/hkpAgentRegisterUtil.h>
#include <Physics2012/Dynamics/Entity/hkpRigidBody.h>
#include <Physics2012/Collide/Shape/Convex/Box/hkpBoxShape.h>
#include <Physics2012/Utilities/Dynamics/Inertia/hkpInertiaTensorComputer.h>

#define HK_EXCLUDE_FEATURE_SerializeDeprecatedPre700
#define HK_EXCLUDE_FEATURE_RegisterVersionPatches
// Vdb needs the reflected classes
//#define HK_EXCLUDE_FEATURE_RegisterReflectedClasses
#define HK_EXCLUDE_FEATURE_MemoryTracker

class Physics2012Vdb : public HavokInterface {
private:
	hkpWorld* m_world;
	hkpRigidBody* rigidBody;
	VisualDebuggerHk vdb;

	void initPhysics();
public:
	Physics2012Vdb();
	virtual ~Physics2012Vdb();

	void initHk();
	void run();
	void quitHk();
};