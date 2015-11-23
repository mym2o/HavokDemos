#pragma once

#include "../HavokDefinitions.h"

#define HK_EXCLUDE_FEATURE_SerializeDeprecatedPre700
#define HK_EXCLUDE_FEATURE_RegisterVersionPatches
#define HK_EXCLUDE_FEATURE_RegisterReflectedClasses
#define HK_EXCLUDE_FEATURE_MemoryTracker

//Base system
#include <Common\Base\System\Stopwatch\hkStopwatch.h>

//Debug display, handler and process factory
#include <Common\Visualize\hkDebugDisplay.h>
#include <Common\Visualize\hkDebugDisplayHandler.h>
#include <Common\Visualize\hkProcessFactory.h>
#include <Common\Visualize\hkProcess.h>
#include <Common\Visualize\hkProcessContext.h>

//Physics world
#include <Physics2012\Dynamics\World\hkpWorld.h>
#include <Physics2012\Dynamics\Entity\hkpRigidBody.h>
#include <Physics2012\Collide\Dispatch\hkpAgentRegisterUtil.h>
#include <Physics2012\Collide\Shape\Convex\Box\hkpBoxShape.h>
#include <Physics2012\Utilities\Dynamics\Inertia\hkpInertiaTensorComputer.h>

//Visual Debugger includes
#include <Common\Visualize\hkVisualDebugger.h>
#include <Physics2012\Utilities\VisualDebugger\hkpPhysicsContext.h>

#include <Common/Base/Monitor/hkMonitorStream.h>
#include <Common/Base/Monitor/MonitorStreamAnalyzer/hkMonitorStreamAnalyzer.h>

//TO MAKE THE LINKER HAPPY
//#include <Common/Serialize/Util/hkBuiltinTypeRegistry.h>

class Visualize : public HavokInterface {
private:
	hkTransform m_transform;
	hkpWorld* m_world;
	VisualDebuggerHk vdb;

	void initPhysicWorld();
	void destroyPhysics();
	void step(hkReal frameTime);

	void createGround();
	void addDynamicBody();
public:
	Visualize();
	virtual ~Visualize();

	void initHk();
	void quitHk();
	void run();
};