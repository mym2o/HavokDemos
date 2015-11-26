#pragma once

#include "../HavokDefinitions.h"
#include "../IrrInterface.h"

#include <Physics2012\Dynamics\World\hkpWorld.h>
#include <Physics2012\Collide\Dispatch\hkpAgentRegisterUtil.h>
#include <Physics2012\Dynamics\Entity\hkpRigidBody.h>
#include <Physics2012\Collide\Shape\Convex\Box\hkpBoxShape.h>
#include <Physics2012\Utilities\Dynamics\Inertia\hkpInertiaTensorComputer.h>

#include <Common\Base\System\Stopwatch\hkStopwatch.h>

#define HK_EXCLUDE_FEATURE_SerializeDeprecatedPre700
#define HK_EXCLUDE_FEATURE_RegisterVersionPatches
#define HK_EXCLUDE_FEATURE_MemoryTracker

class Physics2012 : public HavokInterface, IrrInterface {
private:
	hkpWorld* world;
	hkpRigidBody* rigidBody;
	scene::IMeshSceneNode* cube;
	VisualDebuggerHk vdb;

	void initPhysics();
	void stepPhysics();
	void quitPhysics();

	const int add_cameraIrr();
	const int runIrr();
	const int add_scene_nodesIrr();
	const int add_gui_elementsIrr();
public:
	Physics2012();
	virtual ~Physics2012();

	void initHk();
	void runHk();
	void quitHk();
};