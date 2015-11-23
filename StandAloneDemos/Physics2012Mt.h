#pragma once

#include "../HavokDefinitions.h"

#include <Common/Base/System/Hardware/hkHardwareInfo.h>
#include <Common/Base/Thread/Pool/hkCpuThreadPool.h>
#include <Common/Base/Monitor/hkMonitorStream.h>

#include <Physics2012/Collide/hkpCollide.h>										
#include <Physics2012/Collide/Agent/ConvexAgent/SphereBox/hkpSphereBoxAgent.h>	
#include <Physics2012/Collide/Shape/Convex/Box/hkpBoxShape.h>					
#include <Physics2012/Collide/Shape/Convex/Sphere/hkpSphereShape.h>				
#include <Physics2012/Collide/Dispatch/hkpAgentRegisterUtil.h>					

#include <Physics2012/Collide/Query/CastUtil/hkpWorldRayCastInput.h>			
#include <Physics2012/Collide/Query/CastUtil/hkpWorldRayCastOutput.h>			

#include <Physics2012/Dynamics/World/hkpWorld.h>
#include <Physics2012/Dynamics/Entity/hkpRigidBody.h>							
#include <Physics2012/Utilities/Dynamics/Inertia/hkpInertiaTensorComputer.h>	

#define HK_FEATURE_REFLECTION_PHYSICS_2012
#define HK_CLASSES_FILE <Common/Serialize/Classlist/hkClasses.h>
#define HK_EXCLUDE_FEATURE_MemoryTracker
#define HK_EXCLUDE_FEATURE_SerializeDeprecatedPre700
#define HK_EXCLUDE_FEATURE_RegisterVersionPatches
#define HK_EXCLUDE_LIBRARY_hkGeometryUtilities

class Physics2012Mt : public HavokInterface {
private:
	hkpWorld* m_world;
	hkpRigidBody* m_ball;
	VisualDebuggerHk vdb;
	hkJobQueue* jobQueue;
	hkThreadPool* threadPool;

	void initThreads();
	void initPhysics();
	void createGroundPh();
	void createWallsPh(hkVector4& posy);
	void createSingleWallPh(const int height, const int length, const hkVector4& position, const hkReal gapWidth, hkpConvexShape* box, hkVector4Parameter halfExtents);
	void createBallPh(hkVector4& posy);
public:
	Physics2012Mt();
	virtual ~Physics2012Mt();

	void initHk();
	void run();
	void quitHk();
};