/*
*
* Confidential Information of Telekinesys Research Limited (t/a Havok). Not for disclosure or distribution without Havok's
* prior written consent. This software contains code, techniques and know-how which is confidential and proprietary to Havok.
* Product and Trade Secret source code contains trade secrets of Havok. Havok Software (C) Copyright 1999-2014 Telekinesys Research Limited t/a Havok. All Rights Reserved. Use of this software is subject to the terms of an end user license agreement.
*
*/

#pragma once

#include <Common/Base/keycode.cxx>

#include <Common/Base/hkBase.h>
#include <Common/Base/Memory/System/Util/hkMemoryInitUtil.h>
#include <Common/Base/Memory/Allocator/Malloc/hkMallocAllocator.h>
#include <Common/Base/Fwd/hkcstdio.h>

// We're not using anything product specific yet. We undef these so we don't get the usual
// product initialization for the products.
#undef HK_FEATURE_PRODUCT_AI
#undef HK_FEATURE_PRODUCT_ANIMATION
#undef HK_FEATURE_PRODUCT_CLOTH
#undef HK_FEATURE_PRODUCT_DESTRUCTION_2012
#undef HK_FEATURE_PRODUCT_DESTRUCTION
#undef HK_FEATURE_PRODUCT_BEHAVIOR
//#undef HK_FEATURE_PRODUCT_PHYSICS_2012
#undef HK_FEATURE_PRODUCT_PHYSICS

// This include generates an initialization function based on the products
// and the excluded features.
#include <Common/Base/Config/hkProductFeatures.cxx>


// Platform specific initialization
#include <Common/Base/System/Init/PlatformInit.cxx>

static void HK_CALL errorReport(const char* msg, void* userContext = HK_NULL)
{
	using namespace std;
	printf("%s\n", msg);
}

class HavokInterface {
protected:
	hkMemoryRouter* memoryRouter;
public:
	HavokInterface() {}
	virtual ~HavokInterface() {}

	virtual void initHk() = 0;
	virtual void quitHk() = 0;
	virtual void run() = 0;
};

//Visual Debugger includes
#include <Common\Visualize\hkVisualDebugger.h>
#include <Physics2012\Utilities\VisualDebugger\hkpPhysicsContext.h>

#include <Physics2012/Dynamics/World/hkpWorld.h>

class VisualDebuggerHk {
protected:
	hkVisualDebugger* visualDebugger;
	hkArray<hkProcessContext*> m_contexts;
	hkpPhysicsContext* physicsContext;
public:
	void setVisualDebugger(hkpWorld* world, const bool isMultiThreaded = false) {
		hkDebugDisplayProcess::registerProcess();

		hkpPhysicsContext::registerAllPhysicsProcesses();
		physicsContext = new hkpPhysicsContext();
		physicsContext->addWorld(world);
		m_contexts.pushBack(physicsContext);

		if (isMultiThreaded) {
			world->unmarkForWrite();
		}

		visualDebugger = new hkVisualDebugger(m_contexts);
		visualDebugger->serve();
	}

	void step() {
		visualDebugger->step();
		hkMonitorStream::getInstance().reset();
	}

	void quitVdb() {
		visualDebugger->removeReference();

		for (int i = 0; i < m_contexts.getSize(); i++) {
			delete m_contexts[i];
		}
	}

	hkpPhysicsContext* getVDBContext() const {
		return physicsContext;
	}
};