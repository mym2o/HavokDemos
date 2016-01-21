/***************************************************\
|													|
|           *										|
|         (  `                     )				|
|         )\))(   (        )    ( /(				|
|        ((_)()\  )\ )    (     )(_)) (				|
|        (_()((_)(()/(    )\  '((__)   )\			|
|        |  \/  | )(_)) _((_)) |   )  ((_)			|
|        | |\/| || || || '  \() / /  / _ \			|
|        |_|  |_| \_, ||_|_|_| /___| \___/			|
|                 |__/								|
|													|
\***************************************************/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//																												//
//	This demo shows the use of the hkpPairCollisionFilter. Use this type of filter to disable collisions		//
//	between dedicated pairs of objects.																			//
//																												//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "../../../../../HavokDefinitions.h"
#include "../../../../../IrrInterface.h"
#include "../../../../../Utilities/EventReceiver/DemoEventReceiver.h"

#include <Physics2012\Dynamics\Collide\Filter\Pair\hkpPairCollisionFilter.h>

class PairCollisionFilter : public HavokInterface, IrrInterface {
private:
	hkpWorld* m_world;
	hkpRigidBody* movingBox;

	VisualDebuggerHk vdb;

	DemoEventReceiver receiver;

	void createWorld();
	hkpPairCollisionFilter* createPairCollisionFilter();
	void createFloor();
	hkpRigidBody* create2ndFloor();
	hkpRigidBody* createMovingBox();
public:
	PairCollisionFilter();
	virtual ~PairCollisionFilter();

	void runHk();
	void initHk();
	void quitHk();

	const int add_cameraIrr();
	const int add_gui_elementsIrr();
	const int add_scene_nodesIrr();
	const int runIrr();
};