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
//	This demo shows how to use of the hkpConstraintCollisionFilter. Use this filter to disable collisions		//
//	between all pairs of objects that are connected with a constraint (excluding contact point constraints).	//
//	Note that collisions between unconnected objects will still occur unless you disable them using any			//
//	of the other available filters.																				//
//																												//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "../../../../../HavokDefinitions.h"
#include "../../../../../IrrInterface.h"
#include "../../../../../Utilities/EventReceiver/DemoEventReceiver.h"

#include <vector>

class ConstraintCollisionFilter : public HavokInterface, IrrInterface {
private:
	hkpWorld* m_world;
	std::vector<hkpRigidBody*> m_bodies;
	std::vector<scene::IMeshSceneNode*> g_bodies;

	DemoEventReceiver receiver;

	VisualDebuggerHk vdb;

	void createWorld();
	void createCollisionFilter();
	void createRigidBodies();
	void createHingeConstraint();
public:
	ConstraintCollisionFilter();

	void initHk();
	void runHk();
	void quitHk();

	const int add_cameraIrr();
	const int add_gui_elementsIrr();
	const int add_scene_nodesIrr();
	const int runIrr();
};