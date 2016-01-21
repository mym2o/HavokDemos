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
//	This demo illustrates how to maintain contact state using a contactListener.								//
//	The cylinder is colored according to the triangles it is currently in contact with.							//
//	Changes in state are reported to the console.																//
//																												//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "../../../../../HavokDefinitions.h"
#include "../../../../../IrrInterface.h"

#include "../../../../../Utilities/EventReceiver/DemoEventReceiver.h"

#include "MyContactStateListener.h"

#include <Physics2012\Dynamics\Collide\ContactListener\hkpContactListener.h>

class ContactState : public HavokInterface, IrrInterface {
private:
	hkpWorld* m_world;
	hkpRigidBody* m_body;
	hkArray<hkpShapeKey> m_shapeKeys;
	MyContactStateListener* m_listener;
	hkColor::Argb average_color;

	DemoEventReceiver receiver;

	VisualDebuggerHk vdb;

	void createWorld();
	void createMesh();
	void createCapsule();

	void step(const hkReal& timeStep);
public:
	ContactState();
	virtual ~ContactState();

	void runHk();
	void quitHk();
	void initHk();

	const int add_cameraIrr();
	const int add_gui_elementsIrr();
	const int add_scene_nodesIrr();
	const int runIrr();
};