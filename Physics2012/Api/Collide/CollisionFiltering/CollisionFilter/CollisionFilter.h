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
//	A demo which creates an array of box piles. Each pile is used to show how the hkpGroupFilter works.			//
//																												//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "../../../../../HavokDefinitions.h"
#include "../../../../../IrrInterface.h"
#include "../../../../../Utilities/EventReceiver/DemoEventReceiver.h"

#include <vector>

class CollisionFilter : public HavokInterface, IrrInterface {
private:
	hkpWorld* m_world;
	VisualDebuggerHk vdb;
	DemoEventReceiver receiver;

	std::vector<hkpRigidBody*> m_bodies;
	std::vector<scene::IMeshSceneNode*> g_bodies;
	std::vector<scene::ICameraSceneNode*> cameras;
	video::SColor bodies_color;

	int id_camera;

	void createWorld();
	void createCollisionFilter();
	void createBase();
	void createGrids();
public:
	CollisionFilter();
	virtual ~CollisionFilter();

	void initHk();
	void runHk();
	void quitHk();

	const int runIrr();
	const int add_cameraIrr();
	const int add_gui_elementsIrr();
	const int add_scene_nodesIrr();
};