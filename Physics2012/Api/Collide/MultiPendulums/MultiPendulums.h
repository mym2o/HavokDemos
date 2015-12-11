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
//	A demo, which creates an array of pendulums, and a body to "drive" through them.							//
//	It demonstrates the quality of the broadphase (worst case scenario for 3-axis sweep and prune).				//
//																												//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "../../../../HavokDefinitions.h"
#include "../../../../Utilities/MarbleAction/MarbleAction.h"
#include "../../../../Utilities/GameUtils/GameUtils.h"
#include "../../../../Utilities/EventReceiver/DemoEventReceiver.h"

#include <Common\Base\Algorithm\PseudoRandom\hkPseudoRandomGenerator.h>

#include <Physics\Constraint\Data\BallAndSocket\hkpBallAndSocketConstraintData.h>

#include <Physics2012\Collide\Dispatch\hkpAgentRegisterUtil.h>
#include <Physics2012\Utilities\Dynamics\Inertia\hkpInertiaTensorComputer.h>

class MultiPendulums : public HavokInterface, IrrInterface {
private:
	hkpWorld* m_world;
	VisualDebuggerHk vdb;
	DemoEventReceiver receiver;

	MarbleAction* m_marbleAction;
	void createWorld();
	void createMultiPendulumField();
public:
	MultiPendulums();
	virtual ~MultiPendulums();
	void step();

	void initHk();
	void quitHk();
	void runHk();

	const int add_cameraIrr();
	const int add_gui_elementsIrr() { return 0; }
	const int add_scene_nodesIrr() { return 0; }
	const int runIrr();
};