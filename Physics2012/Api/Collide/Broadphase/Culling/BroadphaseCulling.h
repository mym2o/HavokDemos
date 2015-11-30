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
//	This demonstates how to use hkpTreeBroadphase for frustum culling.											//
//																												//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "../../../../../HavokDefinitions.h"
#include "../../../../../IrrInterface.h"

#include <Common/Base/Algorithm/PseudoRandom/hkPseudoRandomGenerator.h>

#include <Physics2012/Dynamics/Phantom/hkpSimpleShapePhantom.h>
#include <Physics2012/Dynamics/Entity/hkpRigidBody.h>

#include <Physics2012\Collide\Shape\Convex\Box\hkpBoxShape.h>
#include <Physics2012/Collide/Dispatch/hkpAgentRegisterUtil.h>
#include <Physics2012\Collide\Filter\Group\hkpGroupFilter.h>
#include <Physics2012\Collide\Filter\Group\hkpGroupFilterSetup.h>

#include <Physics2012\Internal\BroadPhase\TreeBroadPhase\hkpTreeBroadPhase.h>

#include <Common\Base\Math\Matrix\hkMatrix3Util.h>

#include <vector>

class BroadphaseCulling : public HavokInterface, IrrInterface {
private:
	hkPseudoRandomGenerator m_rand;
	hkArray<hkpSimpleShapePhantom*> m_userObjects;
	hkQuaternion m_orientations[2];
	hkInt32 m_numRigidBodies, m_numUserObjects;
	hkReal m_animTime;
	hkBool m_cullFarPlane, m_useVelocityForUserObjects, m_sort;

	hkpWorld* m_world;

	VisualDebuggerHk vdb;

	std::vector<hkpRigidBody*> m_bodies;
	std::vector<scene::IMeshSceneNode*> g_bodies;
	core::vector3df irr_nearPlaneQuad[4], irr_farPlaneQuad[4];
	std::vector<std::vector<core::triangle3df> > g_irrvecs;

	void step();
public:
	BroadphaseCulling();
	virtual ~BroadphaseCulling();

	void initHk();
	void quitHk();
	void runHk();

	const int add_cameraIrr();
	const int add_gui_elementsIrr();
	const int add_scene_nodesIrr();
	const int runIrr();

	static std::vector<hkpRigidBody*> createRandomBodies(hkpWorld* world, const hkAabb& worldAabb, int num_bodies, hkpMotion::MotionType motionType, class hkPseudoRandomGenerator* rand, hkArray<const hkpCollidable*>& collidablesOut);
	//Compute a plane equation from triangle vertices, store offset to the origin in W
	static hkVector4 computePlaneFromTriangle(const hkVector4& a, const hkVector4& b, const hkVector4& c);
	//Generate color from the visible spectrum
	static hkColor::Argb spectrumColor(hkReal s);
	//Display a solid AABB
	static void displaySolidAABB(const hkAabb& aabb, hkColor::Argb color, std::vector<core::triangle3df>& irr_triangles);
	//Display a solid quad
	static void displaySolidQuad(const hkVector4& a, const hkVector4& b, const hkVector4& c, const hkVector4& d, hkColor::Argb color);
};