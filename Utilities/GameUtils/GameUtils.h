#pragma once

#include <Common\Base\hkBase.h>
#include <Physics2012\Dynamics\World\hkpWorld.h>
#include <Physics2012\Dynamics\Entity\hkpRigidBody.h>
#include <Physics2012\Collide\Shape\Convex\Box\hkpBoxShape.h>
#include <Physics2012\Utilities\Dynamics\Inertia\hkpInertiaTensorComputer.h>

class GameUtils {
public:
	//Create a fixed rigid body made with a hkpBoxShape
	static void createStaticBox(hkpWorld* world, float centerX, float centerY, float centerZ, float radiusX, float radiusY, float radiusZ);

	//This is just a user function to help create a box in one line
	static hkpRigidBody* createBox(const hkVector4& size, const hkReal mass, const hkVector4& position, hkReal radius = 0.0);
};