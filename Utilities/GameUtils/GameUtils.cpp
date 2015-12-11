#include "GameUtils.h"

void GameUtils::createStaticBox(hkpWorld* world, float centerX, float centerY, float centerZ, float radiusX, float radiusY, float radiusZ) {
	hkVector4 radii(radiusX, radiusY, radiusZ);

	hkpShape* boxShape = new hkpBoxShape(radii, 0);

	hkpRigidBodyCinfo boxInfo;
	boxInfo.m_motionType = hkpMotion::MOTION_FIXED;
	boxInfo.m_shape = boxShape;
	boxInfo.m_position.set(centerX, centerY, centerZ);

	hkpRigidBody* boxRigidBody = new hkpRigidBody(boxInfo);
	world->addEntity(boxRigidBody);
	boxRigidBody->removeReference();
	boxShape->removeReference();
}

hkpRigidBody* GameUtils::createBox(const hkVector4& size, const hkReal mass, const hkVector4& position, hkReal radius) {
	hkVector4 halfExtents(size(0) * 0.5f, size(1) * 0.5f, size(2) * 0.5f);
	hkpBoxShape* cube = new hkpBoxShape(halfExtents, radius);

	//create a rigid body construction template
	hkpRigidBodyCinfo boxInfo;
	if (mass != 0.f) {
		boxInfo.m_mass = mass;
		hkMassProperties massProperties;
		hkpInertiaTensorComputer::computeBoxVolumeMassProperties(halfExtents, mass, massProperties);
		boxInfo.m_inertiaTensor = massProperties.m_inertiaTensor;
		boxInfo.m_motionType = hkpMotion::MOTION_BOX_INERTIA;
	}
	else {
		boxInfo.m_motionType = hkpMotion::MOTION_FIXED;
	}
	boxInfo.m_rotation.setIdentity();
	boxInfo.m_shape = cube;
	boxInfo.m_position = position;

	//create a rigid body (using the template above)
	hkpRigidBody* boxRigidBody = new hkpRigidBody(boxInfo);
	cube->removeReference();
	
	return boxRigidBody;
}