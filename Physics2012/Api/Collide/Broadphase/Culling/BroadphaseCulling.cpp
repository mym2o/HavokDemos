#include "BroadphaseCulling.h"

BroadphaseCulling::BroadphaseCulling() : m_rand(180673) {
	m_numRigidBodies = 1024;
	m_numUserObjects = 0;
}

BroadphaseCulling::~BroadphaseCulling() {
	
}

void BroadphaseCulling::initHk() {
	memoryRouter = hkMemoryInitUtil::initDefault(hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(1024 * 1024));
	hkBaseSystem::init(memoryRouter, errorReport);

	hkAabb worldAabb;
	worldAabb.setEmpty();
	hkpWorldCinfo cinfo;
	cinfo.m_gravity.setAll(0);
	cinfo.m_broadPhaseWorldAabb.m_max.set(100, 100, 8);
	cinfo.m_broadPhaseWorldAabb.m_min.setNeg4(cinfo.m_broadPhaseWorldAabb.m_max);

	//we need an hkpTreeBroadPhase to perform culling
	cinfo.m_broadPhaseType = hkpWorldCinfo::BROADPHASE_TYPE_TREE;

	worldAabb = cinfo.m_broadPhaseWorldAabb;
	m_world = new hkpWorld(cinfo);
	m_world->lock();

	hkpAgentRegisterUtil::registerAllAgents(m_world->getCollisionDispatcher());

	//create a random batch of boxes in the world
	if (m_numRigidBodies > 0) {
		hkpMotion::MotionType motionType = hkpMotion::MOTION_SPHERE_INERTIA;
		hkPseudoRandomGenerator rand(100);
		
		hkArray<const hkpCollidable*> collidables;
		
		BroadphaseCulling::createRandomBodies(m_world, worldAabb, m_numRigidBodies, motionType, &rand, collidables);
	}

	//create random user object as phantoms
	if (m_numUserObjects > 0) {
		//TODO: next variant
	}

	m_rand.getRandomRotation(m_orientations[0]);
	m_rand.getRandomRotation(m_orientations[1]);
	m_animTime = 0;
	m_cullFarPlane = false;
	m_useVelocityForUserObjects = true;
	m_sort = true;

	//optimize broadphase
	((hkpTreeBroadPhase*)m_world->getBroadPhase())->fullOptimize();

	m_world->unlock();
}

void BroadphaseCulling::runHk() {
	vdb.setVisualDebugger(m_world);

	hkStopwatch stopWatch;
	stopWatch.start();
	hkReal lastTime = stopWatch.getElapsedSeconds();

	hkReal timeStep = 1.f / 60.f;
	int numSteps = int(10.f / timeStep);
	for (int i = 0; i < numSteps; i++) {
		m_world->stepDeltaTime(timeStep);

		vdb.step();

		step();

		while (stopWatch.getElapsedSeconds() < lastTime + timeStep);
		lastTime += timeStep;
	}
}

void BroadphaseCulling::quitHk() {
	hkReferencedObject::removeReferences(m_userObjects.begin(), m_userObjects.getSize());
	m_world->removeReference();
	vdb.quitVdb();
	hkBaseSystem::quit();
	hkMemoryInitUtil::quit();
}

void BroadphaseCulling::createRandomBodies(hkpWorld* world, const hkAabb& worldAabb, int num_bodies, hkpMotion::MotionType motionType, class hkPseudoRandomGenerator* rand, hkArray<const hkpCollidable*>& collidablesOut) {
	hkpRigidBodyCinfo rigidBodyInfo;
	rigidBodyInfo.m_collisionFilterInfo = hkpGroupFilter::calcFilterInfo(1, 1);

	hkArray<hkpEntity*> bodyArray;
	bodyArray.reserve(num_bodies);

	hkpShape* shape;
	hkVector4 halfExtents(0.5f, 0.5f, 0.5f);
	shape = new hkpBoxShape(halfExtents, 0.0f);

	hkAabb shrunkenAabb = worldAabb;
	shrunkenAabb.m_min.mul4(.9f);
	shrunkenAabb.m_max.mul4(.9f);
	for (int i = 0; i < num_bodies; i++) {
		//all bodies created are movable
		rigidBodyInfo.m_motionType = motionType;
		
		//a collection of many rigid bodies is randomly created using hkpBoxShape
		rigidBodyInfo.m_shape = shape;

		hkReal mass = 10.0f;
		hkReal d = mass * 0.5f;
		hkMatrix3Util::_setDiagonal(d, d, d, rigidBodyInfo.m_inertiaTensor);
		rigidBodyInfo.m_mass = mass;

		//the object is then assigned a random position, orientation and angular velocity and added to the world
		rand->getRandomVectorRange(shrunkenAabb.m_min, shrunkenAabb.m_max, rigidBodyInfo.m_position);
		rand->getRandomRotation(rigidBodyInfo.m_rotation);

		rigidBodyInfo.m_collisionFilterInfo = hkpGroupFilterSetup::LAYER_DEBRIS;

		hkpRigidBody* rigidBody = new hkpRigidBody(rigidBodyInfo);

		//give them an initial velocity
		if (!rigidBody->isFixed()) {
			hkVector4 angularVel(rand->getRandRange(-1.f, 1.f), rand->getRandRange(-1.f, 1.f), rand->getRandRange(-1.f, 1.f), rand->getRandRange(-1.f, 1.f));
			rigidBody->setAngularVelocity(angularVel);
			rigidBody->setAngularDamping(0.f);
		}

		bodyArray.pushBack(rigidBody);
		collidablesOut.pushBack(rigidBody->getCollidable());

		//there is no gravity vector for this world and so the bodies will appear to float in space
	}
	shape->removeReference();

	//batch add all bodies to the system and defragment the broadphase
	world->addEntityBatch(bodyArray.begin(), bodyArray.getSize());
	world->getBroadPhase()->defragment();

	//remove all references to bodies
	for (int i = 0; i < bodyArray.getSize(); i++) {
		bodyArray[i]->removeReference();
	}
}

void BroadphaseCulling::step() {
	//lock the world and get the tree broad-phase
	m_world->lock();
	hkpTreeBroadPhase* broadphase = static_cast<hkpTreeBroadPhase*>(m_world->getBroadPhase());

	//toggle far plane culling
	//TODO: manage keyboard input to change m_cullFarPlane flag

	//toggle front to back sorting
	//TODO: manage keyboard input to change m_sort flag

	//toggle velocity for user objects
	//TODO: manage keyboard input to change m_useVelocityForUserObjects flag

	//optimize broadphase
	//TODO: manage keyboard input to optimize
	broadphase->fullOptimize(); //default

	//remove one user object at random
	//TODO: manage keyboard input to remove

	//TODO: other user objects properties

	//animate frustum
	const hkReal spanDuration = 3.f;
	m_animTime += 1.f / 60.f / spanDuration;
	while (m_animTime > 1.f) {
		m_animTime -= 1.f;
		m_orientations[0] = m_orientations[1];
		m_rand.getRandomRotation(m_orientations[1]);
	}
	hkReal splinePoly = (3 - 2 * m_animTime)*m_animTime*m_animTime;
	hkQuaternion currentOrientation;
	currentOrientation.setSlerp(m_orientations[0], m_orientations[1], hkSimdReal(splinePoly));

	//create frustum vertices
	hkVector4 nearPlaneQuad[4];
	nearPlaneQuad[0].set(-10, -10, 0);
	nearPlaneQuad[1].set(10, -10, 0);
	nearPlaneQuad[2].set(10, 10, 0);
	nearPlaneQuad[3].set(-10, 10, 0);

	hkVector4 farPlaneQuad[4];
	hkSimdReal fovFactor = 4.f;
	hkReal farOffset = 75.f;
	for (int i = 0; i < 4; i++) {
		farPlaneQuad[i].setMul4(fovFactor, nearPlaneQuad[i]);
		farPlaneQuad[i](2) += farOffset;
	}

	//transform frustum
	hkVector4 direction;
	direction.set(0, 0, 1, 0);
	direction.setRotatedDir(currentOrientation, direction);

	for (int i = 0; i < 4; i++) {
		nearPlaneQuad[i].setRotatedDir(currentOrientation, nearPlaneQuad[i]);
		farPlaneQuad[i].setRotatedDir(currentOrientation, farPlaneQuad[i]);
	}

	//draw frustum
	HK_DISPLAY_LINE(nearPlaneQuad[0], nearPlaneQuad[1], hkColor::YELLOW);
	HK_DISPLAY_LINE(nearPlaneQuad[1], nearPlaneQuad[2], hkColor::YELLOW);
	HK_DISPLAY_LINE(nearPlaneQuad[2], nearPlaneQuad[3], hkColor::YELLOW);
	HK_DISPLAY_LINE(nearPlaneQuad[3], nearPlaneQuad[0], hkColor::YELLOW);

	if (m_cullFarPlane) {
		HK_DISPLAY_LINE(farPlaneQuad[0], farPlaneQuad[1], hkColor::YELLOW);
		HK_DISPLAY_LINE(farPlaneQuad[1], farPlaneQuad[2], hkColor::YELLOW);
		HK_DISPLAY_LINE(farPlaneQuad[2], farPlaneQuad[3], hkColor::YELLOW);
		HK_DISPLAY_LINE(farPlaneQuad[3], farPlaneQuad[0], hkColor::YELLOW);
	}

	HK_DISPLAY_LINE(farPlaneQuad[0], nearPlaneQuad[0], hkColor::YELLOW);
	HK_DISPLAY_LINE(farPlaneQuad[1], nearPlaneQuad[1], hkColor::YELLOW);
	HK_DISPLAY_LINE(farPlaneQuad[2], nearPlaneQuad[2], hkColor::YELLOW);
	HK_DISPLAY_LINE(farPlaneQuad[3], nearPlaneQuad[3], hkColor::YELLOW);

	//compute frustum plane
	hkVector4 planes[6];

	//side planes
	planes[0] = computePlaneFromTriangle(nearPlaneQuad[0], nearPlaneQuad[1], farPlaneQuad[0]);
	planes[1] = computePlaneFromTriangle(nearPlaneQuad[1], nearPlaneQuad[2], farPlaneQuad[1]);
	planes[2] = computePlaneFromTriangle(nearPlaneQuad[2], nearPlaneQuad[3], farPlaneQuad[2]);
	planes[3] = computePlaneFromTriangle(nearPlaneQuad[3], nearPlaneQuad[0], farPlaneQuad[3]);

	//near plane
	planes[4] = computePlaneFromTriangle(nearPlaneQuad[0], nearPlaneQuad[2], farPlaneQuad[1]);

	//far plane
	planes[5] = computePlaneFromTriangle(farPlaneQuad[0], farPlaneQuad[1], farPlaneQuad[2]);

	//cull using frustum
	hkArray<const hkpBroadPhaseHandle*> handles;
	HK_TIME_CODE_BLOCK("Culling", this);
	handles.reserve(4096 / sizeof(const hkpBroadPhaseHandle*));
	if (m_sort) {
		broadphase->queryConvexSorted(direction, planes, m_cullFarPlane ? 6 : 5, handles);
	}
	else {
		broadphase->queryConvex(planes, m_cullFarPlane ? 6 : 5, handles);
	}

	//draw objects AABB visible from the frustum
	for (int i = 0; i < handles.getSize(); i++) {
		hkAabb aabb;
		broadphase->getAabb(handles[i], aabb);
		hkColor::Argb color = hkColor::YELLOW;
		if (m_sort) {
			color = spectrumColor((i + 1) / (hkReal)handles.getSize());
		}
		color = hkColor::rgbFromChars(hkColor::getRedAsChar(color), hkColor::getGreenAsChar(color), hkColor::getBlueAsChar(color), 127);
		displaySolidAABB(aabb, color);
	}

	//display current settings
	hkStringBuf str;
	static const char* sortName[] = { "None", "Front to back" };
	str.printf("Far plane culling(F): %s\nSort(B): %s\nVelocities(V): %s", m_cullFarPlane ? "ON" : "OFF", sortName[m_sort ? 1 : 0], m_useVelocityForUserObjects ? "ON" : "OFF");
	//TODO: display the text

	m_world->unlock();
}

hkVector4 BroadphaseCulling::computePlaneFromTriangle(const hkVector4& a, const hkVector4& b, const hkVector4& c) {
	hkVector4 ab, ac, plane;
	ab.setSub4(b, a);
	ac.setSub4(c, a);
	plane.setCross(ab, ac);
	plane.normalize3();
	plane(3) = -plane.dot3(a);
	return plane;
}

hkColor::Argb BroadphaseCulling::spectrumColor(hkReal s) {
	const hkVector4 palette[] = { hkVector4(0, 0, 0, 1), hkVector4(0, 0, 1, 1), hkVector4(0, 1, 1, 1), hkVector4(0, 1, 0, 1), hkVector4(1, 1, 0, 1), hkVector4(1, 0, 0, 1) };
	const int numColors(sizeof(palette) / sizeof(palette[0]));
	hkVector4 color(0, 0, 0, 0);
	if (s <= 0) {
		color = palette[0];
		s = 0;
	}
	else if (s >= 1) {
		color = palette[numColors - 1];
		s = 1;
	}
	else {
		hkReal ms = s*(numColors - 1);
		int c = (int)ms;
		hkReal u = ms - c;
		color.setInterpolate4(palette[c], palette[c + 1], u);
	}
	return hkColor::rgbFromFloats(color(0), color(1), color(2), 1);
}

void BroadphaseCulling::displaySolidAABB(const hkAabb& aabb, hkColor::Argb color) {
	const hkVector4 minMax[2] = { aabb.m_min, aabb.m_max };
	hkVector4 vertices[8];

	for (int i = 0; i < 8; i++) {
		for (int j = 0; j < 3; j++) {
			vertices[i](j) = minMax[(i >> j) & 1](j);
		}
	}

	const int quads[][4] = { { 0, 2, 1, 3 }, { 4, 5, 6, 7 }, { 0, 1, 4, 5 }, { 1, 3, 5, 7 }, { 3, 2, 7, 6 }, { 2, 0, 6, 4 } };
	for (int i = 0; i < (int)(sizeof(quads) / sizeof(quads[0])); i++) {
		displaySolidQuad(vertices[quads[i][0]], vertices[quads[i][1]], vertices[quads[i][2]], vertices[quads[i][3]], color);
	}
}

void BroadphaseCulling::displaySolidQuad(const hkVector4& a, const hkVector4& b, const hkVector4& c, const hkVector4& d, hkColor::Argb color) {
	HK_DISPLAY_TRIANGLE(a, b, c, color);
	HK_DISPLAY_TRIANGLE(c, b, d, color);
}