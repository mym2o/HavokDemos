#include "ContactState.h"

#include <Physics2012\Collide\Dispatch\hkpAgentRegisterUtil.h>
#include <Physics2012\Collide\Shape\Compound\Collection\StorageExtendedMesh\hkpStorageExtendedMeshShape.h>
#include <Physics2012\Collide\Shape\Compound\Tree\Mopp\hkpMoppUtility.h>
#include <Physics2012\Collide\Shape\Compound\Tree\Mopp\hkpMoppBvTreeShape.h>
#include <Physics2012\Collide\Util\Welding\hkpMeshWeldingUtility.h>
#include <Physics2012\Collide\Shape\Convex\Capsule\hkpCapsuleShape.h>
#include <Physics2012\Utilities\Dynamics\Inertia\hkpInertiaTensorComputer.h>

static hkColor::Argb colorTable[] = {
	hkColor::RED,
	hkColor::GREEN,
	hkColor::YELLOW,
	hkColor::BLUE
};

static const int numColors = sizeof(colorTable) / sizeof(int);

static const char* colorNames[] = {
	"red",
	"green",
	"yellow",
	"blue"
};

static const hkVector4 verts[] = {
	hkVector4(-4.f, 0.2f, -4.f),
	hkVector4(4.f, 0.2f, -4.f),
	hkVector4(4.f, -0.2f, 4.f),
	hkVector4(-4.f, -0.2f, 4.f),
	hkVector4(0.f, 0.0f, 0.f),
	hkVector4(0.f, 0.2f, 6.f)
};

static const int numVerts = sizeof(verts) / sizeof(hkVector4);

static const hkUint8 indices[] = {
	1,0,4,
	2,1,4,
	3,2,4,
	0,3,4,
	2,3,5
};

static const int numIndices = sizeof(indices) / sizeof(hkUint8);

ContactState::ContactState() : IrrInterface() {
	device->setEventReceiver(&receiver);
}

void ContactState::initHk() {
	memoryRouter = hkMemoryInitUtil::initDefault(hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(1024 * 1024));
	hkBaseSystem::init(memoryRouter, errorReport);

	createWorld();
	m_world->lock();

	hkpAgentRegisterUtil::registerAllAgents(m_world->getCollisionDispatcher());

	createMesh();
	createCapsule();

	m_listener = new MyContactStateListener(m_body, &m_shapeKeys);
	m_body->addContactListener(m_listener);

	m_world->unlock();
}

void ContactState::createWorld() {
	hkpWorldCinfo info;
	info.setupSolverInfo(hkpWorldCinfo::SOLVER_TYPE_4ITERS_MEDIUM);
	info.setBroadPhaseWorldSize(100.f);
	info.m_enableDeactivation = false;
	m_world = new hkpWorld(info);
}

/**
 * In this demo we have three different objects; the wall, the sphere and a phantom volume. Both the wall,
 * which is simply a box, and the sphere are created in the usual manner using a hkpRigidBodyCInfo 'blueprint'
 * and are added to the world.
 */
void ContactState::createMesh() {
	hkpStorageExtendedMeshShape* mesh = new hkpStorageExtendedMeshShape;
	hkpExtendedMeshShape::TrianglesSubpart subPart;
	subPart.m_vertexBase = &verts[0](0);
	subPart.m_vertexStriding = sizeof(hkVector4);
	subPart.m_numVertices = numVerts;
	subPart.m_indexBase = indices;
	subPart.m_stridingType = hkpExtendedMeshShape::INDICES_INT8;
	subPart.m_indexStriding = sizeof(hkUint8) * 3;
	subPart.m_numTriangleShapes = numIndices / 3;
	HK_ASSERT(0xe2ee3488, subPart.m_numTriangleShapes);

	mesh->addTrianglesSubpart(subPart);

	//set up the shapeKey array
	hkpShapeKey key = mesh->getFirstKey();
	while(key != HK_INVALID_SHAPE_KEY) {
		m_shapeKeys.pushBack(key);
		key = mesh->getNextKey(m_shapeKeys.back());
	}
	HK_ASSERT(0xe2ee3488, m_shapeKeys.getSize() == subPart.m_numTriangleShapes);

	hkpMoppCompilerInput mci;
	hkpMoppCode* code = hkpMoppUtility::buildCode(mesh, mci);
	hkpMoppBvTreeShape* moppShape = new hkpMoppBvTreeShape(mesh, code);
	hkpMeshWeldingUtility::computeWeldingInfo(mesh, moppShape, hkpWeldingUtility::WELDING_TYPE_ANTICLOCKWISE);

	mesh->removeReference();
	code->removeReference();

	hkpRigidBodyCinfo ci;
	ci.m_shape = moppShape;
	ci.m_position = hkVector4::getZero();
	ci.m_restitution = 0.f;
	ci.m_friction = 1.f;
	ci.m_motionType = hkpMotion::MOTION_FIXED;
	//ensure shape keys are stored
	ci.m_numShapeKeysInContactPointProperties = -1;

	hkpRigidBody* rb = new hkpRigidBody(ci);

	moppShape->removeReference();

	m_world->addEntity(rb);

	//make the mesh slightly transparent so we can see the debug lines
	HK_SET_OBJECT_COLOR(hkUlong(rb->getCollidable()), hkColor::rgbFromChars(0xff, 0xff, 0xff, 0x60));

	rb->removeReference();
}

void ContactState::createCapsule() {
	hkReal radius = 0.5f;
	hkVector4 vecA(-1.f, 0.f, 0.f);
	hkVector4 vecB(1.f, 0.f, 0.f);
	hkpCapsuleShape* shape = new hkpCapsuleShape(vecA, vecB, radius);

	hkpRigidBodyCinfo info;
	info.m_shape = shape;
	info.m_position.set(-0.5f, 2.f, -3.f);
	info.m_restitution = 0.9f;
	info.m_mass = 1.f;
	hkMassProperties massProperties;
	hkpInertiaTensorComputer::computeCapsuleVolumeMassProperties(vecA, vecB, radius, info.m_mass, massProperties);
	info.m_inertiaTensor = massProperties.m_inertiaTensor;
	info.m_centerOfMass = massProperties.m_centerOfMass;
	info.m_mass = massProperties.m_mass;
	//we want to update our contact manifold every step
	info.m_contactPointCallbackDelay = 0;

	m_body = new hkpRigidBody(info);
	shape->removeReference();
	m_world->addEntity(m_body);
}

void ContactState::quitHk() {
	m_world->markForWrite();
	m_body->removeContactListener(m_listener);
	m_listener->removeReference();
	m_world->removeEntity(m_body);
	m_body->removeReference();
	m_world->unmarkForWrite();

	vdb.quitVdb();

	hkBaseSystem::quit();
	hkMemoryInitUtil::quit();
	
	quit_Irr();
}

static void drawTriangles() {
	const int numTriangles = numIndices / 3;
	for(int i = 0; i < numTriangles; i++) {
		hkUint8 index = hkUint8(i * 3);
		const hkVector4& vert0 = verts[indices[index]];
		const hkVector4& vert1 = verts[indices[index + 1]];
		const hkVector4& vert2 = verts[indices[index + 2]];

		hkColor::Argb& color = colorTable[i % numColors];
		HK_DISPLAY_LINE(vert0, vert1, color);
		HK_DISPLAY_LINE(vert1, vert2, color);
		HK_DISPLAY_LINE(vert2, vert0, color);

		hkVector4 center = vert0;
		center.add4(vert1);
		center.add4(vert2);
		center.mul4(0.333f);

		HK_DISPLAY_3D_TEXT(colorNames[i % numColors], center, color);
	}
}

static void drawIrrTriangles(IrrlichtDevice* device) {
	const int numTriangles = numIndices / 3;
	for(int i = 0; i < numTriangles; i++) {
		hkUint8 index = hkUint8(i * 3);
		const hkVector4& vert0 = verts[indices[index]];
		const hkVector4& vert1 = verts[indices[index + 1]];
		const hkVector4& vert2 = verts[indices[index + 2]];

		hkColor::Argb& color = colorTable[i % numColors];
		video::SColor irr_color = video::SColor(color);

		video::IVideoDriver* driver = device->getVideoDriver();
		core::vector3df irr_vert0(vert0(0), vert0(1), vert0(2));
		core::vector3df irr_vert1(vert1(0), vert1(1), vert1(2));
		core::vector3df irr_vert2(vert2(0), vert2(1), vert2(2));
		driver->setTransform(video::ETS_WORLD, core::matrix4());
		driver->draw3DLine(irr_vert0, irr_vert1, irr_color);
		driver->draw3DLine(irr_vert1, irr_vert2, irr_color);
		driver->draw3DLine(irr_vert2, irr_vert0, irr_color);
	}
}

void ContactState::step(const hkReal& timeStep) {
	m_world->markForWrite();

	average_color = m_listener->update();
	//add debug lines so we can see the triangles' colors
	drawTriangles();

	m_world->unmarkForWrite();

	m_world->stepDeltaTime(timeStep);
}

void ContactState::runHk() {
	vdb.setVisualDebugger(m_world);

	add_cameraIrr();
	add_gui_elementsIrr();
	add_scene_nodesIrr();

	hkStopwatch stopwatch;
	stopwatch.start();
	hkReal lastTime = stopwatch.getElapsedSeconds();
	hkReal timeStep = 1.f/60.f;
	int numSteps = int(20.f/timeStep);
	while(!receiver.IsKeyDown(EKEY_CODE::KEY_ESCAPE)) {
		runIrr();
		
		vdb.step();

		step(timeStep);

		while(stopwatch.getElapsedSeconds() <= lastTime + timeStep);
		lastTime += timeStep;
	}
}

const int ContactState::add_cameraIrr() {
	scene_manager->addCameraSceneNode(0, core::vector3df(-8.f, 8.f, 8.f), core::vector3df());
	//scene_manager->addCameraSceneNodeFPS();
	return 0;
}

const int ContactState::add_gui_elementsIrr() {
	device->setWindowCaption(L"ContactState Demo - Havok+Irrlicht Engine");
	return 0;
}

const int ContactState::add_scene_nodesIrr() {
	scene_manager->addLightSceneNode(0, core::vector3df(0.f, 100.f, 100.f), video::SColorf(1.f, 1.f, 1.f), 800.f, 0);

	scene::IMesh* mesh = scene_manager->getGeometryCreator()->createCylinderMesh(0.5f, 2.f, 16);
	scene_manager->addMeshSceneNode(mesh, 0, 1, core::vector3df(), core::vector3df(0, 0, 90.f));

	return 0;
}

const int ContactState::runIrr() {
	device->run();
	
	driver->beginScene();

	scene_manager->drawAll();
	gui_env->drawAll();

	drawIrrTriangles(device);

	hkVector4 hk_pos = m_body->getPosition();
	scene::IMeshSceneNode* g_capsule = (scene::IMeshSceneNode*)scene_manager->getSceneNodeFromId(1);
	g_capsule->setPosition(core::vector3df(hk_pos(0) + 1.f, hk_pos(1), hk_pos(2)));
	setColorAndShadow(g_capsule, video::SColor(average_color));

	driver->endScene();
	return 0;
}