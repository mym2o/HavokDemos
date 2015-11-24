#include "Serialize.h"

Serialize::Serialize() {

}

Serialize::~Serialize() {

}

void Serialize::initHk() {
	//Allocate 1mb for the solver
	memoryRouter = hkMemoryInitUtil::initDefault(hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(1024 * 1024));
	hkBaseSystem::init(memoryRouter, errorReport);

	//Create a hkGeometry object to illustrate the serialization functionality
	geometry = new hkGeometry();
}

void Serialize::quitHk() {
	delete geometry;

	hkBaseSystem::quit();
	hkMemoryInitUtil::quit();
}

void Serialize::runHk() {
	setGeometry();
	writeXMLtagfile();
	readBack();
}

//make a box geometry of dimension 2x4x6
void Serialize::setGeometry() {
	hkVector4 halfExtents(1.0f, 2.0f, 3.0f);

	hkVector4* v = geometry->m_vertices.expandBy(8);
	v[0].set(-halfExtents(0), halfExtents(1), halfExtents(2));
	v[1].set(halfExtents(0), halfExtents(1), halfExtents(2));
	v[2].set(halfExtents(0), -halfExtents(1), halfExtents(2));
	v[3].set(-halfExtents(0), -halfExtents(1), halfExtents(2));
	v[4].set(-halfExtents(0), halfExtents(1), -halfExtents(2));
	v[5].set(halfExtents(0), halfExtents(1), -halfExtents(2));
	v[6].set(halfExtents(0), -halfExtents(1), -halfExtents(2));
	v[7].set(-halfExtents(0), -halfExtents(1), -halfExtents(2));

	geometry->m_triangles.expandBy(1)->set(3, 2, 1);
	geometry->m_triangles.expandBy(1)->set(3, 1, 0);
	geometry->m_triangles.expandBy(1)->set(6, 7, 4);
	geometry->m_triangles.expandBy(1)->set(6, 4, 5);
	geometry->m_triangles.expandBy(1)->set(4, 7, 3);
	geometry->m_triangles.expandBy(1)->set(4, 3, 0);
	geometry->m_triangles.expandBy(1)->set(2, 6, 5);
	geometry->m_triangles.expandBy(1)->set(2, 5, 1);
	geometry->m_triangles.expandBy(1)->set(7, 6, 2);
	geometry->m_triangles.expandBy(1)->set(7, 2, 3);
	geometry->m_triangles.expandBy(1)->set(1, 5, 4);
	geometry->m_triangles.expandBy(1)->set(1, 4, 0);
}

//Write out a binary file
void Serialize::writeXMLtagfile() {
	hkOstream stream("geometry_xml_tagfile.xml");
	hkResult res = hkSerializeUtil::saveTagfile(geometry, hkGeometryClass, stream.getStreamWriter(), HK_NULL, hkSerializeUtil::SAVE_TEXT_FORMAT);
	if (res != HK_SUCCESS) {
		HK_ERROR(0x000FF, "Failed to save binary");
	}
	else {
		HK_WARN_ALWAYS(0x111AA, "Saved successfully");
	}
}

//Read back a serialized file
void Serialize::readBack() {
	hkIstream stream("geometry_xml_tagfile.xml");
	
	hkResource* resource = hkSerializeUtil::load(stream.getStreamReader());
	hkBool32 failed = true;
	if (resource) {
		//Get the contained Geometry
		hkGeometry* readGeometry = resource->getContents<hkGeometry>();

		//Check to see if the last vertex is the same, as a simple check to
		//see if the serialization has worked correctly.
		failed = !readGeometry->m_vertices[7].equals3(readGeometry->m_vertices[7]);

		resource->removeReference();
	}
	if (failed) {
		HK_ERROR(0x000FF, "Failed loading binary");
	}
	else {
		HK_WARN_ALWAYS(0x111AA, "Loaded successfully");
	}
}