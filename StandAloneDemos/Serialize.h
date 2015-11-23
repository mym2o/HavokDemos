/**
 *	This simple console demo demonstrates how to load and save a Havok object using 
 *	the serialization system.
 */

#pragma once

#include "../HavokDefinitions.h"

#include <Common\Base\System\Io\IStream\hkIStream.h>

//Geometry
#include <Common\Base\Types\Geometry\hkGeometry.h>

//Serialize
#include <Common\Serialize\Util\hkSerializeUtil.h>

// We are using serialization, so we need ReflectedClasses.
// The objects are being saved and then loaded immediately so we know the version of the saved data is the same 
// as the version the application is linked with. Because of this we don't need RegisterVersionPatches or SerializeDeprecatedPre700.
// If the demo was reading content saved from a previous version of the Havok content tools (common in real world Applications) 
// RegisterVersionPatches and perhaps SerializeDeprecatedPre700 are needed.
#define HK_EXCLUDE_FEATURE_SerializeDeprecatedPre700
// We can also restrict the compatibility to files created with the current version only using HK_SERIALIZE_MIN_COMPATIBLE_VERSION.
// If we wanted to have compatibility with at most version 650b1 we could have used something like:
// #define HK_SERIALIZE_MIN_COMPATIBLE_VERSION 650b1.
//#define HK_SERIALIZE_MIN_COMPATIBLE_VERSION Current

#define HK_EXCLUDE_FEATURE_RegisterVersionPatches
//#define HK_EXCLUDE_FEATURE_RegisterReflectedClasses
#define HK_EXCLUDE_FEATURE_MemoryTracker

#define HK_EXCLUDE_LIBRARY_hkcdCollide
#define HK_EXCLUDE_LIBRARY_hkcdInternal
#define HK_EXCLUDE_LIBRARY_hkSceneData
#define HK_EXCLUDE_LIBRARY_hkGeometryUtilities

class Serialize : public HavokInterface {
private:
	hkGeometry* geometry;

	void setGeometry();
	void writeXMLtagfile();
	void readBack();
public:
	Serialize();
	~Serialize();

	virtual void initHk();
	virtual void quitHk();
	virtual void run();
};