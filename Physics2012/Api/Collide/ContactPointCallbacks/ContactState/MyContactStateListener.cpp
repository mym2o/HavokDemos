#include "MyContactStateListener.h"

static hkColor::Argb colorTable[] = {
	hkColor::RED,
	hkColor::GREEN,
	hkColor::YELLOW,
	hkColor::BLUE
};

static const char* colorNames[] = {
	"red",
	"green",
	"yellow",
	"blue"
};

static const int numColors = sizeof(colorTable) / sizeof(int);

MyContactStateListener::MyContactStateListener(hkpRigidBody* body, const hkArray<hkpShapeKey>* shapeKeys) {
	m_body = body;
	m_state = 0;
	m_oldState = 0;
	m_shapeKeys = shapeKeys;
}

void MyContactStateListener::contactPointCallback(const hkpContactPointEvent& event) {
	HK_ASSERT2(0x01344252, event.m_source != hkpCollisionEvent::SOURCE_WORLD, "This listener should not be attached to the world.");

	//get the leaf shape key of the contact
	HK_ASSERT2(0x0451a7e8a, event.getShapeKeys(1 - event.m_source), "No shape keys were stored for the contact body.");
	hkpShapeKey key = event.getShapeKeys(1 - event.m_source)[0];

	//find which triangle we have encountered
	int triangleNumber = m_shapeKeys->indexOf(key);
	HK_ASSERT2(0x01344252, triangleNumber != -1, "Triangle not found. This very simple demo assumes contact between two specific bodies only.");

	//set the state bit
	HK_ASSERT2(0x01344252, event.m_firingCallbacksForFullManifold, "We should be requesting the full contact manifold.");
	if(event.m_firstCallbackForFullManifold) {
		m_state = 0;
	}
	m_state |= (1 << triangleNumber);
	if(event.m_lastCallbackForFullManifold) {
		m_oldState = m_state;
	}

	//draw the contact point in the triangle's color
	hkVector4 dir;
	dir.setMul4(2.0f, event.m_contactPoint->getNormal());
	HK_DISPLAY_ARROW(event.m_contactPoint->getPosition(), dir, colorTable[triangleNumber % numColors]);
}

hkColor::Argb MyContactStateListener::getAverageColor(hkUint8 state) {
	int i = 0;
	int count = 0;
	int r = 0;
	int g = 0;
	int b = 0;
	while(state) {
		if(state & 1) {
			hkColor::Argb c = colorTable[i % numColors];
			r += hkColor::getRedAsChar(c);
			g += hkColor::getGreenAsChar(c);
			b += hkColor::getBlueAsChar(c);
			++count;
		}
		++i;
		state = state >> 1;
	}
	if(count) {
		//average the components piecewise
		return hkColor::rgbFromChars(static_cast<unsigned char>(r/count), static_cast<unsigned char>(g/count), static_cast<unsigned char>(b/count), 0xff);
	} else {
		return hkColor::LIGHTGREY;
	}
}

const hkColor::Argb MyContactStateListener::update() {
	//color the cylinder
	const hkColor::Argb average_color = getAverageColor(m_state);
	HK_SET_OBJECT_COLOR(hkUlong(m_body->getCollidable()), average_color);

	//report any triangles we've entered
	hkUint8 enteredStates = m_state & ~m_oldState;
	int i = 0;
	while(enteredStates) {
		if(enteredStates & 1) {
			printf("Entered a %s triangle.\n", colorNames[i % numColors]);
		}
		++i;
		enteredStates = enteredStates >> 1;
	}

	//report any triangles we've left
	hkUint8 leftStates = m_oldState & ~m_state;
	i = 0;
	while(leftStates) {
		if(leftStates & 1) {
			printf("Left a %s triangle.\n", colorNames[i % numColors]);
		}
		++i;
		leftStates = leftStates >> 1;
	}
	
	return average_color;
}