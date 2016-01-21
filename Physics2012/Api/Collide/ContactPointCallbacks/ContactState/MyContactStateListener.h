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

#pragma once

#include "../../../../../HavokDefinitions.h"
#include <Physics2012\Dynamics\Collide\ContactListener\hkpContactListener.h>

class MyContactStateListener : public hkReferencedObject, public hkpContactListener {
private:
public:
	MyContactStateListener(hkpRigidBody* body, const hkArray<hkpShapeKey>* shapeKeys);
	
	//hkpContactListener interface
	void contactPointCallback(const hkpContactPointEvent& event);

	//update the body (to be called once every frame)
	const hkColor::Argb update();

	static hkColor::Argb getAverageColor(hkUint8 state);

	//the rigid body to which this listener is attached
	hkpRigidBody* m_body;
	//a bitfield representing contact with the various triangles in the mesh
	hkUint8 m_state;
	//the state from the last frame
	hkUint8 m_oldState;
	//a pointer to the "global" array of shapeKeys
	const hkArray<hkpShapeKey>* m_shapeKeys;
};