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
//	This is a really basic Action which applies impulses to a body to roll it around a world.					//
//	When the corresponding keys are pressed, the internal state is set to "push" or "rotate" the given body.	//
//	If this body is a sphere, it will roll (due to friction) when in contact with the ground. When "rotated",	//
//	ie. turned left/right, the body itself not actually rotated, only the "forward" direction.					//
//	The setXPressed() accessors allow you to change the internal state. The body maintains its state			//
//	internally, so it is not necessary (for example) to continuously tell the action to go forward once			//
//	setForwardPressed(true) is called. It will go forward until setForwardPressed(false) is called.				//
//	You can also make the body "jump" by applying an upward impulse, and "brake" by killing all velocity.		//
//																												//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "../../HavokDefinitions.h"

#include <Physics2012\Dynamics\Action\hkpUnaryAction.h>

class MarbleAction : public hkpUnaryAction {
public:
	//The forward direction here is the original direction (in world space) which the body is said to be "facing"
	MarbleAction(hkpRigidBody* r, const hkVector4& forward, const hkVector4& resetPosition, hkReal impulseScale = 0.1f);

	//Set internal states
	inline void setForwardPressed(hkBool f) { m_forwardPressed = f; }
	inline void setBackwardPressed(hkBool b) { m_backwardPressed = b; }
	inline void setLeftPressed(hkBool l) { m_leftPressed = l; }
	inline void setRightPressed(hkBool r) { m_rightPressed = r; }
	inline void setJumpPressed(hkBool j) { m_jumpPressed = j; }
	inline void setBrakePressed(hkBool b) { m_brakePressed = b; }

	//Move body back to reset position (set on construction)
	void reset();

	//this is the call that performs the action, that gets called from the physics engine
	void applyAction(const hkStepInfo& stepInfo);

	//no need to clone as we don't use clone func in this demo
	virtual hkpAction* clone(const hkArray<hkpEntity*>& newEntities, const hkArray<hkpPhantom*>& newPhantoms) const {
		return HK_NULL;
	}

	inline const hkVector4 getStartVector() const { return start; }
	inline const hkVector4 getEndVector() const { return end; }

private:
	hkBool m_forwardPressed;
	hkBool m_backwardPressed;
	hkBool m_leftPressed;
	hkBool m_rightPressed;
	hkBool m_jumpPressed;
	hkBool m_brakePressed;

	hkReal m_lastJump;
	hkReal m_impulseScale;
	hkReal m_lasttimeCalled;

	const float m_rotationIncrement;
	float m_currentAngle;

	const hkVector4 m_forward;
	const hkVector4 m_resetPosition;

	hkVector4 start, end;
};