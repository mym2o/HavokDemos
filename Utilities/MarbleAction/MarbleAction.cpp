#include "MarbleAction.h"

MarbleAction::MarbleAction(hkpRigidBody* r, const hkVector4& forward, const hkVector4& resetPosition, hkReal impulseScale) : hkpUnaryAction(r), m_forward(forward), m_resetPosition(resetPosition), m_rotationIncrement(0.01f) {
	m_forwardPressed = false;
	m_backwardPressed = false;
	m_leftPressed = false;
	m_rightPressed = false;
	m_jumpPressed = false;
	m_brakePressed = false;

	m_lastJump = 0.f;
	m_impulseScale = impulseScale;
	m_lasttimeCalled = 0.f;
	m_currentAngle = 0.f;
}

void MarbleAction::applyAction(const hkStepInfo& stepInfo) {
	hkpRigidBody* rb = getRigidBody();

	//we need the delta time to record how long it's been since we last jumped
	//(to avoid jumping while in the air!)
	hkReal dt = stepInfo.m_deltaTime;

	//get a "scale" to change the force of the impulse by, depending on both the mass of the body,
	//an arbitrary "gain", eg 0.1
	hkReal scale = rb->getMass() * m_impulseScale;

	hkVector4 axis(0, 1, 0);
	hkQuaternion q(axis, m_currentAngle);
	hkVector4 f;
	f.setRotatedDir(q, m_forward);

	if (m_forwardPressed) {
		hkVector4 imp;
		imp.setMul4(scale, f);
		rb->applyLinearImpulse(imp);
	}
	if (m_backwardPressed) {
		hkVector4 imp;
		imp.setMul4(-scale, f);
		rb->applyLinearImpulse(imp);
	}

	if (m_rightPressed) {
		m_currentAngle += 3.141592653f * 2 * m_rotationIncrement;
	}

	if (m_leftPressed) {
		m_currentAngle -= 3.141592653f * 2 * m_rotationIncrement;
	}

	m_lasttimeCalled += dt;

	//Jump (only if haven't jumped for at least 1 second)
	if (m_jumpPressed && ((m_lasttimeCalled - m_lastJump) > 1.f)) {
		m_lastJump = m_lasttimeCalled;
		hkVector4 imp(0, rb->getMass() * 6, 0);
		rb->applyLinearImpulse(imp);
		setJumpPressed(false);
	}
	setJumpPressed(false);

	//if brake pressed, zero all velocities
	if (m_brakePressed) {
		hkVector4 zero;
		zero.setZero4();
		rb->setLinearVelocity(zero);
		rb->setAngularVelocity(zero);
		setBrakePressed(false);
	}

	//draw current "facing" direction, usings "Debug" line. This gets pushed onto a global list,
	//and gets dealt with by (perhaps) a drawDebugPointsAndLines() method from the mainline
	start = rb->getPosition();
	//end = start + 1.5 * "forward"
	end = start;
	f.mul4(1.5f);
	end.add4(f);
	//TODO: draw line from start to end
}

void MarbleAction::reset() {
	hkpRigidBody* rb = getRigidBody();

	//put marble back to the reset position defined on construction, and zero velocities
	hkVector4 zero;
	zero.setZero4();
	rb->setPosition(m_resetPosition);
	rb->setLinearVelocity(zero);
	rb->setAngularVelocity(zero);
}