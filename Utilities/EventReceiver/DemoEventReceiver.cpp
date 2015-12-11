#include "DemoEventReceiver.h"

DemoEventReceiver::DemoEventReceiver() {
	for (u32 i = 0; i < KEY_KEY_CODES_COUNT; i++) {
		KeyIsDown[i] = false;
	}
}

bool DemoEventReceiver::IsKeyDown(EKEY_CODE keyCode) const {
	return KeyIsDown[keyCode];
}

bool DemoEventReceiver::OnEvent(const SEvent& event) {
	if (event.EventType == EET_KEY_INPUT_EVENT) {
		KeyIsDown[event.KeyInput.Key] = event.KeyInput.PressedDown;
	}
	return false;
}

bool DemoEventReceiver::IsAnyDown() {
	for (int i = 0; i < KEY_KEY_CODES_COUNT; i++) {
		if (KeyIsDown[i]) {
			return true;
		}
	}
}