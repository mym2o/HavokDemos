#include "IrrInterface.h"

IrrInterface::IrrInterface(const video::E_DRIVER_TYPE deviceType, const u32& h, const u32& w, const u32& bits, const bool& fullscreen, const bool& stencilBuffer) {
	device = createDevice(deviceType, core::dimension2d<u32>(h, w), bits, fullscreen, stencilBuffer, fullscreen, 0);
	driver = device->getVideoDriver();
	gui_env = device->getGUIEnvironment();
	scene_manager = device->getSceneManager();
}

IrrInterface::~IrrInterface() {
	device->drop();
}