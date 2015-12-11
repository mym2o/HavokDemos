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

#include <irrlicht.h>

using namespace irr;

class IrrInterface {
protected:
	IrrlichtDevice* device;

	video::IVideoDriver* driver;
	scene::ISceneManager* scene_manager;
	gui::IGUIEnvironment* gui_env;
public:
	IrrInterface(const video::E_DRIVER_TYPE deviceType = video::EDT_OPENGL, const u32& h = 640, const u32& w = 480, const u32& bits = 16, const bool& fullscreen = false, const bool& stencilBuffer = true, IEventReceiver* receiver = 0) {
		device = createDevice(deviceType, core::dimension2d<u32>(h, w), bits, fullscreen, stencilBuffer, fullscreen, receiver);
		driver = device->getVideoDriver();
		gui_env = device->getGUIEnvironment();
		scene_manager = device->getSceneManager();
	}

	virtual ~IrrInterface() {
		
	}

	virtual const int add_gui_elementsIrr() = 0;

	/**
	 *	In this function, you must have to draw everything
	 */
	virtual const int runIrr() = 0;
	/**
	 *	In this function, you must have to add every node in the scene
	 */
	virtual const int add_scene_nodesIrr() = 0;
	virtual const int add_cameraIrr() = 0;
	virtual const int quit_Irr() {
		device->closeDevice();
		device->drop();
		return 0;
	}
};