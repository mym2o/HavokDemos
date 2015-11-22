// This simple console demo demonstrates how to initialize memory using hkMemoryInitUtil, 
// as well as startup/teardown of Havok's core systems.

#pragma once

#include "MemoryInitUtil.h"
#include "Serialize.h"
#include "Visualize.h"
#include "Physics2012.h"
#include "Physics2012Monitor.h"
#include "Physics2012Vdb.h"
#include "Physics2012Mt.h"
#include <iostream>

void PlatformInit();
void PlatformFileSystemInit();

int HK_CALL main(int argc, const char** argv)
{
	// Perform platform specific initialization for this demo - you should already have something similar in your own code.
	PlatformInit();

	HavokInterface* demo = NULL;
	int choose = INT_MAX;
	while (choose > 0) {
		while (choose == INT_MAX) {
			printf(" Choose a demo: ");
			scanf("%i", &choose);
		}
		switch (choose)
		{
		case 1:
		{
			demo = new MemoryInitUtil();
		}
		break;
		case 2:
		{
			demo = new Serialize();
		}
		break;
		case 3:
		{
			demo = new Visualize();
		}
		break;
		case 4:
		{
			demo = new Physics2012();
		}
		break;
		case 5:
		{
			demo = new Physics2012Monitor();
		}
			break;
		case 6:
		{
			demo = new Physics2012Vdb();
		}
		break;
		case 7:
		{
			demo = new Physics2012Mt();
		}
		break;
		default:
			printf(" [Warning!] No demo selected. Please retry!\n");
			break;
		}
		if (demo) {
			demo->initHk();
			demo->run();
			demo->quitHk();
		}
		demo = NULL;

		printf(" Would you want to try another demo? (press 0 to exit)\n ");
		scanf("%i", &choose);
	}

	return 0;
}