#pragma once

#include "StandAloneDemos\MemoryInitUtil.h"
#include "StandAloneDemos\Serialize.h"
#include "StandAloneDemos\Visualize.h"
#include "StandAloneDemos\Physics2012.h"
#include "StandAloneDemos\Physics2012Monitor.h"
#include "StandAloneDemos\Physics2012Vdb.h"
#include "StandAloneDemos\Physics2012Mt.h"
#include "Common\Api\Base\DetailedTimers\DetailedTimers.h"
#include "Common\Api\Base\Streams\Streams.h"
#include "Physics2012\Api\Collide\Broadphase\Culling\BroadphaseCulling.h"
#include "Physics2012\Api\Collide\MultiPendulums\MultiPendulums.h"
#include "Physics2012\Api\Collide\CollisionFiltering\CollisionFilter\CollisionFilter.h"
#include "Physics2012\Api\Collide\CollisionFiltering\ConstraintCollisionFilter\ConstraintCollisionFilter.h"
#include "Physics2012\Api\Collide\CollisionFiltering\PairCollisionFilter\PairCollisionFilter.h"
#include "Physics2012\Api\Collide\ContactPointCallbacks\ContactState\ContactState.h"

#include <iostream>

#include <driverChoice.h>

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
			printf(" Choose a demo (0 to exit): ");
			scanf("%i", &choose);
		}
		switch (choose)
		{
		case 0:
			break;
		case 1:
		{
			//No 3D
			demo = new MemoryInitUtil();
		}
		break;
		case 2:
		{
			//No 3D
			demo = new Serialize();
		}
		break;
		case 3:
		{
			//TODO: 3D
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
		case 8:
		{
			demo = new DetailedTimers();
		}
		break;
		case 9:
		{
			using namespace StreamsHK;
			demo = new Streams();
		}
		break;
		case 10:
		{
			demo = new BroadphaseCulling();
		}
		break;
		case 11:
		{
			demo = new MultiPendulums();
		}
		break;
		case 12:
		{
			demo = new CollisionFilter();
		}
		break;
		case 13:
		{
			demo = new ConstraintCollisionFilter();
		}
		break;
		case 14:
		{
			demo = new PairCollisionFilter();
		}
		break;
		case 15:
		{
			demo = new ContactState();
		}
		break;
		default:
			printf(" [Warning!] No demo selected. Please retry!\n");
			printf(" 1. MemoryInitUtil\n 2. Serialize\n 3. Visualize\n 4. Physics2012\n 5. Physics2012Monitor\n " \
					"6. Physics2012Vdb\n 7. Physics2012Mt\n 8. DetailedTimers\n 9. Streams\n 10. BroadphaseCulling\n " \
					"11. MultiPendulums\n 12. CollisionFilter\n 13. ConstraintCollisionFilter\n 14. PairCollisionFilter\n " \
					"15. ContactState\n 0. Exit\n");
			break;
		}
		if (demo) {
			demo->initHk();
			demo->runHk();
			demo->quitHk();
		}
		demo = NULL;

		if (choose != 0) {
			printf(" Would you want to try another demo? (press 0 to exit)\n ");
			scanf("%i", &choose);
		}
	}

	return 0;
}

ContactState::~ContactState() {
}