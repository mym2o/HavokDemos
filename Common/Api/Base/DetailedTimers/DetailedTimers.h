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
//	The purpose of this example is to show how to get most detailed timer information possible from havok.		//
//																												//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "HavokDefinitions.h"

#include <Common/Base/Types/Color/hkColor.h>

#include <Common/Base/Thread/Pool/hkCpuThreadPool.h>

#include <Common/Base/Monitor/MonitorStreamAnalyzer/hkMonitorStreamAnalyzer.h>
#include <Common/Base/System/Stopwatch/hkStopwatch.h>
#include <Common/Base/Thread/Thread/hkWorkerThreadContext.h>

class DetailedTimers : public HavokInterface {
private:
	int m_step;

	void timeDetailedTimers(const int iterations, const char* fileName);
public:
	DetailedTimers();
	~DetailedTimers();

	void initHk();
	void run();
	void quitHk();
};