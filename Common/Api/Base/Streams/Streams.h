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
//	This demo does not produce output. It is a source code example which shows:									//
//	* loading and saving from memory buffers																	//
//	* hooking havok streams into your streams																	//
//																												//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "../../../../HavokDefinitions.h"

#include <Common/Base/System/Io/IStream/hkIStream.h>
#include <Common/Base/System/Io/OStream/hkOStream.h>
#include <Common/Base/System/Io/Reader/hkStreamReader.h>
#include <Common/Base/System/Io/Writer/hkStreamWriter.h>
#include <Common/Base/System/Io/Reader/Buffered/hkBufferedStreamReader.h>

namespace StreamsHK {

	//implement only the basic operations and let a proxy class
	//do the hard work
	struct MyStreamReader : public hkStreamReader {
		MyStreamReader() {}
		virtual hkBool isOk() const { return true; }
		virtual int read(void* buf, int nbytes) { return 0; } // dummy implementation
	};

	class Streams : public HavokInterface {
	private:
		hkBool m_done;
	public:
		Streams();
		~Streams();

		void initHk();
		void runHk();
		void quitHk();

		//Output Streams
		static void functionTakingOstream(hkOstream& os);
		static void functionTakingStreamWriter(hkStreamWriter* writer);
		static void streamToFixedSizeMemory();
		static void streamToExpandingMemory();
		static void streamToFile();

		//Input Streams
		static void functionTakingIstream(hkIstream& is);
		static void functionTakingStreamReader(hkStreamReader* reader);
		static void streamFromFixedSizeMemory();
		static void streamFromFile();
		static void bufferingCustomInput();
	};
}