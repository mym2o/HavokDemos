#include "Streams.h"

using namespace StreamsHK;

Streams::Streams() {
	m_done = false;
}

Streams::~Streams() {

}

void Streams::initHk() {
	memoryRouter = hkMemoryInitUtil::initDefault(hkMallocAllocator::m_defaultMallocAllocator, hkMemorySystem::FrameInfo(1024 * 1024));
	hkBaseSystem::init(memoryRouter, errorReport);
}

void Streams::runHk() {
	if (m_done == false) {
		m_done = true;
		hkReferencedObject::lockAll();

		//writing
		Streams::streamToFixedSizeMemory();
		Streams::streamToExpandingMemory();
		Streams::streamToFile();

		//reading
		Streams::streamFromFixedSizeMemory();
		Streams::streamFromFile();

		//custom input
		Streams::bufferingCustomInput();

		hkReferencedObject::unlockAll();
	}
}

void Streams::quitHk() {
	hkBaseSystem::quit();
	hkMemoryInitUtil::quit();
}

//ostreams are used for formatted text output
void Streams::functionTakingOstream(hkOstream& os) {

}

//StreamWriters used for low level byte output
void Streams::functionTakingStreamWriter(hkStreamWriter* writer) {

}

void Streams::streamToFixedSizeMemory() {
	char buf[100];

	//create a stream from an in memory buffer
	//overrun bytes are simple discarded
	//passing true keeps the buffer null terminated
	hkOstream os(buf, sizeof(buf), true);

	//write to the stream
	Streams::functionTakingOstream(os);

	//hkOstream is a convenient way of creating hkStreamWriter instances
	//It creates and manages the reference to its hkStreamWriter
	functionTakingStreamWriter(os.getStreamWriter());

	errorReport("Streams::streamToFixedSizeMemory()\n");
}

void Streams::streamToExpandingMemory() {
	hkArray<char> buf;

	//If we know the buffer is likely to be small,
	//we could avoid a heap allocation
	//hkInplaceArray<char, 128> buf;

	//buf will automatically expand as needed
	hkOstream os(buf);

	//as before
	functionTakingOstream(os);
	functionTakingStreamWriter(os.getStreamWriter());

	//we now have buf.getSize() bytes of data from buf.begin()

	errorReport("Streams::streamToExpandingMemory()\n");
}

void Streams::streamToFile() {
	//opening with a name forwards to hkFileSystem
	//to do the platform specific calls and filename munging
	//(such as prefixing host0:, replacing backslahs, etc)
	hkOstream os("filename.txt");

	functionTakingOstream(os);

	hkStreamWriter* writer = os.getStreamWriter();

	//the underlying hkStreamWriter are reference counted
	//so we can let it be destroyed with the ostream (default)
	//or keep it alive by adding a reference.
	//In that case we also have responsibility to removeReference.
	writer->addReference();
	functionTakingStreamWriter(writer);
	writer->removeReference();

	errorReport("Streams::streamToFile()\n");
}

//Istreams are used for formatted text input
void Streams::functionTakingIstream(hkIstream& is) {

}

//StreamReaders used for low level byte input
void Streams::functionTakingStreamReader(hkStreamReader* reader) {

}

void Streams::streamFromFixedSizeMemory() {
	//fill these buffers externally
	char buf[100];
	hkArray<char> array;

	//create a stream from an in memory buffer
	hkIstream is(buf, sizeof(buf));
	hkIstream is2(array.begin(), array.getSize());

	functionTakingIstream(is);

	//hkIstream is a convenient way of creating hkStreamReader instances
	//it creates and manages the reference to its hkStreamReader
	functionTakingStreamReader(is.getStreamReader());

	errorReport("Streams::streamFromFixedSizeMemory()\n");
}

void Streams::streamFromFile() {
	hkIstream is("filename.txt");

	functionTakingIstream(is);
	functionTakingStreamReader(is.getStreamReader());

	errorReport("Streams::streamFromFile()\n");
}

void Streams::bufferingCustomInput() {
	MyStreamReader mystream;

	//Real code would probably new/addReference/removeReference
	hkBufferedStreamReader buffered(&mystream);

	//stream now behaves as if it supported rewinding
	functionTakingStreamReader(&buffered);

	errorReport("Streams::bufferingCustomInput()\n");
}