
#ifndef _ISERIALIZABLE_h
#define _ISERIALIZABLE_h

#ifdef WIN32
#include "ArduinoEmulation.h"
#else
#include "Arduino.h"
#endif
#include "ByteArray.h"

class ISerializable
{
protected:
	ISerializable() { }
public:
	ISerializable(ByteArray* data) { }
	virtual size_t Size() { return sizeof(this); }
	virtual ByteArray* Serialize() { return nullptr; }
	virtual void Deserialize(ByteArray* data) { }
};

#endif