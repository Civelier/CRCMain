#ifndef _BYTEUNIONS_h
#define _BYTEUNIONS_h

#ifdef WIN32
#include "ArduinoEmulation.h"
#else
#include "Arduino.h"
#endif

union ShortByteUnion
{
	short asShort;
	byte asBytes[2];
};

union ByteShortUnion
{
	byte asBytes[2];
	short asShort;
};

union UShortByteUnion
{
	uint16_t asUShort;
	byte asBytes[2];
};

union ByteUShortUnion
{
	byte asBytes[2];
	uint16_t asUShort;
};

union IntByteUnion
{
	int asInt;
	byte asBytes[4];
};

union ByteIntUnion
{
	byte asBytes[4];
	int asInt;
};

union UIntByteUnion
{
	unsigned int asUInt;
	byte asBytes[4];
};

union ByteUIntUnion
{
	byte asBytes[4];
	unsigned int asUInt;
};

union LongByteUnion
{
	long asLong;
	byte asBytes[4];
};

union ByteLongUnion
{
	byte asBytes[4];
	long asLong;
};

union ULongByteUnion
{
	unsigned long asULong;
	byte asBytes[4];
};

union ByteULongUnion
{
	byte asBytes[4];
	unsigned long asULong;
};

union FloatByteUnion
{
	float asFloat;
	byte asBytes[4];
};

union ByteFloatUnion
{
	byte asBytes[4];
	float asFloat;
};
#endif