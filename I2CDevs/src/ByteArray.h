#ifndef _BYTEARRAY_h
#define _BYTEARRAY_h

#ifdef WIN32
#include "ArduinoEmulation.h"
#else
#include "Arduino.h"
#endif

class ByteArray
{
private:
	size_t m_index;
public:
	size_t Length;
	byte* Array;
public:
	ByteArray(size_t length);
	ByteArray(byte data[]);
	bool Append(byte value);
	byte& operator[](int);
#ifdef WIN32
	void PrintBinary();
	void PrintDecimal();
#endif
	ByteArray* Copy();
	explicit operator String();
	operator byte*();
	void Print();
	~ByteArray();
};

#endif