#ifndef _BINARYSERIALIZER_h
#define _BINARYSERIALIZER_h

#include "ISerializable.h"
#ifdef WIN32
#include "ArduinoEmulation.h"
#else
#include "Arduino.h"
#endif
#include "ByteArray.h"
#include "ByteUnions.h"

class BinarySerializer
{
private:
	ByteArray* m_array;
	bool m_isReading;
	size_t m_index = 0;
public:
	size_t Size;
private:
	void GetBytes(byte* arr, size_t length);
public:
	BinarySerializer();
	BinarySerializer(ByteArray* arr);
	void AddByteSize();
	void AddShortSize();
	void AddIntSize();
	void AddLongSize();
	void AddFLoatSize();
	void AddSize(size_t size);
	void AddSize(ISerializable* serializable);

	void GenerateArray();

	void Add(char value);
	void Add(byte value);
	/*void Add(short value);
	void Add(uint16_t value);*/
	void Add(int value);
	//void Add(unsigned int value);
	void Add(long value);
	void Add(unsigned long value);
	void Add(float value);
	void Add(String value);
	void Add(ISerializable* serializable);

	char GetChar();
	byte GetByte();
	/*short GetShort();
	uint16_t GetUShort();*/
	int GetInt();
	unsigned int GetUInt();
	long GetLong();
	unsigned long GetULong();
	float GetFloat();
	String GetString();
	String GetString(size_t length);
	void GetSerializable(ISerializable* value);

	ByteArray* GetArray();

	~BinarySerializer();
};

#define _APPEND_UNION(myUnion)\
for (size_t i = 0; i < sizeof(myUnion); i++)\
{\
	m_array->Append(myUnion.asBytes[i]);\
}

#endif