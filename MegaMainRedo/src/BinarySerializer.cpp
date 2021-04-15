#include "BinarySerializer.h"

void BinarySerializer::GetBytes(byte* arr, size_t length)
{
	for (size_t i = 0; i < length; i++)
	{
		arr[i] = (*m_array)[i + m_index];
	}
	m_index += length;
}

BinarySerializer::BinarySerializer()
{
	Size = 0;
	m_array = nullptr;
	m_isReading = false;
}

BinarySerializer::BinarySerializer(ByteArray* arr)
{
	Size = arr->Length;
	m_array = arr;
	//m_array->PrintBinary();
	m_isReading = true;
}

void BinarySerializer::AddByteSize()
{
	AddSize(sizeof(char));
}

void BinarySerializer::AddShortSize()
{
	AddSize(sizeof(uint16_t));
}

void BinarySerializer::AddIntSize()
{
	AddSize(sizeof(int));
}

void BinarySerializer::AddLongSize()
{
	AddSize(sizeof(long));
}

void BinarySerializer::AddFLoatSize()
{
	AddSize(sizeof(float));
}

void BinarySerializer::AddSize(size_t size)
{
	if (m_isReading) m_index += size;
	else Size += size;
}

void BinarySerializer::AddSize(ISerializable* serializable)
{
	if (m_isReading) m_index += serializable->Size();
	else Size += serializable->Size();
}

void BinarySerializer::GenerateArray()
{
	m_array = new ByteArray(Size);
}

void BinarySerializer::Add(char value)
{
	m_array->Append(value);
}

void BinarySerializer::Add(byte value)
{
	m_array->Append(value);
}

//void BinarySerializer::Add(short value)
//{
//	ShortByteUnion u = { value };
//	_APPEND_UNION(u)
//}
//
//void BinarySerializer::Add(uint16_t value)
//{
//	UShortByteUnion u = { value };
//	_APPEND_UNION(u)
//}

void BinarySerializer::Add(int value)
{
	IntByteUnion u = { value };
	_APPEND_UNION(u)
}

//void BinarySerializer::Add(unsigned int value)
//{
//	UIntByteUnion u = { value };
//	_APPEND_UNION(u)
//}

void BinarySerializer::Add(long value)
{
	LongByteUnion u = { value };
	_APPEND_UNION(u)
}

void BinarySerializer::Add(unsigned long value)
{
	ULongByteUnion u = { value };
	_APPEND_UNION(u)
}

void BinarySerializer::Add(float value)
{
	FloatByteUnion u = { value };
	_APPEND_UNION(u)
}

void BinarySerializer::Add(String value)
{
	int length = value.length();

	for (size_t i = 0; i < value.length(); i++)
	{
		char c = value[i];
		m_array->Append(c);
	}
}

void BinarySerializer::Add(ISerializable* serializable)
{
	ByteArray* data = serializable->Serialize();
	for (size_t i = 0; i < data->Length; i++)
	{
		m_array->Append((*data)[i]);
	}
	delete data;
}

char BinarySerializer::GetChar()
{
	return GetByte();
}

byte BinarySerializer::GetByte()
{
	byte b = (*m_array)[m_index];
	m_index++;
	return b;
}

//short BinarySerializer::GetShort()
//{
//	byte b[2];
//	GetBytes(b, 2);
//	ByteShortUnion u = { b[0], b[1] };
//	return u.asShort;
//}
//
//uint16_t BinarySerializer::GetUShort()
//{
//	byte b[2];
//	GetBytes(b, 2);
//	ByteUShortUnion u = { b[0], b[1] };
//	return u.asUShort;
//}

int BinarySerializer::GetInt()
{
	byte b[4];
	GetBytes(b, 4);
	ByteIntUnion u = { b[0], b[1], b[2], b[3] };
	return u.asInt;
}

unsigned int BinarySerializer::GetUInt()
{
	byte b[4];
	GetBytes(b, 4);
	ByteUIntUnion u = { b[0], b[1], b[2], b[3] };
	return u.asUInt;
}

long BinarySerializer::GetLong()
{
	byte b[4];
	GetBytes(b, 4);
	ByteLongUnion u = { b[0], b[1], b[2], b[3] };
	return u.asLong;
}

unsigned long BinarySerializer::GetULong()
{
	byte b[4];
	GetBytes(b, 4);
	ByteULongUnion u = { b[0], b[1], b[2], b[3] };
	return u.asULong;
}

float BinarySerializer::GetFloat()
{
	byte b[4];
	GetBytes(b, 4);
	ByteFloatUnion u = { b[0], b[1], b[2], b[3] };
	return u.asFloat;
}

String BinarySerializer::GetString()
{
	return GetString(Size - m_index);
}

String BinarySerializer::GetString(size_t length)
{
	ByteArray arr = ByteArray(length + 1);
	GetBytes(arr.Array, length);
	arr[length] = 0;
	return String((const char*)arr.Array);
}

void BinarySerializer::GetSerializable(ISerializable* value)
{
	ByteArray* arr = new ByteArray(value->Size());
	for (size_t i = 0; i < arr->Length; i++)
	{
		(*arr)[i] = GetByte();
	}
	value->Deserialize(arr);
	delete arr;
}

ByteArray* BinarySerializer::GetArray()
{
	m_isReading = true;
	return m_array;
}

BinarySerializer::~BinarySerializer()
{
	//if (!m_isReading && m_array != nullptr) m_array->~ByteArray();
}

