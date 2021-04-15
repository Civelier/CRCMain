#include "ByteArray.h"

ByteArray::ByteArray(size_t length)
{
	Length = length;
	m_index = 0;
	Array = (byte*)malloc(length);
}

ByteArray::ByteArray(byte data[])
{
	Length = sizeof(data);
	m_index = Length;
	Array = data;
}

bool ByteArray::Append(byte value)
{
	if (m_index < Length)
	{
		Array[m_index] = value;
		m_index++;
		return true;
	}
	return false;
}

byte& ByteArray::operator[](int index)
{
	if (index < Length)
	{
		return Array[index];
	}
	else
	{
		byte last = 0;
		return last;
	}
}

#ifdef WIN32
void ByteArray::PrintBinary()
{
	for (size_t i = 0; i < Length; i++)
	{
		std::cout << i << ':';
		byte cur_byte = Array[i];
		std::cout << cur_byte << "->";
		for (size_t ii = 0; ii < 8; ii++)
		{
			bool bit = cur_byte & (0x80 >> ii);
			std::cout << bit ? '1' : '0';
		}
		std::cout << '\n';
	}
}

void ByteArray::PrintDecimal()
{
	for (size_t i = 0; i < Length; i++)
	{
		std::cout << (short)Array[i] << '\n';
	}
}
#endif

ByteArray* ByteArray::Copy()
{
	ByteArray* arr = new ByteArray(Length);
	for (size_t i = 0; i < Length; i++)
	{
		(*arr)[i] = (*this)[i];
	}
	return arr;
}

ByteArray::operator String()
{
	ByteArray bytes = ByteArray(Length + 1);
	for (size_t i = 0; i < Length + 1; i++)
	{
		bytes[i] = (*this)[i];
	}
	return String((const char*)bytes.Array);
#ifdef WIN32
	String output = "";
	output.append((const char*)Array);
#else
	String output = String();
	output.reserve(Length + 1);
	for (size_t i = 0; i < Length + 1; i++)
	{
		output[i] = (*this)[i];
	}
#endif
	return output;
}

ByteArray::operator byte* ()
{
	return Array;
}

void ByteArray::Print()
{
	Serial.write(Array, Length);
}


ByteArray::~ByteArray()
{
	free(Array);
}
