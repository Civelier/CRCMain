
#ifndef _ILOCALDEVICE_h
#define _ILOCALDEVICE_h

#include "Request.h"
#include "Arduino.h"

namespace Local
{
class ILocalDevice
{
protected:
	Packet m_req = new uint8_t[MaxMessageLength];
public:
	static uint8_t LastRequestID;
protected:
	ILocalDevice() 
	{ 
		ID = 0;
		for (size_t i = 0; i < MaxMessageLength; i++)
		{
			m_req[i] = 0;
		}
	}
public:
	uint8_t ID;
	virtual void Decode(Packet request)
	{
		Serial.print("Request decoded for device #");
		Serial.println(ID);
		for (size_t i = 0; i < request[0]; i++)
		{
			m_req[i] = request[i];
		}
	}
	virtual bool Test() = 0;
};
}


#endif