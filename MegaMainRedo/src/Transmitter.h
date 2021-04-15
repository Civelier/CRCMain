#ifndef _TRANSMITTER_h
#define _TRANSMITTER_h

#include <Wire.h>
#include "Addresses.h"
#include "Arduino.h"

namespace Remote
{
class Transmitter
{
private:
public:
	Transmitter();
	void Init();
	uint8_t GetReplyDeviceID();
	void SendRequest(uint8_t* buff);
};

extern Transmitter Transmit;

}


#endif