#include "Transmitter.h"

Remote::Transmitter::Transmitter()
{
	
}

void Remote::Transmitter::Init()
{
	Wire.begin(CRCduinoAddress);
	Wire.setClock(400000);
}

uint8_t Remote::Transmitter::GetReplyDeviceID()
{
	Wire.requestFrom(SlaveAddress, 1);
	return (uint8_t)Wire.read();
}

void Remote::Transmitter::SendRequest(uint8_t* buff)
{
	/*Serial.print("Count: ");
	Serial.println(buff[0]);*/
	Wire.beginTransmission(SlaveAddress);
	Wire.write(buff, buff[0]);
	Wire.endTransmission();
}
