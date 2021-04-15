#ifndef _LOCALSR04DISTANCESENSOR_h
#define _LOCALSR04DISTANCESENSOR_h

#include "ILocalDistanceSensor.h"
namespace Local
{
class LocalSR04DistanceSensor : public ILocalDistanceSensor
{
private:
	uint8_t m_trig;
	uint8_t m_echo;
	uint32_t m_lastDistance;
public:
	LocalSR04DistanceSensor(uint8_t trig, uint8_t echo);
	virtual uint32_t GetDistance();
	void Debug();
};
}

#endif