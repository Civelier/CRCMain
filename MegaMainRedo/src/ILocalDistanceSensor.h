#ifndef _ILOCALDISTANCESENSOR_h
#define _ILOCALDISTANCESENSOR_h

#include "ILocalDevice.h"
#include <Wire.h>

namespace Local
{
class ILocalDistanceSensor : public ILocalDevice
{
protected:
	ILocalDistanceSensor() : ILocalDevice()
	{ }
public:
	virtual uint32_t GetDistance() = 0;
	virtual bool Test()
	{
		/*
		* [0]: Length
		* [1]: ID
		* [2]: Type/mode
		* [...]: Type-specific information
		*/
		static uint8_t state;
		DistanceTresholdRequest* req = reinterpret_cast<DistanceTresholdRequest*>(m_req);
		if (state != m_req[2])// && Wire.available() == 0)
		{
			/*Serial.print("State of sensor #");
			Serial.print(ID);
			Serial.print(" is: ");*/
			state = m_req[2];
			/*Serial.println(state);
			if (state > 0)
			{
				Serial.print("MM: "); Serial.println(req->MM);
			}
			if (state > 0 && state <= 2)
			{
				Serial.print("Treshold: "); Serial.println(req->Treshold);
			}*/
		}
		if (state == 0)
		{
			return false;
		}
		int32_t d = GetDistance();
		//if (Wire.available() == 0) Serial.println(d);
		switch (req->Comparisson)
		{
		case DistanceComparison::DCLessThan:
			return d < req->MM;
		case DistanceComparison::DCLessThanOrEqualTo:
			return d <= req->MM;
		case DistanceComparison::DCMoreThan:
			return d > req->MM;
		case DistanceComparison::DCMoreThanOrEqualTo:
			return d >= req->MM;
		case DistanceComparison::DCEqualTreshold:
			return d >= req->MM - req->Treshold &&
				req->MM + req->Treshold >= d;
		case DistanceComparison::DCOutsideTreshold:
			return d <= req->MM - req->Treshold &&
				req->MM + req->Treshold <= d;
		default:
			break;
		}
	}
};
}

#endif