#ifndef _IREMOTEDISTANCESENSOR_h
#define _IREMOTEDISTANCESENSOR_h

#include "IRemoteDevice.h"

namespace Remote
{
class IRemoteDistanceSensor : public IRemoteDevice
{
protected:
public:
	virtual void RequestCloserThan(uint32_t mm)
	{
		SendRequest(new DistanceRequest(ID, mm, DistanceComparison::DCLessThan));
	}
	virtual void RequestCloserThanOrEqualTo(uint32_t mm)
	{
		SendRequest(new DistanceRequest(ID, mm, DistanceComparison::DCLessThanOrEqualTo));
	}
	virtual void RequestFurtherThan(uint32_t mm)
	{
		SendRequest(new DistanceRequest(ID, mm, DistanceComparison::DCMoreThan));
	}
	virtual void RequestFurtherThanOrEqualTo(uint32_t mm)
	{
		SendRequest(new DistanceRequest(ID, mm, DistanceComparison::DCMoreThanOrEqualTo));
	}
	virtual void RequestWithinTreshold(uint32_t mm, uint32_t treshold)
	{
		SendRequest(new DistanceTresholdRequest(ID, mm, treshold, DistanceComparison::DCEqualTreshold));
	}
	virtual void RequestOutsideTreshold(uint32_t mm, uint32_t treshold)
	{
		SendRequest(new DistanceTresholdRequest(ID, mm, treshold, DistanceComparison::DCOutsideTreshold));
	}
};
}

#endif