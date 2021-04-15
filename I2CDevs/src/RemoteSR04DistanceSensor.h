#ifndef _REMOTESR04DISTANCESENSOR_h
#define _REMOTESR04DISTANCESENSOR_h

#include "IRemoteDistanceSensor.h"

namespace Remote
{
class RemoteSR04DistanceSensor : public IRemoteDistanceSensor
{
private:
public:
	RemoteSR04DistanceSensor();
	//virtual void RequestCloserThan(int32_t mm);
	//virtual void RequestCloserThanOrEqualTo(int32_t mm);
	//virtual void RequestFurtherThan(int32_t mm);
	//virtual void RequestFurtherThanOrEqualTo(int32_t mm);
	//virtual void RequestWithinTreshold(int32_t mm, int32_t treshold);
};
}

#endif