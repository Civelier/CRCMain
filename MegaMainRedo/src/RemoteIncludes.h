#ifndef _REMOTEINCLUDES_h
#define _REMOTEINCLUDES_h

#include "RemoteDeviceManager.h"
#include "IRemoteDevice.h"
#include "IRemoteDistanceSensor.h"
#include "RemoteSR04DistanceSensor.h"
#include "RemoteDebugDevice.h"

#define REMOTE

namespace Remote
{
	typedef IRemoteDevice IDevice;
	typedef IRemoteDistanceSensor DistanceSensor;
	typedef RemoteSR04DistanceSensor SR04DistanceSensor;
	typedef RemoteDebugDevice DebugDevice;
}

#endif