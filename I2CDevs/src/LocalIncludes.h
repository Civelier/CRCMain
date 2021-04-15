#ifndef _LOCALINCLUDES_h
#define _LOCALINCLUDES_h

#include "LocalDeviceManager.h"
#include "ILocalDevice.h"
#include "ILocalDistanceSensor.h"
#include "LocalSR04DistanceSensor.h"
#include "LocalDebugDevice.h"
#include "Reciever.h"

#define LOCAL

namespace Local
{
typedef ILocalDevice IDevice;
typedef ILocalDistanceSensor DistanceSensor;
typedef LocalSR04DistanceSensor SR04DistanceSensor;
typedef LocalDebugDevice DebugDevice;
}

#endif