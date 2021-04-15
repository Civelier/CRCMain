#ifndef _LOCALDEBUGDEVICE_h
#define _LOCALDEBUGDEVICE_h

#include "ILocalDevice.h"
namespace Local
{
class LocalDebugDevice : public ILocalDevice
{
public:
	LocalDebugDevice();
	virtual bool Test();
};
}

#endif