#ifndef _LOCALDEVICEMANAGER_h
#define _LOCALDEVICEMANAGER_h

#include "ILocalDevice.h"
#include "Request.h"
#include "Arduino.h"

namespace Local
{
class DeviceManager
{
private:
	ILocalDevice** m_devices;
	size_t m_count;
	uint8_t m_replyPin;
public:
	uint8_t LastEventDeviceID = 255;
public:
	DeviceManager(size_t count, uint8_t replyPin);
	void Init();
	void AddDevice(ILocalDevice* device);
	void HandleRequest(Packet request);
	void Run();
	~DeviceManager();
};
}

#endif