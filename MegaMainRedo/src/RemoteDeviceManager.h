#ifndef _REMOTEDEVICEMANAGER_h
#define _REMOTEDEVICEMANAGER_h

#include "Request.h"
#include "IRemoteDevice.h"
#include "Arduino.h"
#include "CrcLib.h"

namespace Remote
{
class DeviceManager
{
private:
	IRemoteDevice** m_devices;
	size_t m_count;
	uint8_t m_resetPin;
	uint8_t m_replyAvailablePin;
public:
	DeviceManager(size_t count, uint8_t resetPin, uint8_t replyAvailable);
	void Init();
	void AddDevice(IRemoteDevice* device);
	bool CheckReply();
	~DeviceManager();
};
}

#endif