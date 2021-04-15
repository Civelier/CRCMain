#ifndef _RECIEVER_h
#define _RECIEVER_h

#include "Wire.h"
#include "Request.h"
#include "Addresses.h"
#include "LocalDeviceManager.h"

namespace Local
{
class Reciever
{
private:
	bool m_available;
	Packet m_buff = new uint8_t[MaxMessageLength];
	DeviceManager* m_manager;
private:
	static void RecieveService(int count);
	static void RequestService();
	Reciever(DeviceManager* manager);
public:
	static Reciever& GetInstance(DeviceManager* manager = nullptr);
	bool RequestAvailable();
	Packet ReadRequest();
};
}

#endif