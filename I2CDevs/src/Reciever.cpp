#include "Reciever.h"
namespace Local
{
void Reciever::RecieveService(int count)
{
	Reciever instance = GetInstance();
	Wire.readBytes(instance.m_buff, count);
	instance.m_manager->HandleRequest(instance.m_buff);
	instance.m_available = true;
	//Serial.println("Recieved message");
	//Serial.print("Count: "); Serial.println(count);
}

void Reciever::RequestService()
{
	Reciever instance = GetInstance();
	Wire.write(&(instance.m_manager->LastEventDeviceID), 1);
}

Reciever::Reciever(DeviceManager* manager)
{
	m_manager = manager;
	Wire.begin(SlaveAddress);
	Wire.setClock(400000);
	Wire.onReceive(RecieveService);
	Wire.onRequest(RequestService);
}

bool Reciever::RequestAvailable()
{
	return m_available;
}

Reciever& Reciever::GetInstance(DeviceManager* manager)
{
	static Reciever instance = manager == nullptr ? instance : Reciever(manager);
	return instance;
}

Packet Reciever::ReadRequest()
{
	m_available = false;
	return m_buff;
}
}
