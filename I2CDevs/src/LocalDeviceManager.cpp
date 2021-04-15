#include "LocalDeviceManager.h"

namespace Local
{
DeviceManager::DeviceManager(size_t count, uint8_t replyPin)
{
	m_devices = (ILocalDevice**)malloc(sizeof(ILocalDevice*) * count);
	m_replyPin = replyPin;
	pinMode(m_replyPin, OUTPUT);
}

void DeviceManager::Init()
{
	digitalWrite(m_replyPin, HIGH);
	delay(10);
	digitalWrite(m_replyPin, LOW);
	Serial.println("Ready");
}

void DeviceManager::AddDevice(ILocalDevice* device)
{
	device->ID = m_count;
	m_devices[m_count++] = device;
}

void DeviceManager::HandleRequest(Packet request)
{
	m_devices[request[1]]->Decode(request);
}

void DeviceManager::Run()
{
	for (size_t i = 0; i < m_count; i++)
	{
		bool res = m_devices[i]->Test();
		if (res)
		{
			Serial.print("Condition for device \'");
			Serial.print(i);
			Serial.println("\' was obtained!");
			digitalWrite(m_replyPin, HIGH);
			LastEventDeviceID = (uint8_t)i;
		}
		else if (i == LastEventDeviceID)
		{
			digitalWrite(m_replyPin, LOW);
			LastEventDeviceID = (uint8_t)255;
		}
	}
}

DeviceManager::~DeviceManager()
{
	free(m_devices);
}
}

