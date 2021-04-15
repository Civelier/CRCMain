#include "RemoteDeviceManager.h"

namespace Remote
{
DeviceManager::DeviceManager(size_t count, uint8_t resetPin, uint8_t replyAvailable)
{
	m_devices = (IRemoteDevice**)malloc(sizeof(IRemoteDevice*) * count);
	m_resetPin = resetPin;
	m_replyAvailablePin = replyAvailable;
}

void DeviceManager::Init()
{
	Crc::CrcLib::SetDigitalPinMode(m_resetPin, OUTPUT);
	Crc::CrcLib::SetDigitalPinMode(m_replyAvailablePin, INPUT);
	digitalWrite(m_resetPin, LOW);
	delay(100);
	digitalWrite(m_resetPin, HIGH);
	Serial.println("Waiting for reply...");
	while (!Crc::CrcLib::GetDigitalInput(m_replyAvailablePin));
	while (Crc::CrcLib::GetDigitalInput(m_replyAvailablePin));
	Serial.println("SUCESS!");
	delay(10);
}

void DeviceManager::AddDevice(IRemoteDevice* device)
{
	device->ID = m_count;
	m_devices[m_count++] = device;
}
bool DeviceManager::CheckReply()
{
	return digitalRead(m_replyAvailablePin) == HIGH;
}
DeviceManager::~DeviceManager()
{
	free(m_devices);
}
}


