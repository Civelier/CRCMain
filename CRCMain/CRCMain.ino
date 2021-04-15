/*
 Name:		CRCMain.ino
 Created:	4/14/2021 3:10:42 PM
 Author:	civel
*/

#define LOCAL_ADDRESS 0x16
#define REMOTE_ADDRESS 0x17

// the setup function runs once when you press reset or power the board
#include <Wire.h>
#include <Servo.h>
#include <PIDController.h>
#include <MPU6050.h>
#include <Encoder.h>
#include <CrcLib.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#include "Bilda5202Motor.h"
#include "RemoteDeviceManager.h"
#include "RemoteIncludes.h"
#include "LinkedList.h"
#include <RemoteLinearEncodedMotor.h>
#include "Transmitter.h"

//using namespace Crc;

Remote::DeviceManager Manager(2, CRC_DIG_2, CRC_DIG_1);
Remote::RemoteLinearEncodedMotor* motorCharriot;
Remote::RemoteLinearEncodedMotor* motorWind;

void CommandManager()
{
	if (!Serial.available()) return;
	int command = Serial.parseInt();
	switch (command)
	{
	case 1: // set speed
	{
		double v = Serial.parseFloat();
		motorWind->SetPercent(v);
	}
	break;
	case 2: // move pos
	{
		double d = Serial.parseFloat();
		motorWind->MoveDistance(d, 1);
	}
	break;
	default:
		break;
	}
}

int* myArray;
void setup() 
{
	Crc::CrcLib::Initialize();
	Serial.begin(115200);
	while (!Serial);
	Serial.println();

	myArray = (int*)malloc(sizeof(int) * 2);
	myArray[0] = 1;
	myArray[1] = 2;

	Remote::Transmit.Init();
	Manager.Init();
	motorCharriot = new Remote::RemoteLinearEncodedMotor(0);
	motorWind = new Remote::RemoteLinearEncodedMotor(1);

	Manager.AddDevice(motorCharriot);
	Manager.AddDevice(motorWind);
}

// the loop function runs over and over again until power down or reset
void loop()
{
	Crc::CrcLib::Update();
	CommandManager();
	if (Manager.CheckReply())
	{
		uint8_t id = Remote::Transmit.GetReplyDeviceID();
		if (id != 255)
		{
			Serial.print("Reply from: ");
			Serial.println((int)id);
		}
	}
}
