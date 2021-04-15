/*
 Name:		MegaMain.ino
 Created:	4/14/2021 3:27:51 PM
 Author:	civel
*/

#define LOCAL_ADDRESS 0x16
#define REMOTE_ADDRESS 0x17


#include <Servo.h>
#include <PIDController.h>
#include <LinearEncoder.h>
#include <JankyEncoder.h>
#include <Encoder.h>
#include <LocalDeviceManager.h>
#include "LinearEncodedMotor.h"
#include "Wire.h"
#include "Addresses.h"
#include "Reciever.h"

using namespace Local;

DeviceManager Manager = DeviceManager(2, 52);

LinearEncoder* encoCharriot;
LinearEncodedMotor* motorCharriot;
LinearEncoder* encoWind;
LinearEncodedMotor* motorWind;
uint8_t Buffer[MaxMessageLength];

#define Charriot 2
#define CharriotPWM 4
#define Wind 3
#define WindPWM 5

void WindInt()
{
	encoWind->_update();
	digitalWrite(13, digitalRead(Wind));
}

void CharriotInt()
{
	encoCharriot->_update();
}

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

// the setup function runs once when you press reset or power the board
void setup()
{
	pinMode(13, OUTPUT);
	Wire.begin(SlaveAddress);
	Serial.begin(115200);
	encoCharriot = new LinearEncoder(Charriot, 1);
	encoWind = new LinearEncoder(Wind, 1);
	motorCharriot = new LinearEncodedMotor(encoWind, CharriotPWM, 0);
	motorWind = new LinearEncodedMotor(encoWind, WindPWM, 1);
	
	Manager.Init();
	Reciever::GetInstance(&Manager);


	Manager.AddDevice(motorCharriot);
	Manager.AddDevice(motorWind);

	motorCharriot->Begin();
	motorWind->Begin();

	pinMode(Charriot, INPUT_PULLUP);
	pinMode(Wind, INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(Charriot), &CharriotInt, CHANGE);
	attachInterrupt(digitalPinToInterrupt(Wind), &WindInt, CHANGE);

}

// the loop function runs over and over again until power down or reset
void loop()
{
	Manager.Run();
	CommandManager();
	motorCharriot->Update();
	//motorWind->Update();
	/*static uint32_t nextMillis = 0;
	if (nextMillis < millis())
	{
		nextMillis = millis() + 500;
		Serial.println(encoWind->GetPosition());
	}*/
	if (Reciever::GetInstance().RequestAvailable())
	{
		Serial.println("Request recieved");
	}
}
