/*
 Name:		MegaMain.ino
 Created:	4/14/2021 3:27:51 PM
 Author:	civel
*/

#include <Servo.h>
#include <PIDController.h>
#include <LinearEncoder.h>
#include <JankyEncoder.h>
#include <Encoder.h>
#include <LocalDeviceManager.h>
#include "LinearEncodedMotor.h"


using namespace Local;

DeviceManager Manager(2, 52);

LinearEncoder* encoCharriot;
LinearEncodedMotor* motorCharriot;
LinearEncoder* encoWind;
LinearEncodedMotor* motorWind;

#define Charriot 2
#define CharriotPWM 4
#define Wind 3
#define WindPWM 5

void WindInt()
{
	encoWind->_update();
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
	default:
		break;
	}
}

// the setup function runs once when you press reset or power the board
void setup()
{
	encoCharriot = new LinearEncoder(Charriot, 1);
	encoWind = new LinearEncoder(Wind, 1);
	motorCharriot = new LinearEncodedMotor(encoWind, CharriotPWM, 0);
	motorWind = new LinearEncodedMotor(encoWind, WindPWM, 1);

	Manager.AddDevice(motorCharriot);
	Manager.AddDevice(motorWind);

	motorCharriot->Begin();
	motorWind->Begin();
}

// the loop function runs over and over again until power down or reset
void loop()
{
	CommandManager();
}
