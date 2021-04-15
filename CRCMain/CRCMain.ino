/*
 Name:		CRCMain.ino
 Created:	4/14/2021 3:10:42 PM
 Author:	civel
*/



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


//using namespace Crc;



void setup() 
{
	Crc::CrcLib::Initialize();
}

// the loop function runs over and over again until power down or reset
void loop()
{
	Crc::CrcLib::Update();
}
