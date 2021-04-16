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
#include <RemoteLinearEncodedMotor.h>
#include "Transmitter.h"
#include "CRCScheduler.h"

//using namespace Crc;


//CRCSchedulerClass Scheduler;
//Remote::DeviceManager Manager(2, CRC_DIG_2, CRC_DIG_1);
//Remote::RemoteLinearEncodedMotor* motorCharriot;
//Remote::RemoteLinearEncodedMotor* motorWind;
//Remote::RemoteLinearEncodedMotor* motorRotate;

//Bilda5202Motor* motorL;
//Bilda5202Motor* motorR;

struct Motor
{
	Motor(uint8_t pin, bool inverse = false)
	{
		Pin = pin;
		Inverse = inverse;
	}
	void Begin()
	{
		Crc::CrcLib::InitializePwmOutput(Pin);
	}
	void SetPercent(double value)
	{
		Crc::CrcLib::SetPwmOutputFastDouble(Pin, 0.5 + (Inverse ? -value : value) / 2);
	}
	uint8_t Pin;
	bool Inverse;
};

struct LimitSwitch
{
	LimitSwitch(uint8_t pin, bool inverse = false)
	{
		Pin = pin;
		Inverse = inverse;
	}
	void Begin()
	{
		Crc::CrcLib::SetDigitalPinMode(Pin, INPUT);
	}
	bool GetState()
	{
		return digitalRead(Pin) == (Inverse ? LOW : HIGH);
	}
	void Test()
	{
		Serial.print("state: ");
		Serial.println(GetState());
	}
	uint8_t Pin;
	bool Inverse;
};

Motor Pivot(CRC_PWM_9);
Motor Vertical(CRC_PWM_2);
Motor Horizontal(CRC_PWM_3);
LimitSwitch PivotLS(CRC_DIG_3, true);
LimitSwitch VerticalLow(CRC_DIG_4);
LimitSwitch PivotHighLS(CRC_DIG_5);

//enum MotorGroupMode
//{
//	Off,
//	HoldPosition,
//	MoveStraight,
//	TurnCentered,
//	OnStraight,
//	OnTurn,
//};

//struct MotorGroup
//{
//	MotorGroup(Bilda5202Motor* left, Bilda5202Motor* right)
//	{
//		Left = left;
//		Right = right;
//	}
//	void Begin()
//	{
//		SpeedPID.begin();
//		SpeedPID.limit(-1, 1);
//		SpeedPID.setpoint(0);
//		SpeedPID.tune(1, 0, 0);
//		PosPID.begin();
//		PosPID.limit(-1, 1);
//		PosPID.tune(PID_VALUES_BILDA_188);
//	}
//	void AdjustSpeedStraight()
//	{
//		double ratio = SpeedPID.compute((Left->GetEncoderCount() - LeftStart) - (Right->GetEncoderCount() - //RightStart));
//		double speed = PosPID.compute((Left->GetDegrees() + Right->GetDegrees()) / 2);
//		double l = (1 - ratio) * speed;
//		double r = (1 + ratio) * speed;
//		Left->SetSpeed(constrain(l, -MaxSpeed, MaxSpeed));
//		Right->SetSpeed(constrain(r, -MaxSpeed, MaxSpeed));
//	}
//
//	void AdjustSpeedTurning()
//	{
//		double ratio = SpeedPID.compute((Left->GetEncoderCount() - LeftStart) + (Right->GetEncoderCount() - //RightStart));
//		double speed = PosPID.compute((Left->GetDegrees() - Right->GetDegrees()) / 2);
//		double l = (1 - ratio) * speed;
//		double r = (1 + ratio) * -speed;
//		Left->SetSpeed(constrain(l, -1, 1));
//		Right->SetSpeed(constrain(r, -1, 1));
//	}
//
//	void AdjustSpeedGoStraight()
//	{
//		double ratio = SpeedPID.compute((Left->GetEncoderCount() - LeftStart) - (Right->GetEncoderCount() - //RightStart));
//		double l = (1 - ratio) * MaxSpeed;
//		double r = (1 + ratio) * MaxSpeed;
//		Left->SetSpeed(constrain(l, -1, 1));
//		Right->SetSpeed(constrain(r, -1, 1));
//	}
//
//	void AdjustSpeedTurnForever()
//	{
//		double ratio = SpeedPID.compute((Left->GetEncoderCount() - LeftStart) + (Right->GetEncoderCount() - //RightStart));
//		double speed = PosPID.compute((Left->GetDegrees() - Right->GetDegrees()) / 2);
//		double l = (1 - ratio) * speed;
//		double r = (1 + ratio) * -speed;
//		Left->SetSpeed(constrain(l, -1, 1));
//		Right->SetSpeed(constrain(r, -1, 1));
//	}
//
//	void Update()
//	{
//		switch (Mode)
//		{
//		case Off:
//			break;
//		case HoldPosition:
//			Left->UpdateSpeed();
//			Right->UpdateSpeed();
//			break;
//		case MoveStraight:
//			AdjustSpeedStraight();
//			break;
//		case TurnCentered:
//			AdjustSpeedTurning();
//			break;
//		case OnStraight:
//			AdjustSpeedGoStraight();
//			break;
//		default:
//			break;
//		}
//	}
//	void TurnOff()
//	{
//		Left->Off();
//		Right->Off();
//		Mode = MotorGroupMode::Off;
//	}
//
//	void Brake(bool overwrite = false)
//	{
//		Left->RotateDegrees(0, overwrite);
//		Right->RotateDegrees(0, overwrite);
//		Mode = MotorGroupMode::HoldPosition;
//	}
//
//	void OnGoStraight(double speed)
//	{
//		SpeedPID.limit(-speed, speed);
//		MaxSpeed = speed;
//		Mode = MotorGroupMode::OnStraight;
//		LeftStart = Left->GetEncoderCount();
//		RightStart = Right->GetEncoderCount();
//	}
//
//	void MoveDistance(double mm, double speed)
//	{
//		SpeedPID.limit(-speed, speed);
//		MaxSpeed = speed;
//		Mode = MotorGroupMode::MoveStraight;
//		Target = mm;
//		PosPID.setpoint(mm);
//		LeftStart = Left->GetEncoderCount();
//		RightStart = Right->GetEncoderCount();
//	}
//
//	void TurnDistance(double mm, double speed)
//	{
//		SpeedPID.limit(-speed, speed);
//		MaxSpeed = speed;
//		Mode = MotorGroupMode::TurnCentered;
//		Target = mm;
//		PosPID.setpoint(mm);
//		LeftStart = Left->GetEncoderCount();
//		RightStart = Right->GetEncoderCount();
//	}
//
//	void TurnForever(double speed)
//	{
//		SpeedPID.limit(-speed, speed);
//		MaxSpeed = speed;
//		Mode = MotorGroupMode::OnTurn;
//		LeftStart = Left->GetEncoderCount();
//		RightStart = Right->GetEncoderCount();
//	}
//
//	MotorGroupMode Mode;
//	double Target;
//	double MaxSpeed;
//	int32_t LeftStart;
//	int32_t RightStart;
//	Bilda5202Motor* Left;
//	Bilda5202Motor* Right;
//	PIDController SpeedPID;
//	PIDController PosPID;
//};

struct ServoMotor
{
	ServoMotor(uint8_t pin)
	{
		Pin = pin;
	}
	uint8_t Pin;
	void Init()
	{
		Crc::CrcLib::InitializePwmOutput(Pin);
	}
	void Write(double percent)
	{
		Crc::CrcLib::SetPwmOutputFastDouble(Pin, percent);
	}
};

struct Pince
{
	Pince(uint8_t pin1, uint8_t pin2, double open, double closed) : Servo1(pin1), Servo2(pin2)
	{
		Opened = open;
		Closed = closed;
	}
	void Init()
	{
		Servo1.Init();
		Servo2.Init();
	}
	double Opened;
	double Closed;
	ServoMotor Servo1;
	ServoMotor Servo2;
	void SetPercent(double percent)
	{
		Servo1.Write(percent);
		Servo2.Write(percent);
	}
	void Open()
	{
		Servo1.Write(Opened);
		Servo2.Write(Opened);
	}
	void Close()
	{
		Servo1.Write(Closed);
		Servo2.Write(Closed);
	}
};

Pince pince(CRC_PWM_5, CRC_PWM_6, -1, 1);
CRCSchedulerClass Scheduler;
Remote::DeviceManager Manager(2, CRC_DIG_2, CRC_DIG_1);

void CommandManager()
{
	if (!Serial.available()) return;
	int command = Serial.parseInt();
	switch (command)
	{
	case 1: // set speed
	{
		double v = Serial.parseFloat();
		Pivot.SetPercent(v);
	}
	break;
	case 2: // move pos
	{
		double d = Serial.parseFloat();
		// motorWind->MoveDistance(d, 1);
	}
	break;
	case 3: // move pince
		pince.Open();
		break;
	case 4:
		pince.Close();
		break;
	case 5:
	{
		double d = Serial.parseFloat();
		Serial.print("Pince: ");
		Serial.println(d);
		pince.SetPercent(d);
	}
	break;
	case 6:
		PivotLS.Test();
		break;
	case 7:
		PivotHighLS.Test();
		break;
	case 8:
		Pivot.SetPercent(-0.2);
		while (Serial.available()) Serial.read();
		while (!Serial.available() && !PivotHighLS.GetState())
		{
		}
		Pivot.SetPercent(0);
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
	//while (!Serial);
	Serial.println();

	myArray = (int*)malloc(sizeof(int) * 2);
	myArray[0] = 1;
	myArray[1] = 2;

	Remote::Transmit.Init();
	Manager.Init();
	//motorCharriot = new Remote::RemoteLinearEncodedMotor(0);
	//motorWind = new Remote::RemoteLinearEncodedMotor(1);
	//SETUP_DRIVING();
	Pivot.Begin();
	Vertical.Begin();
	Horizontal.Begin();
	PivotLS.Begin();
	PivotHighLS.Begin();
	VerticalLow.Begin();
	pince.Init();
	//Manager.AddDevice(motorCharriot);
	//Manager.AddDevice(motorWind);
}

#define WaitForBegin(condition) while (!(condition)) { Crc::CrcLib::Update();

#define WaitForEnd }

#define Debug(text) Serial.println(text)

void EasyRun()
{
	Debug("Start");
	Pivot.SetPercent(0.2);
	/*uint8_t s0 = millis() + 750;
	WaitForBegin(millis() > s0);
	WaitForEnd;*/
	//Debug("Up");
	//Pivot.SetPercent(0);
	WaitForBegin(PivotLS.GetState());
	WaitForEnd;
	Debug("Pivot down");
	Pivot.SetPercent(0);
	/*uint32_t s1 = millis() + 1000;
	WaitForBegin(millis() > s1);
	WaitForEnd;*/
	Debug("Stable");
	pince.Open();
	uint32_t s2 = millis() + 2000;
	WaitForBegin(millis() > s2);
	WaitForEnd;
	Debug("Open");
	Pivot.SetPercent(-0.25);
	WaitForBegin(PivotHighLS.GetState());
	WaitForEnd;

	/*uint32_t s3 = millis() + 7000;
	WaitForBegin(millis() > s3);
	WaitForEnd;*/
	Debug("Up");
	Pivot.SetPercent(0);

}

// the loop function runs over and over again until power down or reset
void loop()
{
	Crc::CrcLib::Update();

	if (digitalRead(CRC_DIG_12))
	{
		CommandManager();
	}
	else
	{
		Serial.println("ARMED!!!!!");
		while (!digitalRead(CRC_DIG_1));
		while (digitalRead(CRC_DIG_1));

		Serial.println("resetting");
		Pivot.SetPercent(-0.2);
		while (Serial.available()) Serial.read();
		while (!Serial.available() && !PivotHighLS.GetState())
		{
		}
		Pivot.SetPercent(0);

		while (!digitalRead(CRC_DIG_1));
		while (digitalRead(CRC_DIG_1));
		EasyRun();
		while (!digitalRead(CRC_DIG_1));
		while (digitalRead(CRC_DIG_1));
	}
	if (PivotLS.GetState()) Pivot.SetPercent(0);
	if (PivotHighLS.GetState()) Pivot.SetPercent(0);
	if (Manager.CheckReply())
	{
		uint8_t id = Remote::Transmit.GetReplyDeviceID();
		if (id != 255)
		{
			Serial.print("Reply from: ");
			Serial.println((int)id);
		}
	}

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
