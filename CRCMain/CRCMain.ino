/*
 Name:		CRCMain.ino
 Created:	4/14/2021 3:10:42 PM
 Author:	civel
*/

//Includes
#include <Wire.h> //Pour I2C
#include <Servo.h> //Pour servos et moteurs
#include <PIDController.h> //PID
#include <Encoder.h> //Pour moteurs encoder
#include <CrcLib.h> //Pour CRCDuino
#include <Adafruit_NeoPixel.h> //Neopixel du Crcduino
#include "Bilda5202Motor.h" //Ma library des moteurs encoder
//#include "CRCScheduler.h" //Le scheduler (trop instable pour etre utilliser mais je le garde pcq le code marche avec et je veux pas prendre le temps de l'enlever et d'avoir a le remettre apres si on en a besoin)

Bilda5202Motor* TestMotor;

/// <summary>
/// Moteur simple
/// </summary>
struct Motor
{
	/// <summary>
	/// Constructeur
	/// </summary>
	/// <param name="pin">PWM pin of the motor</param>
	/// <param name="inverse">Is the motor reversed</param>
	Motor(uint8_t pin)
	{
		Pin = pin;
	}
	/// <summary>
	/// Initialize the motor
	/// </summary>
	void Begin()
	{
		Crc::CrcLib::InitializePwmOutput(Pin);
	}
	/// <summary>
	/// Set the motor speed
	/// </summary>
	/// <param name="value">value from -1 to 1 describing the speed</param>
	void SetPercent(double value)
	{
		Crc::CrcLib::SetPwmOutputFastDouble(Pin, 0.5 + value / 2);
	}
	uint8_t Pin;
};


/// <summary>
/// Limit switch simple
/// </summary>
struct LimitSwitch
{
	/// <summary>
	/// Constructor
	/// </summary>
	/// <param name="pin">Digital IO pin</param>
	/// <param name="inverse">Is the switch NC</param>
	LimitSwitch(uint8_t pin, bool inverse = false)
	{
		Pin = pin;
		Inverse = inverse;
	}
	/// <summary>
	/// 
	/// </summary>
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

Motor MoteurPivot(CRC_PWM_1);
Motor MoteurVertical(CRC_PWM_2);
Motor MoteurHorizontal(CRC_PWM_3);
Motor MoteurRotation(CRC_PWM_4);
LimitSwitch PivotLS(CRC_DIG_3, true);
LimitSwitch VerticalLow(CRC_DIG_4);
LimitSwitch PivotHighLS(CRC_DIG_5);
LimitSwitch VerticalHigh(CRC_DIG_6);
LimitSwitch HorizontalMin(CRC_DIG_7);
LimitSwitch HorizontalMax(CRC_DIG_8);

#pragma region Driving

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

#pragma endregion




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
	void Stop()
	{
		Crc::CrcLib::UnInitializePwmOutput(Pin);
	}
	void Start()
	{
		Crc::CrcLib::ReInitializePwmOutput(Pin);
	}
};

struct Pince
{
	Pince(uint8_t pin1, uint8_t pin2) : Servo1(pin1), Servo2(pin2)
	{
	}
	void Init()
	{
		Servo1.Init();
		Servo2.Init();
	}
	ServoMotor Servo1;
	ServoMotor Servo2;
	void SetPercent(double percent)
	{
		Servo1.Write(percent);
		Servo2.Write(percent);
	}
	void Open()
	{
		Servo1.Write(-1);
		Servo2.Write(-1);
	}
	void Close()
	{
		Servo1.Write(1);
		Servo2.Write(1);
	}
};

Pince pinceCrabCrab(CRC_PWM_5, CRC_PWM_6);
ServoMotor ServoLacheBalle(CRC_PWM_7);
//CRCSchedulerClass Scheduler;
//Remote::DeviceManager Manager(2, CRC_DIG_2, CRC_DIG_1);
//Bilda5202Motor* MoteurRotation;

void DebugCRC()
{
	while (true)
	{
		if (!Serial.available()) return;
		int command = Serial.parseInt();
		switch (command)
		{
		case 1: // set speed Pivot
		{
			double v = Serial.parseFloat();
			MoteurPivot.SetPercent(v);
		}
		break;
		case 2: // move pos
		{
			double d = Serial.parseFloat();
			// motorWind->MoveDistance(d, 1);
		}
		break;
		case 3: // move pince
			pinceCrabCrab.Open();
			break;
		case 4:
			pinceCrabCrab.Close();
			break;
		case 5:
		{
			double d = Serial.parseFloat();
			Serial.print("Pince: ");
			Serial.println(d);
			pinceCrabCrab.SetPercent(d);
		}
		break;
		case 6:
			PivotLS.Test();
			break;
		case 7:
			PivotHighLS.Test();
			break;
		case 8:
			MoteurPivot.SetPercent(-0.2);
			while (Serial.available()) Serial.read();
			while (!Serial.available() && !PivotHighLS.GetState())
			{
			}
			MoteurPivot.SetPercent(0);
			break;
		case 9:
		{
			double speed = Serial.parseFloat();
			MoteurRotation.SetPercent(speed);
		}
		case 10: 
		{
			double speed = Serial.parseFloat();
			int miliseconds = Serial.parseInt();
			MoteurVertical.SetPercent(speed);
			uint32_t m = millis() + miliseconds;
			while (millis() < m)
			{
				Crc::CrcLib::Update();
				if (VerticalHigh.GetState()) MoteurVertical.SetPercent(0);
				if (VerticalLow.GetState()) MoteurVertical.SetPercent(0);
			}
		}
		default:
			break;
		}

		//Failsafe du moteur pivot
		if (PivotLS.GetState()) MoteurPivot.SetPercent(0);
		if (PivotHighLS.GetState()) MoteurPivot.SetPercent(0);

		if (HorizontalMax.GetState()) MoteurHorizontal.SetPercent(0);
		if (HorizontalMin.GetState()) MoteurHorizontal.SetPercent(0);

		if (VerticalHigh.GetState()) MoteurVertical.SetPercent(0);
		if (VerticalLow.GetState()) MoteurVertical.SetPercent(0);
	}
}

//int* myArray;
void setup() 
{
	TestMotor = new Bilda5202Motor(CRC_PWM_4, CRC_ENCO_A, CRC_ENCO_B);
	Crc::CrcLib::Initialize();
	Serial.begin(115200);
	Serial.println();
	//while (!Serial);


	//Initialize I2C pour Mega
	//Remote::Transmit.Init();

	//Initialize les Device deportes (protocole de communication avec Mega)
	//Manager.Init();

	//motorCharriot = new Remote::RemoteLinearEncodedMotor(0);
	//motorWind = new Remote::RemoteLinearEncodedMotor(1);

	//MoteurRotation->RotateDegrees(0); 
	//SETUP_DRIVING();

	//Moteurs pas encoder
	MoteurPivot.Begin();
	MoteurVertical.Begin();
	MoteurHorizontal.Begin();
	//MoteurRotation.Begin();

	//Buttons
	Crc::CrcLib::SetDigitalPinMode(CRC_DIG_1, INPUT);
	Crc::CrcLib::SetDigitalPinMode(CRC_DIG_2, INPUT_PULLUP);

	//Limit switches
	PivotLS.Begin();
	PivotHighLS.Begin();
	VerticalLow.Begin();
	VerticalHigh.Begin();
	HorizontalMax.Begin();
	HorizontalMin.Begin();
	ServoLacheBalle.Init();
	ServoLacheBalle.Stop();
	//La pince (2 servo-moteurs)
	pinceCrabCrab.Init();
	//Manager.AddDevice(motorCharriot);
	//Manager.AddDevice(motorWind);

	pinceCrabCrab.Open();
	delay(3000);
	pinceCrabCrab.Close();
}

#define WaitFor(condition) while (!(condition)) { Crc::CrcLib::Update(); }

#define WaitForBegin(condition) while (!(condition)) { Crc::CrcLib::Update();

#define WaitForEnd }

#define Debug(text) Serial.println(text)

void ProgramJoute()
{
	Debug("Start");
	MoteurPivot.SetPercent(0.2);
	WaitFor(PivotLS.GetState());
	Debug("Pivot down");
	MoteurPivot.SetPercent(0);
	Debug("Stable");
	pinceCrabCrab.Open();
	uint32_t s2 = millis() + 2000;
	WaitFor(millis() > s2);
	Debug("Open");
	MoteurPivot.SetPercent(-0.25);
	WaitFor(PivotHighLS.GetState());
	Debug("Up");
	MoteurPivot.SetPercent(0);


	//Balle
	ServoLacheBalle.Start();
	ServoLacheBalle.Write(1);
	uint32_t s3 = millis() + 3000;
	WaitFor(millis() > s3);
	ServoLacheBalle.Write(-1);
	ServoLacheBalle.Stop();

	MoteurPivot.SetPercent(0.2);
	WaitFor(PivotLS.GetState());
	MoteurPivot.SetPercent(0);

	//Reset
	MoteurPivot.SetPercent(-0.25);
	WaitFor(PivotHighLS.GetState());
	Debug("Up");
	MoteurPivot.SetPercent(0);
}

void TestBilda()
{

	if (!Serial.available()) return;
	double deg = Serial.parseFloat();
	TestMotor->RotateToDegrees(deg);
}

void ProgramRotate()
{
	uint32_t s2;
	Debug("Move 1");
	MoteurRotation.SetPercent(0.99);
	s2 = millis() + 1020;
	WaitFor(millis() > s2);
	MoteurRotation.SetPercent(0);
	WaitFor(digitalRead(CRC_DIG_1));
	WaitFor(!digitalRead(CRC_DIG_1));
	Debug("Move -1");
	MoteurRotation.SetPercent(-0.99);
	s2 = millis() + 1020;
	WaitFor(millis() > s2);
	MoteurRotation.SetPercent(0);
	WaitFor(digitalRead(CRC_DIG_1));
	WaitFor(!digitalRead(CRC_DIG_1));
}

void PosInitiale()
{
	Serial.println("ARMED!!!!!");
	//Attend bouton reset du mega
	while (!digitalRead(CRC_DIG_1));
	while (digitalRead(CRC_DIG_1));

	Serial.println("resetting");
	MoteurPivot.SetPercent(-0.2);
	while (!PivotHighLS.GetState())
	{}
	MoteurPivot.SetPercent(0);

	while (Serial.available()) Serial.read();
	while (!Serial.available() && !PivotHighLS.GetState())
	{
	}
}


// the loop function runs over and over again until power down or reset
void loop()
{
	Crc::CrcLib::Update();
	
	TestBilda();
	TestMotor->UpdateSpeed();
	if (!digitalRead(CRC_DIG_1))
	{
		//Crc::CrcLib::SetPwmOutputFastDouble(CRC_PWM_4, 1);
		//DebugCRC();
		//ProgramJoute();
		//ProgramRotate();
		//double degree = 90;
		////MoteurRotation->RotateToDegrees(degree);
		//MoteurRotation->SetPercent(0.5);
		//MoteurRotation->Reset();
		//while (MoteurRotation->GetDegrees() < 90);
		//MoteurRotation->SetPercent(0);

	}
	//MoteurRotation->UpdateSpeed();
	/*if (!digitalRead(CRC_DIG_2))
	{
		PosInitiale();
	}*/
}
