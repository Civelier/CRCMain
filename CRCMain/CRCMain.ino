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
#include <Adafruit_VL53L0X.h> //Pour capteur I2c IR
#include <Adafruit_NeoPixel.h> //Neopixel du Crcduino
#include "Bilda5202Motor.h" //Ma library des moteurs encoder
#include "RemoteDeviceManager.h" //Fait partie de ma library: Communication entre crc et Mega
#include "RemoteIncludes.h" //Includes specifiques au cote remote de la communication
#include "LinkedList.h" //Ma library de liste dynamique
#include <RemoteLinearEncodedMotor.h> //Janky encoder a distance
#include "Transmitter.h" //Communication au Mega (ma library)
//#include "CRCScheduler.h" //Le scheduler (trop instable pour etre utilliser mais je le garde pcq le code marche avec et je veux pas prendre le temps de l'enlever et d'avoir a le remettre apres si on en a besoin)


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
	Motor(uint8_t pin, bool inverse = false)
	{
		Pin = pin;
		Inverse = inverse;
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
		Crc::CrcLib::SetPwmOutputFastDouble(Pin, 0.5 + (Inverse ? -value : value) / 2);
	}
	uint8_t Pin;
	bool Inverse;
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

Motor MoteurPivot(CRC_PWM_9);
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

Pince pinceCrabCrab(CRC_PWM_5, CRC_PWM_6, -1, 1);
//CRCSchedulerClass Scheduler;
Remote::DeviceManager Manager(2, CRC_DIG_2, CRC_DIG_1);
Bilda5202Motor* Rotation;

void CommandManager()
{
	if (!Serial.available()) return;
	int command = Serial.parseInt();
	switch (command)
	{
	case 1: // set speed
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
	default:
		break;
	}
}

//int* myArray;
void setup() 
{
	Crc::CrcLib::Initialize();
	Serial.begin(115200);
	//while (!Serial);
	Serial.println();

	/*myArray = (int*)malloc(sizeof(int) * 2);
	myArray[0] = 1;
	myArray[1] = 2;*/

	//Initialize I2C pour Mega
	Remote::Transmit.Init();

	//Initialize les Device deportes (protocole de communication avec Mega)
	Manager.Init();

	//motorCharriot = new Remote::RemoteLinearEncodedMotor(0);
	//motorWind = new Remote::RemoteLinearEncodedMotor(1);

	Rotation = new Bilda5202Motor(CRC_PWM_1, CRC_ENCO_A, CRC_ENCO_B);
	Rotation->RotateDegrees(0); 
	//SETUP_DRIVING();

	//Moteurs pas encoder
	MoteurPivot.Begin();
	Vertical.Begin();
	Horizontal.Begin();
	//Limit switches
	PivotLS.Begin();
	PivotHighLS.Begin();
	VerticalLow.Begin();
	//La pince (2 servo-moteurs)
	pinceCrabCrab.Init();
	//Manager.AddDevice(motorCharriot);
	//Manager.AddDevice(motorWind);
}

#define WaitForBegin(condition) while (!(condition)) { Crc::CrcLib::Update();

#define WaitForEnd }

#define Debug(text) Serial.println(text)

void EasyRun()
{
	Debug("Start");
	MoteurPivot.SetPercent(0.2);
	/*uint8_t s0 = millis() + 750;
	WaitForBegin(millis() > s0);
	WaitForEnd;*/
	//Debug("Up");
	//Pivot.SetPercent(0);
	WaitForBegin(PivotLS.GetState());
	WaitForEnd;
	Debug("Pivot down");
	MoteurPivot.SetPercent(0);
	/*uint32_t s1 = millis() + 1000;
	WaitForBegin(millis() > s1);
	WaitForEnd;*/
	Debug("Stable");
	pinceCrabCrab.Open();
	uint32_t s2 = millis() + 2000;
	WaitForBegin(millis() > s2);
	WaitForEnd;
	Debug("Open");
	MoteurPivot.SetPercent(-0.25);
	WaitForBegin(PivotHighLS.GetState());
	WaitForEnd;

	/*uint32_t s3 = millis() + 7000;
	WaitForBegin(millis() > s3);
	WaitForEnd;*/
	Debug("Up");
	MoteurPivot.SetPercent(0);

}

// the loop function runs over and over again until power down or reset
void loop()
{
	Crc::CrcLib::Update();

	
	if (digitalRead(CRC_DIG_12)) //Pin Debug 
	{
		//Mode Debug
		CommandManager();
	}
	else
	{
		Serial.println("ARMED!!!!!");
		//Attend bouton reset du mega
		while (!digitalRead(CRC_DIG_1));
		while (digitalRead(CRC_DIG_1));

		Serial.println("resetting");
		MoteurPivot.SetPercent(-0.2);
		while (Serial.available()) Serial.read();
		while (!Serial.available() && !PivotHighLS.GetState())
		{
		}
		MoteurPivot.SetPercent(0);

		while (!digitalRead(CRC_DIG_1));
		while (digitalRead(CRC_DIG_1));
		EasyRun();
		while (!digitalRead(CRC_DIG_1));
		while (digitalRead(CRC_DIG_1));
	}
	if (PivotLS.GetState()) MoteurPivot.SetPercent(0);
	if (PivotHighLS.GetState()) MoteurPivot.SetPercent(0);
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
