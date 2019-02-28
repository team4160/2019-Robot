/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <iostream>
#include "frc/ADXRS450_Gyro.h"
#include "frc/BuiltInAccelerometer.h"
#include "frc/Compressor.h"
#include "frc/DoubleSolenoid.h"
#include "frc/DriverStation.h"
#include "frc/GenericHID.h"
#include "frc/I2C.h"
#include "frc/Joystick.h"
#include "frc/PowerDistributionPanel.h"
#include "frc/Preferences.h"
#include "frc/RobotDrive.h"
#include "frc/SPI.h"
#include "frc/Solenoid.h"
#include "frc/Threads.h"
#include "frc/TimedRobot.h"
#include "frc/Timer.h"
#include "frc/WPIErrors.h"
#include "frc/XboxController.h"
#include "frc/buttons/InternalButton.h"
#include "frc/buttons/JoystickButton.h"
#include "frc/buttons/NetworkButton.h"
#include "frc/buttons.h"
#include "frc/drive/DifferentialDrive.h"
#include "frc/filters/LinearDigitalFilter.h"
#include "frc/smartdashboard/SendableChooser.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <ctre/Phoenix.h>

using namespace frc;

class Robot : public frc::TimedRobot
{
  public:
	static constexpr int kTimeoutMs = 0; //change this to 0 if you don't want verification
	static constexpr int kEncoderUnit = 4096;
	static constexpr double turnSensitivity = 0.6;

	//Setting up the TalonSRX's config
	static constexpr double driveRampTime = 0.2;
	static constexpr int driveCurrentLimit = 0;
	static constexpr int driveMaxCurrent = 0;
	static constexpr int driveMaxTime = 0;

	static constexpr double clawRampTime = 0;
	static constexpr int clawCurrentLimit = 20;
	static constexpr int clawMaxCurrent = 40; //claw shouldn't need a lot of current
	static constexpr int clawMaxTime = 100;

	static constexpr double elevatorRampTime = 0.3;
	static constexpr int elevatorCurrentLimit = 5;
	static constexpr int elevatorMaxCurrent = 10;
	static constexpr int elevatorMaxTime = 100;

	static constexpr int clawForwardLimit = -150; //eg. 5 rotations TODO test the top soft limit
	static constexpr int clawReverseLimit = -850; //TODO test bottom soft limit

	//Creating the TalonSRXs and sensors
	Joystick *Joystick1, *Joystick2;
	Button *BCargoCollect, *BClawIn, *BCargoMid, *BCargoLow, *BClawOut;
	Button *BHatchLow, *BHatchMid, *BHatchOut, *BElevatorHome;
	Button *BClimbPistons, *BClimbArmOn, *BClimbFrontOff, *BArmOff, *BClimbBackOff;
	Button *BClawUp, *BClawDown;
	Button *BElevatorUp, *BElevatorDown;
	WPI_TalonSRX *DBLeft, *DBLeft2;
	WPI_TalonSRX *DBRight, *DBRight2;
	WPI_TalonSRX *Claw, *Claw2, *ClawLeft, *ClawRight;
	WPI_TalonSRX *Elevator1, *Elevator2;
	WPI_TalonSRX *ClimbArm;
	DoubleSolenoid *ClimbFront, *ClimbBack;
	Solenoid *Hatch, *ClimbWheel;
	CANifier *ClawSensor;
	DifferentialDrive *db;
	ADXRS450_Gyro *gyro;
	BuiltInAccelerometer *accel;
	PowerDistributionPanel *PDP;
	Timer *mytimer;

	void MotorBuilder(WPI_TalonSRX *srx, bool brake, bool inverted, double RampTime, int CurrentLimit, int MaxCurrent, int MaxTime);
	void RobotInit() override;
	void RobotPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;

  private:
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;

	unsigned int driveState;

	double left;
	double right;
	double turn;
	double driveSpeed;
	double clawLeftSpeed;
	double clawRightSpeed;
	double ClawSpeed;
	double ClawHold;
	double ElevatorSpeed;

	bool isClawHomed = false;
	bool isElevatorHomed = false;
	bool flagElevatorDown = false;
	bool flagElevatorMid = false;
	bool flagElevatorHigh = false;
	bool flagClawUp = false;
};

enum PS4
{
	Square = 1,			   //
	Cross = 2,			   //
	Circle = 3,			   //
	Triangle = 4,		   //
	L1 = 5,				   //
	R1 = 6,				   //
	L2 = 7,				   //
	R2 = 8,				   //
	Share = 9,			   //
	Options = 10,		   //
	L3 = 11,			   //
	R3 = 12,			   //
	PS = 13,			   //
	Pad = 14,			   //
	PSLeftStickRight = 0,  //
	PSLeftStickDown = 1,   //
	PSRightStickRight = 2, //
	L2In = 3,			   //start at -1 and goes to 1      ((L2In)+1)/2 0 to 100%
	R2In = 4,			   //start at -1 and goes to 1
	PSRightStickDown = 5   //
	//POV up is 0 none is -1
};

enum XB1
{						   //TODO get XBox mapping
	A = 1,				   //
	B = 2,				   //
	X = 3,				   //
	Y = 4,				   //
	LB = 5,				   //
	RB = 6,				   //
	View = 7,			   //hi
	Menu = 8,			   //
	LS = 9,				   //
	RS = 10,			   //
	XBLeftStickRight = 0,  //
	XBLeftStickDown = 1,   //
	LIn = 2,			   //0 to 1
	RIn = 3,			   //0 to 1
	XBRightStickRight = 4, //
	XBRightStickDown = 5   //
	//POV up is 0 none is -1
};

enum Attack
{
	Right = 0,			//
	Down = 1,			//
	ReverseThrottle = 2 //
};

enum kPDP
{
	DBLeft = 13,	//
	DBLeft2 = 0,	//
	DBRight = 3,	//
	DBRight2 = 2,   //
	Claw = 14,		//
	ClawLeft = 11,  //
	ClawRight = 4,  //
	Elevator1 = 15, //
	Elevator2 = 0,  //
	ClimbArm = 1,  //
};