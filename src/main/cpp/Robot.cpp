/* Rules
 * 1: If you require sensors then have a method to timeout if sensor has failed
 * 2: Avoid while loops (stops the robot from processing anything else)
 * 3: If using the same code over and over then make a function
 * 4: Test your changes when possible
 * 5: Do not feature creep
 * 
 * http://first.wpi.edu/FRC/roborio/release/docs/cpp/namespacefrc.html
 * http://www.ctr-electronics.com/downloads/api/cpp/html/index.html
 */
#include "Robot.h"

void Robot::MotorBuilder(WPI_TalonSRX *srx, bool brake = true, bool inverted = false, double RampTime = 0, int CurrentLimit = 20, int MaxCurrent = 20, int MaxTime = 100)
{
	srx->SetInverted(inverted);
	srx->ConfigOpenloopRamp(RampTime, kTimeoutMs);
	srx->ConfigContinuousCurrentLimit(CurrentLimit, kTimeoutMs);
	srx->ConfigPeakCurrentLimit(MaxCurrent, kTimeoutMs);
	srx->ConfigPeakCurrentDuration(MaxTime, kTimeoutMs);
	if (CurrentLimit > 0)
		srx->EnableCurrentLimit(true);
	else
		srx->EnableCurrentLimit(false);
}

void Robot::RobotInit()
{
	// m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
	// m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
	// frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

	driveState = 0; //set to tank

	// Setting the Controllers
	Driver = new Joystick(0);
	Operator = new Joystick(1);
	OperatorPanel = new Joystick(2);

	gyro = new ADXRS450_Gyro(SPI::kOnboardCS0);
	PDP = new PowerDistributionPanel(0);
	accel = new BuiltInAccelerometer();
	mytimer = new Timer();

	DBLeft = new WPI_TalonSRX(1);
	DBLeft2 = new WPI_TalonSRX(2);
	DBRight = new WPI_TalonSRX(3);
	DBRight2 = new WPI_TalonSRX(4);
	Claw = new WPI_TalonSRX(5);
	Claw2 = new WPI_TalonSRX(6);
	ClawLeft = new WPI_TalonSRX(7);
	ClawRight = new WPI_TalonSRX(8);
	Elevator1 = new WPI_TalonSRX(9);
	Elevator2 = new WPI_TalonSRX(10);
	ClimbWheel = new WPI_TalonSRX(11);

	ClawSensor = new CANifier(21);

	theCompressor = new Compressor(0);

	Hatch = new Solenoid(2); // PCM ID is 0
	ClimbArm = new Solenoid(5);
	ClimbFront = new DoubleSolenoid(0, 1); // foward,reverse
	ClimbBack = new DoubleSolenoid(7, 6);

	db = new DifferentialDrive(*DBLeft, *DBRight);

	// set followers
	DBLeft2->Set(ControlMode::Follower, DBLeft->GetDeviceID());
	DBRight2->Set(ControlMode::Follower, DBRight->GetDeviceID());
	Elevator2->Set(ControlMode::Follower, Elevator1->GetDeviceID());
	Claw2->Set(ControlMode::Follower, Claw->GetDeviceID());

	// Motor Builder(&Motor,brake,invert,Ramp,limit,maxlimit,maxtime)
	MotorBuilder(DBLeft, /*brake*/ true, /*invert*/ false, driveRampTime, driveCurrentLimit, driveMaxCurrent, driveMaxTime);
	MotorBuilder(DBLeft2, /*brake*/ true, /*invert*/ false, driveRampTime, driveCurrentLimit, driveMaxCurrent, driveMaxTime);
	MotorBuilder(DBRight, /*brake*/ true, /*invert*/ false, driveRampTime, driveCurrentLimit, driveMaxCurrent, driveMaxTime);
	MotorBuilder(DBRight2, /*brake*/ true, /*invert*/ false, driveRampTime, driveCurrentLimit, driveMaxCurrent, driveMaxTime);
	MotorBuilder(Claw, /*brake*/ true, /*invert*/ false, clawRampTime, clawCurrentLimit, clawMaxCurrent, clawMaxTime);
	MotorBuilder(Claw2, /*brake*/ true, /*invert*/ false, clawRampTime, clawCurrentLimit, clawMaxCurrent, clawMaxTime);
	MotorBuilder(ClawLeft, /*brake*/ true, /*invert*/ false, clawRampTime, clawCurrentLimit, clawMaxCurrent, clawMaxTime);
	MotorBuilder(ClawRight, /*brake*/ true, /*invert*/ true, clawRampTime, clawCurrentLimit, clawMaxCurrent, clawMaxTime);
	MotorBuilder(Elevator1, /*brake*/ true, /*invert*/ true, elevatorRampTime, elevatorCurrentLimit, elevatorMaxCurrent, elevatorMaxTime);
	MotorBuilder(Elevator2, /*brake*/ true, /*invert*/ true, elevatorRampTime, elevatorCurrentLimit, elevatorMaxCurrent, elevatorMaxTime);
	MotorBuilder(ClimbWheel, /*brake*/ true, /*invert*/ true, elevatorRampTime, elevatorCurrentLimit, elevatorMaxCurrent, elevatorMaxTime);

	// Add CANifier encoder
	ClawSensor->ConfigVelocityMeasurementPeriod(CANifierVelocityMeasPeriod::Period_100Ms, kTimeoutMs);
	ClawSensor->ConfigVelocityMeasurementWindow(64, kTimeoutMs);
	ClawSensor->SetStatusFramePeriod(CANifierStatusFrame::CANifierStatusFrame_Status_2_General, /*refresh rate*/ 10, kTimeoutMs); /* speed up quadrature & DIO */

	// attach CANifier to Claw motor
	Claw->ConfigRemoteFeedbackFilter(ClawSensor->GetDeviceNumber(), RemoteSensorSource::RemoteSensorSource_CANifier_Quadrature, /*REMOTE*/ 0, kTimeoutMs);
	Claw->ConfigRemoteFeedbackFilter(0x00, RemoteSensorSource::RemoteSensorSource_Off, /*REMOTE*/ 1, kTimeoutMs); //turn off second sensor for claw
	Claw->ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, /*PID_PRIMARY*/ 0, kTimeoutMs);

	// TODO Claw PID See 10.1 set P = 1 I = 10 + maybe don't override but use website for now Claw->Config_kP(/*slot*/ 0, 1, kTimeoutMs);
	Claw->Config_kP(/*slot*/ 0, 0.5, kTimeoutMs);
	Claw->Config_kD(/*slot*/ 0, 5, kTimeoutMs);
	Claw->SetSelectedSensorPosition(0, 0, 0);

	// TODO create soft encoder limits when you found positions
	Claw->ConfigReverseSoftLimitThreshold(200, kTimeoutMs);
	Claw->ConfigReverseSoftLimitEnable(true, kTimeoutMs);
	// Claw->ConfigReverseSoftLimitThreshold(clawReverseLimit, kTimeoutMs);
	Claw->ConfigForwardSoftLimitEnable(false, kTimeoutMs);

	// elevator sensors
	Elevator1->ConfigRemoteFeedbackFilter(0x00, RemoteSensorSource::RemoteSensorSource_Off, /*REMOTE*/ 0, kTimeoutMs); //no remote sensors
	Elevator1->ConfigRemoteFeedbackFilter(0x00, RemoteSensorSource::RemoteSensorSource_Off, /*REMOTE*/ 1, kTimeoutMs);
	Elevator1->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, kTimeoutMs);
	// Elevator1->ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 0);

	Elevator1->SetSensorPhase(true);
	Elevator1->SetSelectedSensorPosition(0, 0, 0);
	Elevator1->Config_kP(/*slot*/ 0, 0.45, kTimeoutMs);
	Elevator1->Config_kD(/*slot*/ 0, 5, kTimeoutMs);
	Elevator1->Config_kF(/*slot*/ 0, 0.045, kTimeoutMs);

	// Elevator1->SetSelectedSensorPosition(0, /*REMOTE*/ 0, /*TimeOut*/ 0);
	// Elevator1->ConfigForwardSoftLimitThreshold(-600, 0);
	Elevator1->ConfigForwardSoftLimitEnable(false, 0);
	Elevator1->ConfigReverseSoftLimitThreshold(300, 0);
	Elevator1->ConfigReverseSoftLimitEnable(true, 0);

	// Elevator1->ConfigForwardSoftLimitEnable(false, 0);
	// Elevator1->ConfigReverseSoftLimitEnable(false, 0);

	gyro->Calibrate(); //takes around 5 seconds to execute and must not move
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit()
{
	Claw->Set(ControlMode::PercentOutput, 0);
	Elevator1->Set(ControlMode::PercentOutput, 0);
	theCompressor->Start();
}

void Robot::AutonomousPeriodic()
{
	Periodic();
	// if (Operator->GetRawButton(bOperator::bClimbBackOff))
	// {
	// 	ClimbBack->Set(DoubleSolenoid::Value::kReverse);
	// 	ClimbFront->Set(DoubleSolenoid::Value::kReverse);
	// }
	// else
	// {
	// 	ClimbBack->Set(DoubleSolenoid::Value::kForward);
	// 	ClimbFront->Set(DoubleSolenoid::Value::kForward);
	// }
}

void Robot::TeleopInit()
{
	Claw->Set(ControlMode::PercentOutput, 0);
	Elevator1->Set(ControlMode::PercentOutput, 0);
	theCompressor->Stop();
}

void Robot::TeleopPeriodic()
{
	Periodic();
}
void Robot::Periodic()
{
	//DRIVER CONTROLS
	//Tank
	// left = (Driver->GetRawAxis(PS4::PSLeftStickDown));
	// left = ((turnSensitivity * left * left * left) + (1 - turnSensitivity) * left);
	// right = (Driver->GetRawAxis(PS4::PSRightStickDown));
	// right = ((turnSensitivity * right * right * right) + (1 - turnSensitivity) * right);
	// db->TankDrive(left, right, false);

	//Arcade
	// driveSpeed = (Driver->GetRawAxis(PS4::PSRightStickDown));
	// left = (Driver->GetRawAxis(PS4::PSLeftStickRight));
	// left = ((turnSensitivity * left * left * left) + (1 - turnSensitivity) * left);
	// db->ArcadeDrive(driveSpeed, left, /*squaredInputs*/ false);

	//Curvature
	db->CurvatureDrive(Driver->GetRawAxis(PS4::PSRightStickDown), Driver->GetRawAxis(PS4::PSLeftStickRight) * -1, true);

	//SMART DASHBOARD
	// frc::SmartDashboard::PutNumber("Gyroscope", gyro->GetAngle());
	// frc::SmartDashboard::PutNumber("POV", Operator->GetPOV());
	// frc::SmartDashboard::PutNumber("Drive Left", DBLeft->GetSelectedSensorPosition(0));
	// frc::SmartDashboard::PutNumber("Drive Left Pulse", DBLeft->GetSensorCollection().GetPulseWidthPosition());
	// frc::SmartDashboard::PutNumber("Drive Left Quad", DBLeft->GetSensorCollection().GetQuadraturePosition());
	// frc::SmartDashboard::PutNumber("Drive Right", DBRight->GetSelectedSensorPosition(0));
	// frc::SmartDashboard::PutNumber("Drive Right Pulse", DBRight->GetSensorCollection().GetPulseWidthPosition());
	// frc::SmartDashboard::PutNumber("Drive Right Quad", DBRight->GetSensorCollection().GetQuadraturePosition());
	// frc::SmartDashboard::PutNumber("Elevator", Elevator1->GetSelectedSensorPosition(0));
	// frc::SmartDashboard::PutNumber("Elevator Pulse", Elevator1->GetSensorCollection().GetPulseWidthPosition());
	// frc::SmartDashboard::PutNumber("Elevator Quad", Elevator1->GetSensorCollection().GetQuadraturePosition());
	// frc::SmartDashboard::PutNumber("Elevator Reverse Limit", Elevator1->GetSensorCollection().IsRevLimitSwitchClosed());
	// frc::SmartDashboard::PutNumber("Claw", Claw->GetSelectedSensorPosition(0));
	// frc::SmartDashboard::PutNumber("Claw Pulse", Claw->GetSensorCollection().GetPulseWidthPosition());
	// frc::SmartDashboard::PutNumber("Claw Quad", Claw->GetSensorCollection().GetQuadraturePosition());
	// frc::SmartDashboard::PutNumber("Claw Forward Limit", ClawSensor->GetGeneralInput(ClawSensor->LIMF));

	// frc::SmartDashboard::PutNumber("DBLeft", PDP->GetCurrent(kPDP::DBLeft));
	// frc::SmartDashboard::PutNumber("DBLeft2", PDP->GetCurrent(kPDP::DBLeft2));
	// frc::SmartDashboard::PutNumber("DBRight", PDP->GetCurrent(kPDP::DBRight));
	// frc::SmartDashboard::PutNumber("DBRight2", PDP->GetCurrent(kPDP::DBRight2));
	// frc::SmartDashboard::PutNumber("Elevator1", PDP->GetCurrent(kPDP::Elevator1));
	// frc::SmartDashboard::PutNumber("Elevator2", PDP->GetCurrent(kPDP::Elevator2));
	// frc::SmartDashboard::PutNumber("ClimbArm", PDP->GetCurrent(kPDP::ClimbArm));
	// frc::SmartDashboard::PutNumber("Claw current", PDP->GetCurrent(kPDP::Claw));
	// frc::SmartDashboard::PutNumber("ClawLeft current", PDP->GetCurrent(kPDP::ClawLeft));
	// frc::SmartDashboard::PutNumber("ClawRight current", PDP->GetCurrent(kPDP::ClawRight));
	// frc::SmartDashboard::PutNumber("Drive Mode", driveState);

	//OPERATOR CONTROLS
	//Claw intake
	ClawSpeed = Operator->GetRawAxis(XB1::LIn) - Operator->GetRawAxis(XB1::RIn);
	ClawLeft->Set(ClawSpeed);
	ClawRight->Set(ClawSpeed);

	//Panel ejection
	if (Operator->GetRawButton(XB1::RB))
		Hatch->Set(true);
	else
		Hatch->Set(false);

	ClawHold = Operator->GetRawAxis(XB1::XBRightStickDown);
	if (ClawHold < -0.05 || ClawHold > 0.05)
	{
		Claw->Set(ControlMode::PercentOutput, ClawHold * 0.5);
		ClawFirstRun = true;
	}
	else
	{
		if (ClawFirstRun)
		{
			Claw->Set(ControlMode::Position, Claw->GetSelectedSensorPosition(0));
			ClawFirstRun = false;
		}
	}

	ElevatorHold = Operator->GetRawAxis(XB1::XBLeftStickDown) * -1;
	if (ElevatorHold < -0.05 || ElevatorHold > 0.05)
	{
		Elevator1->Set(ControlMode::PercentOutput, ElevatorHold * 0.5);
		ElevatorFirstRun = true;
	}
	else
	{
		if (ElevatorFirstRun)
		{
			Elevator1->Set(ControlMode::Position, Elevator1->GetSelectedSensorPosition(0));
			ElevatorFirstRun = false;
		}
	}
	// if (Operator->GetRawButton(XB1::A))
	// 	Elevator1->Set(ControlMode::Position, 1000);
	// if (Operator->GetRawButton(XB1::X))
	// 	Elevator1->Set(ControlMode::Position, 4300);
	// if (Operator->GetRawButton(XB1::B))
	// 	Claw->Set(ControlMode::Position, 500);
	// if (Operator->GetRawButton(XB1::Y))
	// 	Claw->Set(ControlMode::Position, 2200);

	//TODO all of them need claw first to a certain degree
	//CARGO cargoship
	if (Operator->GetPOV() == 90)
	{
		Elevator1->Set(ControlMode::Position, 7300);
		Claw->Set(ControlMode::Position, 6000);
	}
	//CARGO rocket mid TODO change angle of claw, need actual rocket
	if (Operator->GetPOV() == 0)
	{
		Elevator1->Set(ControlMode::Position, 8000);
		Claw->Set(ControlMode::Position, 6700);
	}
	//CARGO rocket low -- xb1 bottom keypad
	if (Operator->GetPOV() == 180)
	{
		Elevator1->Set(ControlMode::Position, 4800);
		Claw->Set(ControlMode::Position, 4700);
	}
	//CARGO player station -- xb1 left keypad
	if (Operator->GetPOV() == 270)
	{
		Elevator1->Set(ControlMode::Position, 3400);
		Claw->Set(ControlMode::Position, 2000);
		//CARGO intake floor --xb1 lb
	}
	//HATCH low rocket/cargoship -- xb1 a
	if (Operator->GetRawButton(XB1::A))
	{
		Claw->Set(ControlMode::Position, 400);
		Wait(.5);
		Elevator1->Set(ControlMode::Position, 1400);
	}
	//HATCH player station -- xb1 b
	if (Operator->GetRawButton(XB1::X))
	{
		Elevator1->Set(ControlMode::Position, 2000);
		Claw->Set(ControlMode::Position, 1400);
	}
	//HATCH intake floor -- xb1 x
	if (Operator->GetRawButton(XB1::B))
	{
		Elevator1->Set(ControlMode::Position, 500);
		Claw->Set(ControlMode::Position, 1000);
	}
	//home
	if ((Operator->GetRawButton(XB1::Menu)) && (Elevator1->GetSelectedSensorPosition(0) < 6000))
	{
		Claw->Set(ControlMode::Position, 0);
		Wait(1);
		Elevator1->Set(ControlMode::Position, 300);
	}
	else if (Operator->GetRawButton(XB1::Menu))
	{
		Claw->Set(ControlMode::Position, 3000);
		Wait(1);
		Elevator1->Set(ControlMode::Position, 2000);
		Wait(1);
		Elevator1->Set(ControlMode::Position, 300);
		Claw->Set(ControlMode::Position, 0);
	}
	//HATCH mid -- xb1 y
	// if (Operator->GetRawButton(XB1::Y)){
	// 	Elevator1->Set(ControlMode::Position, 1400);
	// 	Claw->Set(ControlMode::Position, 600);
	// }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
