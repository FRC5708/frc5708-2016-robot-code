#include "WPILib.h"
#include "Buttons/Button.h"
#include "Buttons/JoystickButton.h"
#include "Buttons/Trigger.h"

float const SPEED_STOP = 0.0;
float const SPEED_MOVE = .25;
int const Node_Path[3] = {1,1,1}; //CHANGE THIS BASED ON FIELD!
float const Obstical_Length = 1;
float const LENGTH_TO_BALL = 1;
float const Auto_Fire_Pitch = 1;
enum{Turn1, Drive1, Turn2, Drive2, Turn3, Drive3, Turn4, Pitch, fire, DO_NOTHING};
enum{Shooting, Driving};
enum class PovState{MIDDLE, UP, DOWN, LEFT, RIGHT};
float const WHEEL_DIAMETER = 16.51*2;
int const COUNTS_PER_REV = 280;	//Number of counters per revolution of encoder
float const ENCODER_SCALE_FACTOR = 3.1416 * WHEEL_DIAMETER / COUNTS_PER_REV; //[cm/cnt]
float const RADIANS_PER_COUNT = 1;


class Robot: public IterativeRobot
{
	float StartLoc[3][2] = {
			{1,1},
			{1,1},
			{1,1}
	};
	float MidLoc[5][2] = {
			{1,1},
			{1,1},
			{1,1},
			{1,1},
			{1,1}
	};
	float EndLoc[3][2] = {
			{1,1},
			{1,1},
			{1,1}
	};
	float FireYaw[3] = {
			1,
			1,
			1
	};
	Joystick stick; // only joystick
	Victor left;
	Victor right;
	Victor ShooterRight;
	Victor ShooterLeft;
	Victor LinAc;
	Victor FireBall;
	Encoder encLeft;
	Encoder encRight;
	Encoder encGun;
	DigitalInput ballSwitch;
	LiveWindow *lw;
	DigitalOutput Light;
	int POV = stick.GetPOV();//d-pad position
	int initValueLeft;
	int initValueRight;
	int autoLoopCounter;
	int autoModeStatus = Drive1;
	int Mode = Driving;
	PovState POVCurrent = PovState::MIDDLE;
	float DRIVE_SPEED = .5;
	int RotCount = 0;
	int ShootCounter = 0;
	bool lighton = false;
	int dirsgn = -1;
	int DEGREE_GROUP;
	bool DriveModeButton = stick.GetRawButton(1);
	bool ToggleLight = stick.GetRawButton(4);
	bool Fire = stick.GetRawButton(6);
	bool Load = stick.GetRawButton(5);

	float Locations[7] = {
			StartLoc[Node_Path[1]][1],
			StartLoc[Node_Path[1]][2],
			MidLoc[Node_Path[2]][1]-(Obstical_Length/2),
			MidLoc[Node_Path[2]][1]+(Obstical_Length/2),
			MidLoc[Node_Path[2]][2],
			EndLoc[Node_Path[3]][1],
			EndLoc[Node_Path[3]][2]
	};

	float StartX = Locations[1];
	float StartY = Locations[2];
	float MidX1 = Locations[3];
	float MidX2 = Locations[4];
	float MidY = Locations[5];
	float EndX = Locations[6];
	float EndY = Locations[7];

	int angsn1;
	int angsn2;
	int angsn3;
	int angsn4;

	double Distances[3] = {
			sqrt((StartX-MidX1)*(StartX-MidX1)+(StartY-MidY)*(StartY-MidY)),
			Obstical_Length,
			sqrt((MidX2-EndX)*(MidX2-EndX)+(MidY-EndY)*(MidY-EndY))
	};


	double Angles[4] = {
			atan((MidY-StartY)/(MidX1-StartX)),
			atan((MidY-StartY)/(MidX1-StartX)),
			atan((EndY-MidY)/(EndX-MidX2)),
			atan((EndY-MidY)/(EndX-MidX2))+FireYaw[Node_Path[3]]
	};

public:
	Robot() :
		// DriveModeButton(stick, 1),
		stick(0),
		left(0),//PWM
		right(1),
		ShooterRight(2),
		ShooterLeft(3),
		LinAc(4),
		FireBall(5),
		encLeft(0,1),//Digital
		encRight(2,3),
		encGun(4,5),
		ballSwitch(6),
		lw(LiveWindow::GetInstance()),
		Light(7)
	{
		//myRobot.SetExpiration(0.1);
	}

private:
	void AutonomousInit()
	{
		autoLoopCounter = 0;
		if (MidY > StartY) //The IF code should work now -L
			angsn1 = 1;
		else
			angsn1 = -1;

		angsn2 = -angsn1;

		if(EndY > MidY)
			angsn3 = 1;
		else
			angsn3 = -1;

		angsn4 = -angsn3;
	}

	void AutonomousPeriodic()
		{
			int currentLeft = encLeft.Get() - initValueLeft;
			int currentRight = encRight.Get() - initValueRight;
			int averageLeftRight = (currentLeft + currentRight) / 2;
			SmartDashboard::PutNumber("Auto Mode Status: ",autoModeStatus);
			SmartDashboard::PutNumber("Right Encoder: ", currentRight);
			SmartDashboard::PutNumber("Left Encoder: ", currentLeft);
			switch(autoModeStatus){
				case Turn1:
					if (RotCount*RADIANS_PER_COUNT < Angles[1]){

						left.Set(.5*angsn1);
						right.Set(.5*angsn1);
					}
					else
					{
						left.Set(SPEED_STOP);
						right.Set(SPEED_STOP);
						autoModeStatus = Drive1;
					}
					RotCount++;
					break;
				case Drive1:
					if(averageLeftRight * ENCODER_SCALE_FACTOR < Distances[1])
					{
						left.Set(-SPEED_MOVE);
						right.Set(SPEED_MOVE);
					}
					else
					{
						left.Set(SPEED_STOP);
						right.Set(SPEED_STOP);
						autoModeStatus = Turn2;
						RotCount = 0;
					}
					break;
				case Turn2:
					if (RotCount*RADIANS_PER_COUNT < Angles[2]){
						left.Set(.5*angsn2);
						right.Set(.5*angsn2);
					}
					else
					{
						left.Set(SPEED_STOP);
						right.Set(SPEED_STOP);
						autoModeStatus = Drive2;
					}
					RotCount++;
					break;
				case Drive2:
					if(averageLeftRight * ENCODER_SCALE_FACTOR < Distances[2])
						{
							left.Set(-SPEED_MOVE);
							right.Set(SPEED_MOVE);
						}
						else
						{
							left.Set(SPEED_STOP);
							right.Set(SPEED_STOP);
							autoModeStatus = Turn3;
							RotCount = 0;
						}
					break;
				case Turn3:
					if (RotCount*RADIANS_PER_COUNT < Angles[3]){
						left.Set(.5*angsn3);
						right.Set(.5*angsn3);
					}
					else
					{
						left.Set(SPEED_STOP);
						right.Set(SPEED_STOP);
						autoModeStatus = Drive3;
					}
					RotCount++;
					break;
				case Drive3:
					if(averageLeftRight * ENCODER_SCALE_FACTOR < Distances[3])
						{
							left.Set(-SPEED_MOVE);
							right.Set(SPEED_MOVE);
						}
						else
						{
							left.Set(SPEED_STOP);
							right.Set(SPEED_STOP);
							autoModeStatus = Turn4;
							RotCount = 0;
						}
					break;
				case Turn4:

					if (RotCount*RADIANS_PER_COUNT < Angles[4]){
						left.Set(.5*angsn4);
						right.Set(.5*angsn4);
					}
					else
					{
						left.Set(SPEED_STOP);
						right.Set(SPEED_STOP);
						autoModeStatus = Pitch;
					}
					RotCount++;
					break;
				case Pitch:
					autoModeStatus = fire;
					break;
				case fire:
					autoModeStatus = DO_NOTHING;
					break;
				case DO_NOTHING:
					break;
			}
			autoLoopCounter++;
		}

	void TeleopInit()
	{
		initValueLeft = encLeft.Get();
		initValueRight = encRight.Get();
		ShootCounter = 5;
	}

	void TeleopPeriodic()
	{
		int currentLeft = encLeft.Get() - initValueLeft;
		int currentRight = encRight.Get() - initValueRight;
		DEGREE_GROUP = POV/90 % 4;
		switch(DEGREE_GROUP){
			case 1:
				POVCurrent = PovState::UP;
				break;
			case 2:
				POVCurrent = PovState::RIGHT;
				break;
			case 3:
				POVCurrent = PovState::DOWN;
				break;
			case 4:
				POVCurrent = PovState::LEFT;
				break;
		}


		if(DriveModeButton and Mode == Driving)
			Mode = Shooting;

		if(DriveModeButton and Mode == Shooting)
			Mode = Driving;

		if(ToggleLight and lighton)
			Light.Set(0);

		if(ToggleLight and not lighton)
			Light.Set(1);


		SmartDashboard::PutNumber("LeftEncoder", currentLeft);
		SmartDashboard::PutNumber("RightEncoder", currentRight);
		DriveControl();
		ShooterControl();
		loaderControl();
		if (Mode == Driving){
			DRIVE_SPEED = 1;
			dirsgn = -1;
		}
		else{
			DRIVE_SPEED = .125;
			dirsgn = 1;
		}
	}

	void TestPeriodic()
	{
		lw->Run();
	}
	void DriveControl()
	{
			float valX = -stick.GetX();
	        float valY = stick.GetY();
	        float val_switch = ballSwitch.Get();
			SmartDashboard::PutNumber("X: ", valX);
			SmartDashboard::PutNumber("Switch: ", val_switch);

			float left_speed = (valY + valX) * DRIVE_SPEED;
			float right_speed = (valY - valX) * DRIVE_SPEED;
			left.Set(-left_speed*dirsgn);
			right.Set(right_speed*dirsgn);
	}

	void loaderControl()
	{
	bool load = Load;
	//bool load = -stick.GetY();

	if(load){
		ShooterLeft.Set(-1.0);
		ShooterRight.Set(1.0);}
	else{
		ShooterLeft.Set(0.0);
		ShooterRight.Set(0.0);}
	}

	void LinAcControl()
	{

	if(POVCurrent == PovState::UP){
		LinAc.Set(1.0);}
	else{
		if(POVCurrent == PovState::DOWN){
			LinAc.Set(-1.0);}
	}};

	void ShooterControl()
	{
				bool Shoot = Fire;

				if(Shoot){
					ShooterLeft.Set(1.0);
					ShooterRight.Set(-1.0);
					if(ShootCounter>1){
						FireBall.Set(1.0);}
					else{ if(ShootCounter<=1 and ShootCounter >-1){
						FireBall.Set(0.0);}
						if(ShootCounter<-1){
							FireBall.Set(-1.0);
						}
					}
					ShootCounter--;}
				else{
					ShooterLeft.Set(0.0);
					ShooterRight.Set(0.0);
					ShootCounter = 5 ;}
	}
};

START_ROBOT_CLASS(Robot)
