#ifndef EDUBOT_H
#define EDUBOT_H
#endif

// Dependencies (should be in arduino's library directory)
#include "../LibMotor/LibMotor.h"
#include "../Encoder/Encoder.h"
#include "../LibSonar/LibSonar.h"
#include "../Controller/Controller.h"

// Pinout for your robot
//  The default Pinout.h provided defines EduBot's default connections
#include "Pinout.h"


//----------------*** Library Definitions ***------------------

// *** Encoder params:

// 1496 = pulses per revolution
// 360/1496 = degrees per pulse
// 2pi/1496 = radians per pulse
#define PPR 1496
#define degPP 0.2406
#define radPP 0.0042

// *** Mechanical params:

// R = Wheel radius,
// L = Distance between wheels
// EDU_RL = R/L should be calculated and put here literally to save computation time
#define EDU_R 3.2
#define EDU_L 16.2
#define EDU_RL 0.1975


// *** Controller params:

// Minimum delta error for rotational control (in radians)
// If error varies less than this between iterations, consider 
// that the robot has finished its rotation
#define DEL_ERROR 0.11
// Maximum linear velocity
#define EDU_VMAX 50

// Sample time
#define TS 0.01

// PID Controller gains for right and left motors:
#define KPRIGHT 0.6 
#define KIRIGHT 5
#define KDRIGHT 0 

#define KPLEFT 0.6 
#define KILEFT 5
#define KDLEFT 0

// Controller gains for rotational control:
#define KPTHETA 9.5
#define KITHETA 0.01
#define KDTHETA 0.28 

// Controller objects:
Controller controlRight (KPRIGHT, KIRIGHT, KDRIGHT,TS);
Controller controlLeft  (KPLEFT,  KILEFT,  KDLEFT, TS);
Controller controlTheta (KPTHETA, KITHETA, KDTHETA,TS);

//----------------*** Object Initialization ***------------------
// WheelDrives for each wheel (two digital pins for H-Bridge, two for encoder)
WheelDrive wheelRight(RIGHT_BRIDGE_A, 
					  RIGHT_BRIDGE_B,
					  RIGHT_ENCODER_A,
					  RIGHT_ENCODER_B,
					  EDU_R,radPP);
WheelDrive wheelLeft(LEFT_BRIDGE_A, 
					  LEFT_BRIDGE_B,
					  LEFT_ENCODER_A,
					  LEFT_ENCODER_B,
					  EDU_R,radPP);

//----------------*** Global Variables ***------------------

// Turns controllers on or off 
// TODO: Fix this implementation (turn off the controller objects and fix the update method
// , don't use this bool...)
bool control_on=false; 

// Counts to 80 within timer2 ISR to update at 100 Hz
// TODO: get timer2 to generate ISR at 1/TS (100 Hz) without breaking everything else
int count =0;      // Contador do timer2

//----------------*** Function Prototypes ***------------------

// *** "Top-Level": these functions must be executed for everything else to work.
void edu_update(); // Run at sample rate (ISR)
void edu_setup();  // Run once at startup

// *** "Library-Level": used inside this .h file to abstract implementations
void update_control();
void readMotorData();
void setup_timer2();
void update_setPoint(double v, double w);

// *** Movement Functions ("user-level"):
void edu_stop();              // Stops the motors (open-loop)
void edu_controlledStop();    // Stops the robot (closed-loop)
void edu_rotate(double degs); // Rotates the robot by degs degrees
void edu_moveLine(double v);  // Moves the robot in a straight line
void edu_moveVW(double v, double w); // Moves the robot with specified linear and angular speed

// *** Utility functions 

// Saturation between lower and upper
double saturate(double in, double lower, double upper);

// Speed conversion  (v = linear velocity, w = angular velocity)
// robot (v,w) -> motor w (left, right)
double computeWl(double v, double w);
double computeWr(double v, double w);
// motor w (left, right) -> robot (v,w) 
double getW(double Wl, double Wr);
double getV(double Wl, double Wr);


//----------------*** Function Definitions  ***------------------


double getV(double Wl, double Wr)
{
	return (Wl+Wr)*EDU_R/2;
}

double getW(double Wl, double Wr)
{
	return (Wl-Wr)*EDU_RL;
}

double computeWr(double v, double w)
{
	return ( v/EDU_R - (0.5*w/EDU_RL) );
}

double computeWl(double v, double w)
{
	return ( v/EDU_R + (0.5*w/EDU_RL) );
}

void edu_controlledStop()
{
	edu_moveVW(0,0);
	control_on = true;
}
void edu_stop()
{
	control_on=false;
	wheelLeft.setVoltage(0);
	wheelRight.setVoltage(0);
}

double saturate(double in, double lower, double upper)
{
	if(lower>upper)
		return in;
	if(in>upper)
		return upper;
	else if(in < lower)
		return lower;
	return in;	
}

void update_setPoint(double v, double w)
{
	controlRight.setSP(computeWr(v,w));
	controlLeft.setSP(computeWl(v,w)); 
}
void edu_moveVW(double v, double w)
{
	update_setPoint(v,w);
	control_on = true;
}

void edu_moveLine(double v)
{
	edu_moveVW(v,0);
	control_on = true;
}

void edu_rotate(double degs)
{
	double theta=0;
	char ccount=0;
	
	edu_stop();delay(300);

	controlTheta.setSP(degs*3.14/180.0); 

	control_on = false; // TODO: fix this confusing declaration
						// control_on=false does not turn off controlTheta,
						// only whatever is inside update_control()
	while(ccount < 10)
	{
		// getW is used to convert delta theta from the wheels into delta
		// theta for the robot
		theta+=getW(wheelLeft.getDeltaTheta(),wheelRight.getDeltaTheta());

		// TODO: implement a better controlTheta.
		// This one is modelled considering differential voltage between
		// the motors, but it can generate residual linear velocity
		double V = controlTheta.update(theta);
		wheelRight.setVoltage(-V);
		wheelLeft.setVoltage(V);

		// Stops this loop whenever the error varies less than DEL_ERROR for a while
		if(fabs(controlTheta.getError()) < DEL_ERROR)
			ccount++;
		else
			ccount=0;
		// Delay has to be here to avoid competition between this loop and the ISR
		delay(10);
	 }
	 edu_stop();delay(300);
}


void setup_timer2()
{
  	control_on=false;
  	// This is stolen from https://www.instructables.com/id/Arduino-Timer-Interrupts/
  	// TODO:Implement a better config for timer2 ... sorry!

  	TCCR2A = 0;// set entire TCCR2A register to 0
  	TCCR2B = 0;// same for TCCR2B
  	TCNT2  = 0;//initialize counter value to 0
  	// set compare match register for 8khz increments
  	OCR2A = 249; //8kHz = (16*10^6) / (8000*8) - 1 (must be <256)
  	// turn on CTC mode
  	TCCR2A |= (1 << WGM21);
  	// Set CS21 bit for 8 prescaler
  	TCCR2B |= (1 << CS21);   
  	// enable timer compare interrupt
  	TIMSK2 |= (1 << OCIE2A);
}

void edu_setup()
{
  	wheelLeft.init(maxMvolt, maxBvolt);
  	wheelRight.init(maxMvolt, maxBvolt);
  	setup_timer2();
}

void readMotorData()
{
	wheelLeft.update(TS);
	wheelRight.update(TS);
}


void update_control()
{
	// TODO: long stack, should be a better way to do this without more variables
	// Sets the voltage of each motor
	// to the value calculated by the controller
	// considering the most recent angular speed of the motor
	wheelLeft.setVoltage(controlLeft.update(wheelLeft.getW()));
	wheelRight.setVoltage(controlRight.update(wheelRight.getW()));
}

void edu_update()
{
	// Anything inside this function runs at 100 Hz regardless 
	// Do NOT put delays or loops here! 
	// This should execute quickly

	readMotorData(); // Updates angular speed of each motor
	if(control_on) // If the controllers are on, update the motor voltages
	{
		update_control();
	}
}


ISR(TIMER2_COMPA_vect){//timer2 interrupt @ 8kHz
	// counts to 80 to reduce the frequency.
	// I know, sorry...
	// TODO: run the ISR at 100 Hz to save time
    count++;
    if(count>80)
	{
		count = 0;
		edu_update();
  	}
}


