/* 
 * LibMotor
 * Library for L298N dual H-Bridge
 *
 */

#ifndef LibMotor_h

#define LibMotor_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//Class for L298N Dual H-Bridge
class Mdrive{
  public:
	Mdrive(){}
	Mdrive(int out1, int out2);
	void init(float maxMvolt, float maxBvolt);
	void setPWM(uint8_t dutycycle);
	void stop();
	void setVoltage(float refVolt);
	virtual void enable(int enA, bool value);
	
  private:
	int _out1;	
	int _out2;    
	int _enA;
	float _bVolt;
	float _mVolt;
	float _refVolt;
	int _duty;
    
};

#endif
