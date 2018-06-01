/* 
/*
 * LibMotor
 * Library for L298N dual H-Bridge
 *
 */

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "LibMotor.h"

Mdrive::Mdrive(int out1, int out2){	
	pinMode(out1, OUTPUT);
	pinMode(out2, OUTPUT);
	
	this->_out1 = out1;
	this->_out2 = out2;

}

void Mdrive::init(float maxMvolt, float maxBvolt){
	this->_bVolt = maxBvolt;
	this->_mVolt = maxMvolt;
	stop();
}

void Mdrive::setVoltage(float refVolt){
	
	if(refVolt <= _mVolt && refVolt >= -_mVolt){
		_duty = 255*(_bVolt + refVolt)/(_bVolt*2);
	}
	else if(refVolt > _mVolt){
		_duty = 255*((_bVolt +_mVolt)/_bVolt*2);
	}
	else if(refVolt < -_mVolt){
		_duty = ((_bVolt-_mVolt)/_bVolt*2);
	}
	
	this->_refVolt = refVolt;
	
	setPWM(_duty);
}

void Mdrive::setPWM(uint8_t dutycycle){
	
	analogWrite(_out1, dutycycle);
	analogWrite(_out2, 255-dutycycle);
}

void Mdrive::stop(){
	setVoltage(0);
}

void Mdrive::enable(int enA, bool value){
	this->_enA = enA;
	
	pinMode(_enA, OUTPUT);
	digitalWrite(_enA, value);

}

