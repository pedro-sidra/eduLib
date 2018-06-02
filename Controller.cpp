#include "Controller.h"

Controller::Controller(float Kp, float Ki, float Kd, float ts)
{
	this->_Kp = Kp;
	this->_Ki = Ki;
	this->_Kd = Kd;

  this->_ts = ts;
  
	this->_errorLast = 0;
	this->_iError = 0;

	this->_SP = 0;
}

float Controller::update(float nPV)
{
	_error =_SP - nPV; 
	_dError = (_error - _errorLast)/_ts;
	_iError += (_error + _errorLast)/(2*_ts);

	return( _Kp*_error + _Ki*_iError + _Kd*_dError);

}

void Controller::reset()
{
	 this->_iError=0;
	 this->_errorLast= 0;
}

void Controller::setSP(float nSP)
{
	this->_SP=nSP;
}
