#ifndef CONTROLLER_H

#define CONTROLLER_H
class Controller
{
public:
	Controller(float Kp, float Ki, float Kd,float ts);
	float update(float PV);
	void reset();
	void setSP(float);

private:
	float _error;
	float _iError;
	float _dError;
	float _errorLast;

	float _SP;

	float _ts;
	float _Kp;
	float _Ki;
	float _Kd;
};
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
#endif
