/*
	Biblioteca para uso do Sensor Ultrassonico HC-SR04
	08/05/2018
*/

#ifndef LibSonar_h
#define LibSonar_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class Sonar{
	public:
	Sonar(){}
	Sonar(int _TRIG, int _ECHO); // Construtor da classe
		Sonar(int _TRIG, int _ECHO, long _TOUT);
	long medeTempo();
	long medeDistancia();
	private:
	int Trig_pin;
	int Echo_pin;
	long Time_out;
	long distancia, duracao;
};

#endif
