/*
	Biblioteca para uso do Sensor Ultrassonico HC-SR04
	08/05/2018

	Tempo de time out deve ser especificado em microssegundos
*/
#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "LibSonar.h"

Sonar::Sonar(int _TRIG, int _ECHO){
	pinMode(_TRIG,OUTPUT);
	pinMode(_ECHO,INPUT);
	Trig_pin=_TRIG;
	Echo_pin=_ECHO;
	Time_out=30000;	//3000us -> 50cm //30000us -> 5m
}

Sonar::Sonar(int _TRIG, int _ECHO, long _TOUT){
	pinMode(_TRIG,OUTPUT);
	pinMode(_ECHO,INPUT);
	Trig_pin = _TRIG;
	Echo_pin = _ECHO;
	Time_out = _TOUT;
}

long Sonar::medeTempo(){
	digitalWrite(Trig_pin, LOW);
  	delayMicroseconds(2);
  	digitalWrite(Trig_pin, HIGH);
  	delayMicroseconds(10);
  	digitalWrite(Trig_pin, LOW);
  	duracao = pulseIn(Echo_pin,HIGH,Time_out);
  	if ( duracao == 0 ) {
		duracao = Time_out; }
  	return duracao;
}	

long Sonar::medeDistancia(){
	medeTempo();
	distancia = duracao*0.034/2 ;
 	return distancia;
	
}
