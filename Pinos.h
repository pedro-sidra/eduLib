#ifndef Pinos_h
#define Pinos_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/*
	Edubot Portas Abertas 2018
		Pinos
	
*/

/*
	Pinos de Fim de Curso
*/
#define FCFE A0	// Fim de curso frente esquerda
#define FCFD A1	// Fim de curso frente direita
#define FCTE A2	// Fim de curso atras esquerda
#define FCTD A3	// Fim de curso atras direita
/*
	Pinos dos Sonares
*/
#define ECHOF 7  //Echo Sonar da Frente
#define TRIGF 8 //Trig Sonar da Frente
#define ECHOR 12 //Echo Sonar da Direita
#define TRIGR 13 //Trig Sonar da Direita
#define ECHOL A5 //Echo Sonar da Esquerda
#define TRIGL A4 //Trig Sonar da Esquerda
/*
	Pinos da Ponte H
	Mover para frente: mEsquerda.setVoltage(-V) mDireita.setVoltage(V)
	Girar para esquerda: mEsquerda.setVoltage(V) mDireita.setVoltage(V)
	Motor Esquerda - A	Motor Direita - B
*/
#define IN1 5 
#define IN2 6
#define IN3 10
#define IN4 9
/*
	Pinos dos Encoders
	Mover para Frente: encoderEsquerda - positivo encoderDireita - negativo
	Girar para Esquerda: encoderEsquerda - negativo encoderDireita - negativo
*/
#define CHAD 2
#define CHBD 11
#define CHAE 3
#define CHBE 4
/*
	Tensões dos Motores e Bateria
*/
#define maxMvolt 6
#define maxBvolt 6


#endif
