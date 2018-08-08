#ifndef EDUBOT_H
#define EDUBOT_H
#endif

#include "../LibMotor/LibMotor.h"
#include "../Encoder/Encoder.h"
#include "../LibSonar/LibSonar.h"
#include "../Controller/Controller.h"
#include "Pinos.h"


//----------------*** Definições da Biblioteca ***------------------
// *** Parâmetros dos encoders:
// 1496 = pulses per revolution
// 360/1496 = degrees per pulse
#define PPV 1496
#define degPP 0.2406
#define radPP 0.0042

// *** Parâmetros físicos do EduBot:
// Razao entre o raio das rodas e a distância do centro do edubot até as rodas: R/L = 0.1939
#define EDU_R 3.2
#define EDU_L 16.2
#define EDU_RSOBREL 0.1975


// *** Parâmetros dos controladores:
// para o controle de rotação. Caso o erro entre uma iteração e a próxima mude menos que DEL_ERRO, finaliza a rotina
#define DEL_ERRO 0.04


// Tempo de amostragem
#define TS 0.01

// Duas topologias de controle: robo e motores
//#define EDU_CONTROL_ROBO

	// Controladores PID:
	// Parâmetros para o controle PID dos motores direito e esquerdo:
	#define EDU_VMAX 50

	#define KPRIGHT 0.6 
	#define KIRIGHT 5
	#define KDRIGHT 0 

	#define KPLEFT 0.6 
	#define KILEFT 5
	#define KDLEFT 0
	Controller controlRight (KPRIGHT, KIRIGHT, KDRIGHT,TS);
	Controller controlLeft  (KPLEFT,  KILEFT,  KDLEFT, TS);
	Controller controlTheta (9.5, 0, 0.28,TS);



//----------------*** Inicialização de Objetos ***------------------
WheelDrive rodaDir(IN1,IN2,CHAD,CHBD,EDU_R,radPP);
WheelDrive rodaEsq(IN3,IN4,CHAE,CHBE,EDU_R,radPP);


//----------------*** Variáveis Globais ***------------------

bool control_on=false; // Ativa o controle para andar "reto"
float vref=0,wref=0;
int count =0;      // Contador do timer2

//----------------*** Funções ***------------------
/** edu_para();
 * Seta as tensões nos dois motores para zero, e desativa o controle da velocidade linear
 */
void edu_para();
/* edu_moveReto(int Speed);
 * Modifica o setpoint de velocidade de ambos os cotroladores para 'Speed', e ativa o controle
 * da velocidade linear.
 */
void edu_moveReto(double Speed);
/** edu_rotaciona(int degs);
 * Desativa o controle da velocidade linear, e interrompe a execução em um while:
 * -> dentro do while, realiza controle PD da posição angular do robô (a partir de um modelo simples)
 * -> o controle modifica as tensões em cada motor (ou seja, ainda não está implementado o controle de velocidade)
 * -> caso o erro de posicionamento angular entre uma iteração e a próxima seja menor que DEL_ERRO, sai da rotina
 * -> em ausência de erros ou deslizamento dos motores, o robô atinge o ângulo "Angulo"
 * ps: como pode-se notar, se o robô "trava" devido a algum erro mecanico, a rotina encerra antes de atingir o ângulo desejado
 */
void edu_rotaciona(double Angulo);
/**setup_timer2();
 * inicializa o timer2, necessário para fazer o controle a uma taxa de amostragem constante
 */
void setup_timer2();
/** edu_setup()
 *  Inicializa todas as variáveis necessárias para funcionamento do Edubot
 */
void edu_setup();
/** saturate(double in, double lower, double upper)
* Satura "in" entre "lower" e "upper"
* se lower > upper, retorna "in"
*/
double saturate(double in, double lower, double upper);
/**
*
*
*/

/** ISR
 *  Interrupt Service Routine, ativada quando o timer2 estoura. No código, isso ocorre a 8 KHZ
 */
ISR(TIMER2_COMPA_vect);



double getV(double wE, double wD)
{
	return (wE+wD)*EDU_R/2;
}

double getW(double wE, double wD)
{
	return (wE-wD)*EDU_RSOBREL;
}

double computeWd(double v, double w)
{
	return ( v/EDU_R - (0.5*w/EDU_RSOBREL) );
}

double computeWe(double v, double w)
{
	return ( v/EDU_R + (0.5*w/EDU_RSOBREL) );
}
//----------------*** Definições das Funções ***------------------

void edu_paraControlado()
{
	controlRight.setSP(computeWd(0,0));
	controlLeft.setSP(computeWe(0,0)); 
	control_on = true;
}
void edu_para()
{
  edu_paraControlado();
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

void edu_moveReto(double v)
{
	controlRight.setSP(computeWd(v,0));
	controlLeft.setSP(computeWe(v,0)); 
	control_on = true;
}

void edu_moveVW(double v, double w)
{
	controlRight.setSP(computeWd(v,w));
	controlLeft.setSP(computeWe(v,w)); 
	control_on = true;
}
void edu_rotaciona(double degs)
{
	edu_para();delay(300);
	 controlTheta.setSP(degs*3.14/180); 
	 double wedu=0, theta=0;
	 control_on = false;
	 char ccount=0;
	 while(ccount < 10)
	 {
			 theta+=TS*getW(rodaEsq.getW(),rodaDir.getW());
			 double V = controlTheta.update(theta);
			 rodaDir.setVoltage(-V);
			 rodaEsq.setVoltage(V);
			 if(controlTheta.getError() < DEL_ERRO)
					 ccount++;
			 else
					 ccount=0;
			 delay(10);
	 }
	 edu_para();delay(300);
}



void setup_timer2()
{
  control_on=false;
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
  rodaEsq.init(maxMvolt, maxBvolt);
  rodaDir.init(maxMvolt, maxBvolt);
  setup_timer2();
}

void le_velocidades_motores()
{
		rodaEsq.update(TS);
		rodaDir.update(TS);
}


void edu_update()
{
	le_velocidades_motores();
	if(control_on)
		rodaEsq.setVoltage(controlLeft.update(rodaEsq.getW()));
		rodaDir.setVoltage(controlRight.update(rodaDir.getW()));
}


ISR(TIMER2_COMPA_vect){//timer2 interrupt 8kHz
    count++;
    if(count>80)// conta ate 80 a 8KHz -> amostragem a 100Hz
	{
		count = 0;
		edu_update();
  	}
}


