#ifndef EDUBOT_H
#define EDUBOT_H
#endif

#include "LibMotor.h"
#include "Encoder.h"
#include "LibSonar.h"
#include "Controller.h"
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
#define DEL_ERRO 0.005


// Tempo de amostragem
#define TS 0.01

// Duas topologias de controle: robo e motores
#define EDU_CONTROL_ROBO
//#define EDU_CONTROL_MOTORES
#ifdef EDU_CONTROL_ROBO
// SetPoint máximo de velocidade linear
	#define EDU_VMAX 45
	#define EDU_WMAX 5.5

	#define KPV 0.225
	#define KIV 2.25
	#define KDV 0

	#define KPW 2
	#define KIW 20
	#define KDW 0
	
	#define KPT 3 
	#define KIT 9
	#define KDT 0.1


	Controller controlV (KPV, KIV, KDV,TS);
	Controller controlW (KPW,  KIW,  KDW, TS);
	Controller ControlTheta(KPT,KIT,KDT,TS);
#endif
#ifdef EDU_CONTROL_MOTORES
	// Controladores PID:
	// Parâmetros para o controle PID dos motores direito e esquerdo:
	#define EDU_VMAX 500

	#define KPRIGHT 0.0102
	#define KIRIGHT 0.06
	#define KDRIGHT 0

	#define KPLEFT 0.0185
	#define KILEFT 0.0923
	#define KDLEFT 0
	Controller controlRight (KPRIGHT, KIRIGHT, KDRIGHT,TS);
	Controller controlLeft  (KPLEFT,  KILEFT,  KDLEFT, TS);

#endif

//----------------*** Inicialização de Objetos ***------------------

// Encoders:
Encoder knobLeft(CHAE, CHBE);
Encoder knobRight(CHAD, CHBD);

// Motores:
Mdrive mDireita(IN1, IN2);
Mdrive mEsquerda(IN3, IN4);


//----------------*** Variáveis Globais ***------------------

bool control_on; // Ativa o controle para andar "reto"
bool theta_on=false;
float wD,wE;        // Velocidades angulares de cada motor
double theta =0;
float Vm,Vdiff;	// Tensao media e tensao diferencial
long knobLeftLast, knobRightLast, knobLeftN,knobRightN;// Valores de contagem dos encoders
char count =0;      // Contador do timer2

//----------------*** Funções ***------------------
/** edu_para();
 * Seta as tensões nos dois motores para zero, e desativa o controle da velocidade linear
 */
void edu_para();
/* edu_moveReto(int Speed);
 * Modifica o setpoint de velocidade de ambos os cotroladores para 'Speed', e ativa o controle
 * da velocidade linear.
 */
void edu_moveReto(int Speed);
/** edu_rotaciona(int degs);
 * Desativa o controle da velocidade linear, e interrompe a execução em um while:
 * -> dentro do while, realiza controle PD da posição angular do robô (a partir de um modelo simples)
 * -> o controle modifica as tensões em cada motor (ou seja, ainda não está implementado o controle de velocidade)
 * -> caso o erro de posicionamento angular entre uma iteração e a próxima seja menor que DEL_ERRO, sai da rotina
 * -> em ausência de erros ou deslizamento dos motores, o robô atinge o ângulo "Angulo"
 * ps: como pode-se notar, se o robô "trava" devido a algum erro mecanico, a rotina encerra antes de atingir o ângulo desejado
 */
void edu_rotaciona(double Angulo,double wRot);
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




//----------------*** Definições das Funções ***------------------
void edu_para()
{
  mEsquerda.setVoltage(0);  
  mDireita.setVoltage(0);
  controlW.reset();controlV.reset(); 
  control_on = false;
}

void edu_paraControlado()
{
  controlV.setSP(0);controlW.setSP(0); 
  control_on = true;
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

void edu_moveReto(int Speed)
{
  controlV.setSP(Speed);controlW.setSP(0); 
  control_on = true;
}

void edu_rotaciona(double degs,double wRot=3)
{
  double degsRad = degs*0.0174533;
  double erro[3]={0,0,0};
  double a,b;
  char ccount=0;
  if(degsRad<0)
      wRot = -wRot;
  edu_paraControlado();
  delay(400);
  ControlTheta.reset();ControlTheta.setSP(degsRad);
  control_on =true;theta_on=true;theta=0;
  erro[0] = theta-degsRad;
  do 
  { // Controle de rotação
    delay(10);
    erro[2]=erro[1];
    erro[1]=erro[0];
    erro[0] = theta-degsRad;
    a = erro[0] -1.471*erro[1] + 0.5446*erro[2];
    b = erro[0] + 2*erro[1] + erro[2];
    controlW.setSP(50*a/b);
  } while (true);
  theta=0;theta_on=false;
  edu_para();
  delay(400);
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
  mDireita.init(maxMvolt, maxBvolt);
  mEsquerda.init(maxMvolt, maxBvolt);
  setup_timer2();
  pinMode(FCFE,INPUT);
  pinMode(FCFD,INPUT);
  pinMode(FCTE,INPUT);
  pinMode(FCTD,INPUT);
}

void le_velocidades_motores()
{
      knobLeftN = knobLeft.read();
      knobRightN= knobRight.read();
      wD = radPP*(knobRightN-knobRightLast)/TS;
      wE = radPP*(knobLeftN-knobLeftLast)/TS;
      knobLeftLast = knobLeftN;
      knobRightLast = knobRightN;
}

ISR(TIMER2_COMPA_vect){//timer2 interrupt 8kHz
  
    count++;
    if(count>80)// conta ate 80 a 8KHz -> amostragem a 100Hz
    {
      count = 0;
      le_velocidades_motores();
      if(control_on)
      {
	// Tensão média entre os motores e tensão diferencial entre os motores
	// são dadas pelos controladores de velocidade linear e angular, respectivamente
	Vm = controlV.update((wE+wD)*EDU_R/2);
	Vdiff = controlW.update((wE-wD)*EDU_RSOBREL);

	// Tensões nos motores direito e esquerdo
	double Ve = Vm+Vdiff;
	double Vd = Vm-Vdiff;
	if(Ve>0)
		Ve +=1;
	else if(Ve<0)
		Ve-=1;
	if(Vd>0)
		Vd +=1;
	else if(Vd<0)
		Vd-=1;
	// Liga o integrador apenas quando não há saturação dos motores:
	// (Implementação de anti-windup)
	bool noSat = abs(Ve)<maxMvolt && abs(Vd) < maxMvolt;
	controlV.setIntegrator(noSat);
	controlW.setIntegrator(noSat);
	ControlTheta.setIntegrator(noSat);
	
	// Satura as tensões nos motores e seta
	mEsquerda.setVoltage(saturate(Ve,-maxMvolt,maxMvolt)); 
        mDireita.setVoltage(saturate(Vd,-maxMvolt,maxMvolt));
      }
      if(theta_on)
      {
	       theta+=(wE-wD)*EDU_RSOBREL*TS;
      }
    
  }
}


