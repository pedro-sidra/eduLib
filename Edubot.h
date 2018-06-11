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

// *** Parâmetros físicos do EduBot:
// Razao entre o raio das rodas e a distância do centro do edubot até as rodas: R/L = 0.1939
#define RSOBREL 0.1939


// *** Parâmetros dos controladores:
// para o controle de rotação. Caso o erro entre uma iteração e a próxima mude menos que DEL_ERRO, finaliza a rotina
#define DEL_ERRO 1

// Parâmetros para o controle PID dos motores direito e esquerdo:
#define KPRIGHT 0.0102
#define KIRIGHT 0.06
#define KDRIGHT 0

#define KPLEFT 0.0185
#define KILEFT 0.0923
#define KDLEFT 0
// Também funcionam (automatic tuning do matlab):
//#define KPRIGHT 0.13522
//#define KIRIGHT 0.007
//#define KPLEFT 0.13522
//#define KILEFT 0.007

// Tempo de amostragem
#define TS 0.01
// SetPoint máximo de velocidade linear
#define EDU_VMAX 500



//----------------*** Inicialização de Objetos ***------------------

// Encoders:
Encoder knobLeft(CHAE, CHBE);
Encoder knobRight(CHAD, CHBD);

// Motores:
Mdrive mDireita(IN1, IN2);
Mdrive mEsquerda(IN3, IN4);

// Controladores PID:
Controller controlRight (KPRIGHT, KIRIGHT, KDRIGHT,TS);
Controller controlLeft  (KPLEFT,  KILEFT,  KDLEFT, TS);

//----------------*** Variáveis Globais ***------------------

bool control_frente; // Ativa o controle para andar "reto"
float wD,wE;        // Velocidades angulares de cada motor
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
void edu_rotaciona(int Angulo);
/**setup_timer2();
 * inicializa o timer2, necessário para fazer o controle a uma taxa de amostragem constante
 */
void setup_timer2();
/** edu_setup()
 *  Inicializa todas as variáveis necessárias para funcionamento do Edubot
 */
void edu_setup();

/** ISR
 *  Interrupt Service Routine, ativada quando o timer2 estoura. No código, isso ocorre a 8 KHZ
 */
ISR(TIMER2_COMPA_vect);




//----------------*** Definições das Funções ***------------------
void edu_para()
{
  mEsquerda.setVoltage(0);  
  mDireita.setVoltage(0);
  controlRight.reset();controlLeft.reset(); 
  control_frente = false;
}

void edu_moveReto(int Speed)
{
  controlRight.setSP(Speed);controlLeft.setSP(Speed); 
  control_frente = true;
}

void edu_rotaciona(int degs)
{
  control_frente = false;
  long newLeft=0, newRight=0,Vnew=0;
  double erro=0, errolast=0, derro=0;
  char ccount=0;
  knobLeft.write(0); knobRight.write(0);
  edu_para();
  delay(300);
  do { //Giro para Direita 90 graus
  newLeft = knobLeft.read();  newRight = knobRight.read();
  errolast = erro;
  erro = (double)degs-((newLeft - newRight)*degPP)*RSOBREL; 
  derro = erro-errolast;
  Vnew = 0.5*(erro + 0.6*derro);
  mEsquerda.setVoltage(Vnew); 
  mDireita.setVoltage(-Vnew);   
  if(abs(erro-errolast)< DEL_ERRO)
    ccount++;
  else
    ccount=0;
  delay(30);
  } while (ccount <= 10);
  edu_para();
  delay(300);
}



void setup_timer2()
{
  control_frente=false;
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

ISR(TIMER2_COMPA_vect){//timer2 interrupt 8kHz
  
    count++;
    if(count>80)// conta ate 80 a 8KHz -> amostragem a 100Hz
    {
      count = 0;
      knobLeftN = knobLeft.read();
      knobRightN= knobRight.read();
      wD = degPP*(knobRightN-knobRightLast)/TS;
      wE = degPP*(knobLeftN-knobLeftLast)/TS;
      if(control_frente)
      {
        mEsquerda.setVoltage(controlLeft.update(wE)); 
        mDireita.setVoltage(controlRight.update(wD));
      }
      knobLeftLast = knobLeftN;
      knobRightLast = knobRightN;
    
  }
}


