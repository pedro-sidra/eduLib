/**
 * Código de demonstração do arquivo Edubot.h
 * 
 * Para controle do Edubot, foi implementado controle de velocidade PID em ambos os motores. 
 *    Cada motor é levemente diferente um do outro - para uma mesma tensão aplicada no motor,
 *    o motor pode adquirir maior velocidade ou demorar mais para acelerar.
 *    Por isso, para que o robô ande em linha reta, deve ser feito um controlador de velocidade.
 *    Além disso, para que o robô rotacione exatos 90 graus, por exemplo, é necessário um controle de posição.
 *    (Na UFRGS, aprendemos a teoria necessária para calcular os parâmetros na disciplina Controle I)
 *
 * As funções edu_moveReto(int velocidade), edu_para() e edu_rotaciona(int angulo)
 * fazem a movimentação do robô.
 * Perceba que embora haja controle de velocidade, ainda pode ocorer "drift" devido a erros numericos, entao use os
 * fins de curso para ajuste! 
 * 
 * A rotina de exemplo mostra usos comuns dessas três funções. Velocidades negativas e ângulos negativos são aceitos.
 * 
 * Não é necessário alterar o código fonte Edubot.h!!
 */
#include <Edubot.h>

//     *** Variáveis globais: ***


// Sonares utilizados pelo robô. O método medeDistancia() adquire a distancia
// Exemplo: sonarRight.medeDistancia() retorna a distancia na direita
Sonar sonarRight(TRIGR, ECHOR);
Sonar sonarFront(TRIGF, ECHOF);
Sonar sonarLeft(TRIGL, ECHOL);

// Distancias em cada direção. 
// (as distâncias são armazenadas em variaveis para nao chamar
// a função medeDistancia() repetidamente)
long distR, distF,distL;


//     *** Protótipos de função:  ***

//Adquire as distancias de cada sonar nas variaveis distR, distF, distL
void medeDistancias();
// Envia os dados de distancia para a serial (usar se for desejado)
void mandaDistanciasSerial();

//     *** Rotinas principais: ***

void setup() {
  
  Serial.begin(9600);

  // Inicializa os motores, os encoders, e os controladores,
  // assim como o timer necessário.
  // Sempre chamar edu_setup() no setup!
  edu_setup();
  
}


void loop() {
   // A função edu_rotaciona desta versão aceita 1, 2 ou 3 parâmetros de entrada
   // Primeiro argumento: ângulo em graus
   edu_rotaciona(180);
   
   // Segundo argumento: velocidade de rotação em rad/s
   edu_rotaciona(180,EDU_WMAX);
   
   // terceiro argumento: distância do centro do robô até o centro do círculo de rotação (em cm)
   // por padrão esse argumento é zero, ou seja, o robô rotaciona em torno de seu centro
   // se o argumento for EDU_L, o robô rotaciona em torno da sua borda (aproximadamente...)
    edu_rotaciona(180,EDU_WMAX*2/5,EDU_L);
}

// *** Definições das funções: ***
void medeDistancias()
{
  distR = sonarRight.medeDistancia();
  distL = sonarLeft.medeDistancia();
  distF = sonarFront.medeDistancia();
}

void mandaDistanciasSerial()
{
  Serial.print("Distancia Direita: ");Serial.println(distR);
  Serial.print("Distancia Esquerda: ");Serial.println(distL);
  Serial.print("Distancia Frente: ");Serial.println(distF);
}
