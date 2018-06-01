#include <LibSonar.h>

#define ECHOF 7  //Echo Sonar da Frente
#define TRIGF 8 //Trig Sonar da Frente
#define ECHOR 12 //Echo Sonar da Direita
#define TRIGR 13 //Trig Sonar da Direita
#define ECHOL 0 //Echo Sonar da Esquerda
#define TRIGL 1 //Trig Sonar da Esquerda

Sonar sonarLeft(TRIGL,ECHOL);
Sonar sonarRight(TRIGR,ECHOR);
Sonar sonarFront(TRIGF,ECHOF);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
      //Send the distance measured by the left ultrassonic sensor
//    Serial.print(" Left: ");
//    Serial.print(sonarLeft.medeDistancia()); 
//    Serial.println(" cm " );
    //Send the distance measured by the Front ultrassonic sensor
    Serial.print(" Front: ");
    Serial.print(sonarRight.medeDistancia()); 
    Serial.print(" cm " );
    //Send the distance measured by the right ultrassonic sensor


    delay(500);

}
