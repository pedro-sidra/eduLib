# eduLib: biblioteca para o EduBOT

## Geral

Essa biblioteca implementa controle PID de velocidade em ambos os motores, e fornece
funções para controle de movimentação do EduBOT. Além do controle
PID de velocidadde , há um controle de rotação para rotacionar o robô em um dado
ângulo. Os controladores foram calculados para o edubot marcado como "1", ou seja,
são otimizados para este.

Também é incluída a biblioteca LibSonar, que fornece uma interface simples com os sonares.


## Instalação

Para instalar, basta baixar esse repositório (botao Clone or Download) e extrair o arquivo .zip para a sua pasta Arduino/libraries ou sketchbook/libraries.

## Começando a Utilizar

Para utilizar, recomenda-se abrir o exemplo edubotExemplo (imagem ilustrando como abrir: https://imgur.com/a/kOuJwNj). Esse arquivo mostra os funcionamentos principais oferecidos por essa biblioteca, e pode ser usado como modelo.

O arquivo Pinos.h define o pinout do EduBot. As variáveis definidas nele podem ser usadas no programa. Por exemplo, a leitura do fim de curso da frente e da direita pode ser feita com "digitalRead(FCFD)".
