/*
FIRMWARE DE TESTES DO HARDWARE PROJETO ROBÔ SUMÔ - PI1

AUTOR: PAULO CALEB FERNANDES DA SILVA

MICROCONTROLADOR UTILIZADO: RP2040 (Mbed as Raspberry pi pico)
*/
//-----------------------MAPEAMENTO DE HARDWARE------------------------------------
const uint8_t LED_RBP_PICO= 22, //Pino do led interno da placa do raspberry pi pico (recomendado para debug)
              VOLTAGE_PIN = A0, //Pino analógico dedicado a medição de tensão
              SENS_PIN    = A1, //Pino analógico dedicado a medição dos sensores LDR
              CURRENT_PIN = A2, //Pino analógico dedicado a medição de corrente
              LIGHT_CTRL  = 1,  //Pino para controle da intensidade dos leds excitadores dos sensores LDRs
              MUX_BIT1    = 2,  //Pino do bit 1 do multiplexador
              MUX_BIT0    = 3,  //Pino do bit 0 do multiplexador
              ECHO        = 4,  //Pino do echo do sensor de distância
              TRIG        = 5,  //Pino de trigger do sensor de distância
              MOTOR1      = 6,  //PWM para o motor 1
              IN1         = 7,  //Sentido A do motor 1
              IN2         = 8,  //Sentido B do motor 1
              IN3         = 9,  //Sentido A do motor 2
              IN4         = 10, //Sentido B do motor 2
              MOTOR2      = 11, //PWM para o motor 2
              GREEN       = 12, //Pino de acionamento do led verde (RGB)
              BLUE        = 13, //Pino de acionamento do led azul (RGB)
              RED         = 14, //Pino de acionamento do led vermelho (RGB)
              BUZZER      = 15, //Pino de acionamento do buzzer
              BOT         = 22; //Pino do botão
      //Pinos 16, 17, 18 e 19 estão sendo usados para a comunicação SPI com o cartão SD
      const int _MISO = 16, //Master Input Slave Output
                _MOSI = 19, //Master Output Slave Input
                _CS = 17,   //Chip Select
                _SCK = 18;  //Slave Clock
//Vetores de inicialização
const uint8_t inputs  [5] {SENS_PIN, CURRENT_PIN, VOLTAGE_PIN, BOT, ECHO}; //Vetor de entradas
const uint8_t outputs [14] {MUX_BIT0, MUX_BIT1, TRIG, MOTOR1, IN1, IN2, IN3, IN4, MOTOR2, LIGHT_CTRL, RED, GREEN, BLUE, BUZZER}; //Vetor de saídas

//--------------------FIM DO MAPEAMENTO DE HARDWARE--------------------------------
const float VelocidadeSom_mpors = 340; // em metros por segundo
const float VelocidadeSom_mporus = 0.000340; // em metros por microsegundo

//----------------------INCLUSÃO DE BIBLIOTECAS -----------------------------------
#include <SPI.h>//Biblioteca para comunicação SPI com o cartão SD
#include <SD.h>//Biblioteca para o gerenciamento do cartão SD
File myFile;//Objeto myFile para o cartão SD

//-----------------------VARIÁVEIS DE PROGRAMA-------------------------------------
//Variáveis de programa
float dt = 0;//Armazena o tempo de resposta do sensor de distância
int PWM  = 0;//Armazena o PWM para os motores
//Vetor dos sensores de borda
float edgeSens1 = 0;
float edgeSens2 = 0;
float edgeSens3 = 0;
float edgeSens4 = 0;
float EDGE_SENSORS[4]{edgeSens1, edgeSens2, edgeSens3, edgeSens4};
//-----------------------PROTÓTIPO DAS FUNÇÕES-------------------------------------
void TRIGGER();
float distancia(float tempo_us); //Calcula a distância a frente do HC_SR04

//-----------------------SETUP DO PRIMEIRO NÚCLEO----------------------------------
void setup() {
  //Configura as entradas e saídas
  for(uint8_t i = 0; i < 5; i++) pinMode(inputs[i], INPUT); //Inicializa as entradas
  for(uint8_t k = 0; k < 14; k++) pinMode(outputs[k], OUTPUT); //Inicializa as saídas
  for(uint8_t k = 0; k < 14; k++) digitalWrite(outputs[k], LOW); //Desativa todas as saídas
  //Teste da iluminação dos LDRs
  analogWrite(LIGHT_CTRL, 10);
  delay(250);
  analogWrite(LIGHT_CTRL, 80);
  delay(250);
  analogWrite(LIGHT_CTRL, 130);
  delay(250);
  analogWrite(LIGHT_CTRL, 255);
  //TESTE DO BUZZER
  analogWriteFreq(2000); // Define a frequência para 2000 Hz (2 kHz)
  analogWrite(BUZZER, 128); // Ativa o buzzer com 50% do ciclo de trabalho
  delay(200);
  analogWriteFreq(3000); // Define a frequência para 3000 Hz (3 kHz)
  analogWrite(BUZZER, 128); // Ativa o buzzer com 50% do ciclo de trabalho
  delay(200);
  analogWriteFreq(2500); // Define a frequência para 2500 Hz (2.5 kHz)
  analogWrite(BUZZER, 128); // Ativa o buzzer com 50% do ciclo de trabalho
  delay(200);
  analogWriteFreq(2000); // Define a frequência para 2000 Hz (2 kHz)
  analogWrite(BUZZER, 128); // Ativa o buzzer com 50% do ciclo de trabalho
  delay(400);
  analogWrite(BUZZER, 0);//desativa o buzzer
  
//Fim do setup do primeiro núcleo
}
//-----------------------LOOP DO PRIMEIRO NÚCLEO-----------------------------------
void loop() {
  bool objetoProximo = false; // Variável para verificar se encontrou objeto a menos de 25 cm

  pararRobo(); // Para o robô
  delay(1000); // Pequena pausa
  trocaEstado();
  girarRobo(); // Inicia o movimento de giro

  for (uint8_t i = 0; i < 25; i++) {
    TRIGGER(); // Envia pulso para o sensor HC_SR04
    dt = pulseIn(ECHO, HIGH); // Mede o tempo de retorno do echo
    float distanciaCm = distancia(dt) * 100; // Calcula a distância em cm
    Serial.print("Distancia em centimetros: ");
    Serial.println(distanciaCm);

    // Verifica se encontrou um objeto a menos de 25 cm
    if (distanciaCm < 25.0) {
      pararRobo(); // Para o robô
      delay(1000); // Pequena pausa
      trocaEstado();
      andarParaFrente(); // Anda para frente
      objetoProximo = true; // Atualiza o estado para objeto encontrado
      break; // Sai do loop for
    }

    delay(200); // Aguarda antes de verificar novamente
  }

  if (!objetoProximo) {
    // Caso não encontre objeto a menos de 25 cm
    pararRobo(); // Para o robô
    delay(1000); // Pausa
    trocaEstado();
    andarParaTras(); // Anda para trás
  }

  delay(8000); // Aguarda antes de reiniciar o loop principal
}


//-----------------------DESENVOLVIMENTO DAS FUNÇÕES-------------------------------
//Pulso para o sensor HC_SR04
void TRIGGER(){//Gera pulso de 10us no pino de trigger do HC_SR04
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
}
//Cálculo da distância
float distancia(float tempo_us){//Calcula a distância (a Fisica é linda!)
  return((tempo_us*VelocidadeSom_mporus)/2);
}

// Função que faz o robô andar para frente
void andarParaFrente() {
  analogWriteFreq(100);
  // Coloca os motores para girar no sentido que move o robô para frente
  digitalWrite(IN1, HIGH); // Define o sentido do motor 1
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); // Define o sentido do motor 2
  digitalWrite(IN4, LOW);
  // Ajusta a velocidade dos motores (AJUSTAR O VALOR DEPOIS)
  analogWrite(MOTOR1, 150); // Motor 1
  analogWrite(MOTOR2, 150); // Motor 2
}

// Função que faz o robô andar para trás
void andarParaTras() {
  analogWriteFreq(100);
  // Coloca os motores para girar no sentido que move o robô para trás
  digitalWrite(IN1, LOW); // Define o sentido dos motores
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH);
  // Ajusta a velocidade dos motores (AJUSTAR O VALOR DEPOIS)
  analogWrite(MOTOR1, 150); // Motor 1
  analogWrite(MOTOR2, 150); // Motor 2
}

// Função que faz o robô girar dentro do próprio eixo
void girarRobo() {
  analogWriteFreq(100);
  // Coloca os motores para girar no sentido que move o robô para frente
  digitalWrite(IN1, HIGH); // Define o sentido do motor 1
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); // Define o sentido do motor 2
  digitalWrite(IN4, HIGH);
  // Ajusta a velocidade dos motores (AJUSTAR O VALOR DEPOIS)
  analogWrite(MOTOR1, 150); // Motor 1
  analogWrite(MOTOR2, 150); // Motor 2
}

// Função que para o robô imediatamente
void pararRobo() {
  analogWriteFreq(100);
  // Coloca o PWM dos motores em 0 para interromper o movimento
  analogWrite(MOTOR1, 0); // Motor 1
  analogWrite(MOTOR2, 0); // Motor 2
}

void trocaEstado() {

  digitalWrite(GREEN, HIGH);//Liga led verde
  delay(1000);
  digitalWrite(GREEN, LOW);//Liga led verde
  delay(200);

}


