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
void H_BRIDGE_TEST();//Testa a ponte H
void EDGE_SENSORS_MEASURE(); //Averigua os sensores da fronteira
void TRIGGER();
float distancia(float tempo_us); //Calcula a distância a frente do HC_SR04
bool TEST_BATTERY_VOLTAGE(); // Testa a tensão da bateria
bool TEST_CURRENT(); // Testa a corrente
bool TEST_SENSORS(); // Testa os sensores

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
  delay(300);
  analogWrite(BUZZER, 0);//desativa o buzzer
  
//Fim do setup do primeiro núcleo
}
//-----------------------LOOP DO PRIMEIRO NÚCLEO-----------------------------------
void loop() {
  bool battery_ok = TEST_BATTERY_VOLTAGE();
  bool current_ok = TEST_CURRENT();
  bool sensors_ok = TEST_SENSORS();

  if (battery_ok && current_ok && sensors_ok) {
    digitalWrite(GREEN, HIGH); // Liga o LED verde
    digitalWrite(RED, LOW);    // Desliga o LED vermelho
  } else {
    digitalWrite(GREEN, LOW);  // Desliga o LED verde
    digitalWrite(RED, HIGH);   // Liga o LED vermelho
  }

  delay(1000); // Aguarda 1 segundo antes de repetir o teste
}

//-----------------------SETUP DO SEGUNDO NÚCLEO-----------------------------------
void setup1(){
  Serial.begin(9600);//Inicializa o serial
  //Passos para inicializar a comunicação com o cartão SD
  Serial.print("Inicializando o cartão SD");
  //Configura o Barramento SPI
  SPI.setRX(_MISO); //Seta o MISO
  SPI.setTX(_MOSI); //Seta o MOSI
  SPI.setSCK(_SCK); //Seta o clock
  //Testa a comunicação SPI com o dispositivo conectado ao chip select
  if (!SD.begin(_CS)) {
    //Se o dispositivo não inicializar
    Serial.println("Inicialização do cartão SD falhou");
    return;//Prende o núcleo no setup
  }
  //Se o dispositivo inicializar
  Serial.println("Inicialização do cartão SD concluida");
  myFile = SD.open("test.txt", FILE_WRITE); //Abre o arquivo "test.txt"

  if (myFile) { //Se o arquivo abriu
    Serial.print("Escrevendo no arquivo test.txt..."); //Mostra no serial a escrita do arquivo
    myFile.println("testing 1, 2, 3."); //Escreve uma string no arquivo
    myFile.close();//Fecha o arquivo
  } else {
    //Se o arquivo não abriu
    Serial.println("Erro ao abrir o arquivo test.txt");
  }
  //Abre o arquivo para leitura
  myFile = SD.open("test.txt");
  if (myFile) {
    Serial.println("test.txt:");
    //Realiza a leitura dos dados contidos no arquivo
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    //Fecha o arquivo
    myFile.close();
  } else {
    //Se o arquivo não abrir, mostra erro no serial
    Serial.println("Erro ao abrir o arquivo test.txt");
  }


//FIm do setup do segundo núcleo
}

//-----------------------LOOP DO SEGUNDO NÚCLEO------------------------------------
void loop1(){
   TRIGGER();//Chamada da função trigger
  dt = pulseIn(ECHO, HIGH); //Medição do retorno de echo
  distancia(dt); //Cálculo da distância
  EDGE_SENSORS_MEASURE(); //Chamada da função de averiguação dos sensores da borda
  Serial.print("ACD sensor de tensão: ");
  Serial.println(analogRead(VOLTAGE_PIN));
  Serial.print("ADC sensor de corrente: ");
  Serial.println(analogRead(CURRENT_PIN));
  Serial.print("Distancia em centimetros: ");
  Serial.println(distancia(dt)*100);
  Serial.print("Sensores da borda: ");
  Serial.print(EDGE_SENSORS[0]);
  Serial.print(",");
  Serial.print(EDGE_SENSORS[1]);
  Serial.print(",");
  Serial.print(EDGE_SENSORS[2]);
  Serial.print(",");
  Serial.println(EDGE_SENSORS[3]);
  Serial.println("-----------------");
  delay(250);
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
//Obtenção dos sensores da borda
void EDGE_SENSORS_MEASURE(){
  //Obs: a organização do vetor está levando em conta a montagem em protoboard, isso não se reflete na disposição física dos sensores no robô.
  //Sensor 1
  digitalWrite(MUX_BIT0, LOW);
  digitalWrite(MUX_BIT1, LOW);
  delayMicroseconds(10);
  EDGE_SENSORS[2] = analogRead(SENS_PIN);
  //Sensor 2
  digitalWrite(MUX_BIT0, HIGH);
  digitalWrite(MUX_BIT1, LOW);
  delayMicroseconds(10);
  EDGE_SENSORS[1] = analogRead(SENS_PIN);
  //Sensor 3
  digitalWrite(MUX_BIT0, LOW);
  digitalWrite(MUX_BIT1, HIGH);
  delayMicroseconds(10);
  EDGE_SENSORS[3] = analogRead(SENS_PIN);
  //Sensor 4
  digitalWrite(MUX_BIT0, HIGH);
  digitalWrite(MUX_BIT1, HIGH);
  delayMicroseconds(10);
  EDGE_SENSORS[0] = analogRead(SENS_PIN);
}
//Teste da ponte H
void H_BRIDGE_TEST(){
  analogWriteFreq(100);//Seta a frequencia para o chaveamento dos motores
  if(!digitalRead(BOT)){//Botão solto
    //Sentido e força do motor 1
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    //Sentido e força do motor 1
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    //Varredura do PWM para os motores
    for(PWM = 0; PWM <= 255; PWM++){
    delay(5);
    analogWrite(MOTOR1, PWM);
    analogWrite(MOTOR2, PWM);
    }
    delay(10000);
    for(PWM >= 255; PWM >= 0; PWM--){
    delay(5);
    analogWrite(MOTOR1, PWM);
    analogWrite(MOTOR2, PWM);
    }
    delay(5000);
  } else{
    //Sentido e força do motor 1
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    //Sentido e força do motor 1
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    //Varredura do PWM para os motores
    for(PWM = 0; PWM <= 255; PWM++){
    delay(5);
    analogWrite(MOTOR1, PWM);
    analogWrite(MOTOR2, PWM);
    }
    delay(10000);
    for(PWM >= 255; PWM >= 0; PWM--){
    delay(5);
    analogWrite(MOTOR1, PWM);
    analogWrite(MOTOR2, PWM);
    }
    delay(5000);
  }
}

// Função para testar a tensão da bateria
bool TEST_BATTERY_VOLTAGE() {
  int voltage = analogRead(VOLTAGE_PIN);
  Serial.print("Tensão da bateria: ");
  Serial.println(voltage);
  return (voltage > 0); // Ajuste a condição conforme necessário
}

// Função para testar a corrente
bool TEST_CURRENT() {
  int current = analogRead(CURRENT_PIN);
  Serial.print("Corrente: ");
  Serial.println(current);
  return (current > 0); // Ajuste a condição conforme necessário
}

// Função para testar os sensores
bool TEST_SENSORS() {
  EDGE_SENSORS_MEASURE();
  for (int i = 0; i < 4; i++) {
    Serial.print("Sensor de borda ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(EDGE_SENSORS[i]);
    if (EDGE_SENSORS[i] <= 0) {
      return false;
    }
  }
  return true;
}