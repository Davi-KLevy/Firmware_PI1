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
String fileName;  // nome do arquivo para o cartão SD

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
const float V_REF = 3.3;      // Tensão de referência do Raspberry Pi Pico
const int ADC = 4095;         // Resolução ADC do Raspberry Pi Pico (12 bits)
const float SENSITIVITY = 0.2481203008; // Sensibilidade do ACS712 (185mV/A para o modelo 5A)
const float ZERO_POINT = V_REF / 2; // Ponto zero do sensor (meia escala)
const float VOLTAGE_DIVIDER_RATIO = 5.0; // Razão do divisor de tensão (ex.: 100k/20k)
const float LOAD_RESISTANCE = 10.0; // Variável para a resistência conhecida no circuito em ohms

//INTERVALO PARA ESCRITA DE DADOS NO ARQUIVO NO CARTÃO SD
unsigned long previousTime = 0; // Armazena o último ciclo do data logger
const unsigned long interval = 100; // Ciclo de relógio do data logger (100 ms)


//-----------------------PROTÓTIPO DAS FUNÇÕES-------------------------------------
void H_BRIDGE_TEST();//Testa a ponte H
void EDGE_SENSORS_MEASURE(); //Averigua os sensores da fronteira
void TRIGGER();
float distancia(float tempo_us); //Calcula a distância a frente do HC_SR04

//-------------------PROTÓTIPO DAS FUNÇÕES DE DATA LOG-----------------------------
void dataLogger(); // escreve os dados obtidos durante a execução do firmware em uma tabela no cartão SD
void printData(unsigned long time, float circuitVoltage, float current, bool error = false);
String getNewNumber();  //obtem o próximo número da sequência para criar o arquivo de dataLogger
int getHighestFileNumber(); // obtem o maior número da sequência presente no nome dos arquivos dentro do cartão SD

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
 
  H_BRIDGE_TEST();
  //Teste do led RGB
  digitalWrite(RED, HIGH);//Liga led vermelho
  delay(200);
  digitalWrite(RED, LOW);//Desliga led vermelho
  delay(200);
  digitalWrite(GREEN, HIGH);//Liga led verde
  delay(200);
  digitalWrite(GREEN, LOW);//Desliga led verde
  delay(200);
  digitalWrite(BLUE, HIGH);//Liga led azul
  delay(200);
  digitalWrite(BLUE, LOW);//Desliga led azul
  delay(200);
  
  
}
//-----------------------SETUP DO SEGUNDO NÚCLEO-----------------------------------
void setup1(){
  Serial.begin(9600); // Inicia a comunicação serial

  // Inicializa o cartão SD
  if (!SD.begin(_CS)) {
    Serial.println("Erro ao inicializar o cartão SD.");
    while (1);
  }
  Serial.println("Cartão SD inicializado com sucesso!");

  // Cria o novo arquivo para escrever as colunas
  fileName = getNewNumber();
  myFile = SD.open(fileName, FILE_WRITE);

  if (myFile) {
    myFile.println("Tempo(s),Voltagem(V),Corrente(A), Distância do objeto(cm), Sensor de borda 1, Sensor de borda 2, Sensor de borda 3, Sensor de borda 4 ");
    Serial.print("Arquivo criado: ");
    Serial.println(fileName);
    myFile.close();
  } else {
    Serial.println("Falha ao criar o arquivo.");
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

  dataLogger();
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

void dataLogger() {
  unsigned long time = millis(); // Tempo atual em milissegundos

  if (time - previousTime >= interval) {
    previousTime = time; // Atualiza o último tempo medido

    // --- Medição de Corrente ---
    int rawCurrent = analogRead(CURRENT_PIN); // Leitura do sensor ACS712
    int rawVoltage = analogRead(VOLTAGE_PIN); // Leitura do divisor de tensão
    float voltageMeasured = (rawVoltage / (float)ADC) * V_REF; // Converte para tensão no ADC
    float current = (voltageMeasured - ZERO_POINT) / SENSITIVITY; // Calcula corrente em amperes
    float circuitVoltage = voltageMeasured * VOLTAGE_DIVIDER_RATIO; // Ajusta pela razão do divisor de tensão

    // Abrindo o arquivo para salvar os dados
    myFile = SD.open(fileName, FILE_WRITE);

    if (myFile) {
      // Escrevendo os dados no arquivo CSV
      myFile.print(time / 1000.0); // Converte tempo de milissegundos para segundos
      myFile.print(",");
      myFile.print(circuitVoltage, 2); // 2 casas decimais para tensão
      myFile.print(",");
      myFile.print(current, 2); // 2 casas decimais para corrente
      myFile.print(",");
      myFile.print(distancia(dt)*100);  // Distância medida pelo sensor de distância em cm
      myFile.print(",");
      myFile.print(EDGE_SENSORS[0]);
      myFile.print(",");
      myFile.print(EDGE_SENSORS[1]);
      myFile.print(",");
      myFile.print(EDGE_SENSORS[2]);
      myFile.print(",");
      myFile.println(EDGE_SENSORS[3]);
      myFile.close(); // Fecha o arquivo

      // Exibe no monitor serial
      printData(time, circuitVoltage, current);
    } else {
      // Se não conseguiu salvar, exibe erro no monitor serial
      printData(time, circuitVoltage, current, true);
    }
  }
}

void printData(unsigned long time, float circuitVoltage, float current, bool error) {
  if (error) {
    Serial.println("Falha ao abrir o arquivo.");
    Serial.println("Dados não salvos:");
  } else {
    Serial.println("Dados salvos:");
  }
  Serial.print("Tempo: ");
  Serial.print(time / 1000.0); // Exibe o tempo em segundos no monitor serial
  Serial.print(" s, Tensão: ");
  Serial.print(circuitVoltage, 2);
  Serial.print(" V, Corrente: ");
  Serial.print(current, 2);
  Serial.println(" A");
}

String getNewNumber() {
  // Obter o maior número presente no padrão 'dados-<número sequêncial>' para criar um novo arquivo
  int newNumber = getHighestFileNumber() + 1;
  return "data-" + String(newNumber) + ".csv";
}

// Função para pegar o maior número existente no padrão 'dados-<número sequêncial>.csv' dentro do cartão SD
int getHighestFileNumber() {
  File root = SD.open("/");
  int highestNumber = 0;

  // Faz uma iteração sobre os arquivos dentro do cartão SD
  while (true) {
    File entry = root.openNextFile();
    if (!entry) {
      // Não existem mais arquivos, finaliza o loop
      break;
    }

    String name = entry.name();
    if (name.startsWith("data-") && name.endsWith(".csv")) {
      // Extrai o número presente no nome do arquivo
      int number = name.substring(5, name.lastIndexOf(".")).toInt();
      if (number > highestNumber) {
        highestNumber = number;
      }
    }
    entry.close();
  }

  return highestNumber;
}