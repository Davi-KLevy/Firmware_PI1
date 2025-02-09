/*
FIRMWARE DE TESTES DO HARDWARE PROJETO ROBÔ SUMÔ - PI1

AUTORES: PAULO CALEB FERNANDES DA SILVA, GABRIEL DA CUNHA BARBACELI,

MICROCONTROLADOR UTILIZADO: RP2040 (Mbed as Raspberry pi pico)
*/
//-----------------------MAPEAMENTO DE HARDWARE------------------------------------
const uint8_t LED_RBP_PICO= 22, //Pino do led interno da placa do raspberry pi pico (recomendado para debug)
              VOLTAGE_PIN = A0, //Pino analógico dedicado a medição de tensão
              CURRENT_PIN = A2, //Pino analógico dedicado a medição de corrente
              EDGE_SENS_1    = 1,  //Pino do sensor de borda 1
              EDGE_SENS_2    = 2,  //Pino do sensor de borda 2
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
const uint8_t inputs  [6] { CURRENT_PIN, VOLTAGE_PIN, BOT, ECHO, EDGE_SENS_1, EDGE_SENS_2}; //Vetor de entradas
const uint8_t outputs [11] {TRIG, MOTOR1, IN1, IN2, IN3, IN4, MOTOR2, RED, GREEN, BLUE, BUZZER}; //Vetor de saídas

//--------------------VARIÁVEIS DO PROGRAMA--------------------------------
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_C6  1047
#define NOTE_D6  1175
#define NOTE_F6  1397


const float VelocidadeSom_mpors = 340, // em metros por segundo
            VelocidadeSom_mporus = 0.000340; // em metros por microsegundo
unsigned long int updateAnalog = 0; //Atualização da função que realiza a leitura analógica dos pinos de tensão e corrente
float instVoltage, //Global para armazenar a tensão instantânea
      instCurrent; //Global para armazenar a corrente instantânea
//----------------------INCLUSÃO DE BIBLIOTECAS -----------------------------------
#include <SPI.h>//Biblioteca para comunicação SPI com o cartão SD
#include <SD.h>//Biblioteca para o gerenciamento do cartão SD
File myFile;//Objeto myFile para o cartão SD
String fileName;  // nome do arquivo para o cartão SD

//-----------------------VARIÁVEIS DE PROGRAMA-------------------------------------
//Variáveis de programa
float dt = 0;//Armazena o tempo de resposta do sensor de distância
int PWM  = 0;//Armazena o PWM para os motores
//sensores de borda
volatile bool sensorEdgeTriggered = false; // Variável global para sinalizar que o sensor de borda foi acionado

//INTERVALO PARA ESCRITA DE DADOS NO ARQUIVO NO CARTÃO SD
unsigned long previousTime = 0; // Armazena o último ciclo do data logger
const unsigned long interval = 100; // Ciclo de relógio do data logger (100 ms)
bool botaoPressionado = false; // Variável para verificar se o botão de liga/desliga foi pressionado e segurado

//-----------------------PROTÓTIPO DAS FUNÇÕES-------------------------------------
void H_BRIDGE_TEST();//Testa a ponte H
void TRIGGER();
float distancia(float tempo_us); //Calcula a distância a frente do HC_SR04
void girarRobo(int potenciaMotor = 100);
void analogSensors();//Função para tratamento dos sensores analógicos
void ligaDesligaRobo();
void EDGE_SENSORS_MEASURE(); //Averigua os sensores da fronteira
void sensorEdgeISR();
void playVictoryTheme();
void setColor(int color);

//-------------------PROTÓTIPO DAS FUNÇÕES DE DATA LOG-----------------------------
void dataLogger(); // escreve os dados obtidos durante a execução do firmware em uma tabela no cartão SD
String getNewNumber();  //obtem o próximo número da sequência para criar o arquivo de dataLogger
int getHighestFileNumber(); // obtem o maior número da sequência presente no nome dos arquivos dentro do cartão SD

//-----------------------SETUP DO PRIMEIRO NÚCLEO----------------------------------
void setup() {
  //Configura as entradas e saídas
  for(uint8_t i = 0; i < 6; i++) pinMode(inputs[i], INPUT); //Inicializa as entradas
  for(uint8_t k = 0; k < 11; k++) pinMode(outputs[k], OUTPUT); //Inicializa as saídas
  for(uint8_t k = 0; k < 11; k++) digitalWrite(outputs[k], LOW); //Desativa todas as saídas

  // Anexa a interrupção para os sensores de borda
  attachInterrupt(digitalPinToInterrupt(EDGE_SENS_1), sensorEdgeISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EDGE_SENS_2), sensorEdgeISR, CHANGE);

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
  if(botaoPressionado) procurarObjeto();
}
//-----------------------SETUP DO SEGUNDO NÚCLEO-----------------------------------
void setup1(){

  // Inicializa o cartão SD
  if (SD.begin(_CS)) {

  // Cria o novo arquivo para escrever as colunas
  fileName = getNewNumber();
  myFile = SD.open(fileName, FILE_WRITE);

  if (myFile) {
    myFile.println("Tempo(s),Voltagem(V),Corrente(A), Distância do objeto(cm)");
    myFile.close();
  }
  }




//FIm do setup do segundo núcleo
}

//-----------------------LOOP DO SEGUNDO NÚCLEO------------------------------------
void loop1(){
  ligaDesligaRobo();

  if(botaoPressionado) {
    analogSensors();
    dataLogger();
  }

}

//-----------------------DESENVOLVIMENTO DAS FUNÇÕES-------------------------------

void analogSensors(){
  float voltageRead = 0; // V. auxiliar para armazenar a leitura analógica do sensor de tensão
  float voltageAvg = 0; // V. auxiliar para armazenar a média das leituras de tensão
  float currentRead = 0; // V. auxiliar para armazenar a leitura analógica do sensor de corrente
  float currentAvg = 0; // V. auxiliar para armazenar a média das leituras de corrente
  if((millis() - updateAnalog) >= 100){ // Temporização
    updateAnalog = millis(); // Reset da temporização
    //MEDIA DAS LEITURAS
    for (int k = 0; k < 10; k++){ // Somatório das amostras
        voltageRead += analogRead(VOLTAGE_PIN);//Acumula as leituras do pino analógico de tensão
        currentRead += analogRead(CURRENT_PIN);//Acumula as leituras do pino analógico de corrente
        delayMicroseconds(100);//A cada 100us
    }
    //MEDIÇÃO DE TENSÃO
    voltageAvg  = (voltageRead/10); // Obtendo a média das amostras de tensão
    instVoltage = (((voltageAvg/1023.00)*3.3)/0.24); // Realizando a conversão AD segundo os requisitos de hardware
    //MEDIÇÃO DA CORRENTE DE CONSUMO DO CIRCUITO (ACS712)
    currentAvg  = (currentRead/10); // Obtendo a média das amostras de corrente
    instCurrent = ((((((currentAvg/1023.00)*3.3)-1.25)*100)/0.185)-2);
  }
}

//Pulso para o sensor HC_SR04
void TRIGGER(){//Gera pulso de 10us no pino de trigger do HC_SR04
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
}

// Função ISR que será chamada quando ocorrer uma alteração em EDGE_SENS_1 ou EDGE_SENS_2
void sensorEdgeISR() {
  sensorEdgeTriggered = true;
}

void procurarObjeto(){
  bool objetoProximo = false; // Variável para verificar se encontrou objeto a menos de 25 cm
  sensorEdgeTriggered = false;

  reiniciaLoop();
  girarRobo(); // Inicia o movimento de giro

  for (uint8_t i = 0; i < 45; i++) {

    verificaSensoresGiro();

    if (sensorEdgeTriggered == true){
      break;
    }

    TRIGGER(); // Envia pulso para o sensor HC_SR04
    dt = pulseIn(ECHO, HIGH); // Mede o tempo de retorno do echo
    float distanciaCm = distancia(dt) * 100; // Calcula a distância em cm

    // Verifica se encontrou um objeto a menos de 25 cm
    if (distanciaCm < 50.0) {
      delay(200);
      pararRobo(); // Para o robô
      delay(100); // Pequena pausa
      trocaEstadoAchou();
      andarParaFrente(); // Anda para frente
      objetoProximo = true; // Atualiza o estado para objeto encontrado
      verificaSensores();
      if(!botaoPressionado) break;
      andarParaTras();
      delay(2000);
      pararRobo();
      break; // Sai do loop for
    }

    delay(200); // Aguarda antes de verificar novamente
  }

  if (!objetoProximo) {
    // Caso não encontre objeto a menos de 25 cm
    pararRobo(); // Para o robô
    delay(100); // Pausa
    trocaEstadoNaoAchou();
    andarParaFrente(); // Anda para trás
    verificaSensores();
    andarParaTras();
    delay(2000);
    pararRobo();
  }


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
  analogWrite(MOTOR1, 200); // Motor 1
  analogWrite(MOTOR2, 200); // Motor 2
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
  analogWrite(MOTOR1, 200); // Motor 1
  analogWrite(MOTOR2, 200); // Motor 2
}

// Função que faz o robô girar dentro do próprio eixo
void girarRobo(int potenciaMotor) {
  analogWriteFreq(100);
  // Coloca os motores para girar no sentido que move o robô para frente
  digitalWrite(IN1, HIGH); // Define o sentido do motor 1
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); // Define o sentido do motor 2
  digitalWrite(IN4, HIGH);
  // Ajusta a velocidade dos motores (AJUSTAR O VALOR DEPOIS)
  analogWrite(MOTOR1, potenciaMotor); // Motor 1
  analogWrite(MOTOR2, potenciaMotor); // Motor 2
}

void girarRoboInv() {
  analogWriteFreq(100);
  // Coloca os motores para girar no sentido que move o robô para frente
  digitalWrite(IN1, LOW); // Define o sentido do motor 1
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); // Define o sentido do motor 2
  digitalWrite(IN4, LOW);
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


void trocaEstadoAchou() {
  digitalWrite(GREEN, HIGH);//Liga led verde
  delay(1000);
  digitalWrite(GREEN, LOW);//Liga led verde
  delay(200);
}

void trocaEstadoNaoAchou() {

  digitalWrite(RED, HIGH);//Liga led verde
  delay(1000);
  digitalWrite(RED, LOW);//Liga led verde
  delay(200);

}

void reiniciaLoop() {

  digitalWrite(BLUE, HIGH);//Liga led verde
  delay(1000);
  digitalWrite(BLUE, LOW);//Liga led verde
  delay(200);

}

void verificaSensores(){
  // Reinicia o flag para garantir que a interrupção seja detectada a partir deste ponto
  sensorEdgeTriggered = false;

  // Aguarda até que um dos sensores dispare a interrupção
  while (!sensorEdgeTriggered) {

  }

  pararRobo();

  TRIGGER(); // Envia pulso para o sensor HC_SR04
  dt = pulseIn(ECHO, HIGH); // Mede o tempo de retorno do echo
  float distanciaCm = distancia(dt) * 100; // Calcula a distância em cm

  if (distanciaCm < 7){
    andarParaTras();
    delay(1000);
    girarRobo(250);
    playVictoryTheme();
    pararRobo();
    botaoPressionado = false;
  }
}


void verificaSensoresGiro(){
  if (sensorEdgeTriggered == true){
    pararRobo();
    delay(500);
    girarRoboInv();
    delay(2000);
  }
}

void ligaDesligaRobo(){
  bool estadoAnteriorBotao = botaoPressionado;

  if(digitalRead(BOT) == LOW){
   botaoPressionado = digitalRead(BOT) == HIGH ? false : true;

   delay(1000); // aguarda 1 segundo para verificar se o botão ainda está pressionado

   botaoPressionado = digitalRead(BOT) == HIGH ? false : true;

   if(botaoPressionado) {
    if(!estadoAnteriorBotao){
      digitalWrite(BLUE, HIGH); // comunica para o usuário que o robo está ligando
      delay(1000);
      digitalWrite(BLUE, LOW);
    } else {
      digitalWrite(RED, HIGH); // comunica para o usuário que o robo está desligando
      delay(1000);
      digitalWrite(RED, LOW);

       // Garantir que os motores estão parados
      analogWrite(MOTOR1, 0);
      analogWrite(MOTOR2, 0);

      // Desligar LEDs
      digitalWrite(RED, LOW);
      digitalWrite(GREEN, LOW);
      digitalWrite(BLUE, LOW);

      // Desligar o buzzer
      analogWrite(BUZZER, 0);

      botaoPressionado = false; // desliga o robo

      for(uint8_t k = 0; k < 11; k++) digitalWrite(outputs[k], LOW); // Desativa todas as saídas


    }

   } else botaoPressionado = estadoAnteriorBotao;
    // Aguarda o botão ser solto antes de continuar
    while (digitalRead(BOT) == LOW);
  }
}

void dataLogger() {
  unsigned long time = millis(); // Tempo atual em milissegundos

  if (time - previousTime >= interval) {
    previousTime = time; // Atualiza o último tempo medido

    TRIGGER();
    dt = pulseIn(ECHO, HIGH); //Medição do retorno de echo

    // Abrindo o arquivo para salvar os dados
    myFile = SD.open(fileName, FILE_WRITE);

    if (myFile) {
      // Escrevendo os dados no arquivo CSV
      myFile.print(time / 1000.0); // Converte tempo de milissegundos para segundos
      myFile.print(",");
      myFile.print(instVoltage, 2); // 2 casas decimais para tensão
      myFile.print(",");
      myFile.print(instCurrent, 2); // 2 casas decimais para corrente
      myFile.print(",");
      myFile.print(distancia(dt)*100);  // Distância medida pelo sensor de distância em cm
      myFile.println(",");
      myFile.close(); // Fecha o arquivo
    }
  }
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

void playVictoryTheme() {
  // Melodia ajustada da música de vitória
  int melody[] = {
    NOTE_F5, NOTE_F5, NOTE_D5, NOTE_F5, NOTE_C5, NOTE_F5, NOTE_D5, NOTE_C5, NOTE_F5, // Abertura
    NOTE_C5, NOTE_C6, NOTE_D6, NOTE_C6, NOTE_D6, NOTE_C6, // Meio da música
    NOTE_C5, NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F5, 0,  NOTE_F6 // Finalização
  };

  // Duração das notas: 2 = minima, 4 = semínima, 8 = colcheia, 16 = semicolcheia.
  int noteDurations[] = {
    4, 4, 7, 4, 7, 8, 7, 7, 2, 7,   // Abertura (pausa após o acorde inicial)
    8, 7, 7, 7, 6, 16,  // Meio da melodia
    16, 16, 8, 2, 4, 3  // Finalização
  };
  int ledColor[] = {
    0, 1, 2, 3, 4, 5, 6, 0, 1, 2,   // Abertura
    3, 4, 5, 6, 0, 1,               // Meio da melodia
    2, 3, 4, 5, 6, 7                // Finalização
  };

  for (int i = 0; i < sizeof(noteDurations) / sizeof(noteDurations[0]); i++) {
    int noteDuration = 1000 / noteDurations[i];

    if (melody[i] != 0) {  // Verifica se não é uma pausa
      tone(BUZZER, melody[i], noteDuration);
    }

    setColor(ledColor[i]); // Define a cor do LED

    int pauseBetweenNotes = noteDuration * 1.3;
    delay(pauseBetweenNotes);

    noTone(BUZZER);  // Para o som após a nota

    setColor(7); // Desliga o LED após a nota
  }
}

void setColor(int color) {
  switch (color) {
    case 0: // Vermelho
      analogWrite(RED, 255);
      analogWrite(GREEN, 0);
      analogWrite(BLUE, 0);
      break;
    case 1: // Verde
      analogWrite(RED, 0);
      analogWrite(GREEN, 255);
      analogWrite(BLUE, 0);
      break;
    case 2: // Azul
      analogWrite(RED, 0);
      analogWrite(GREEN, 0);
      analogWrite(BLUE, 255);
      break;
    case 3: // Amarelo (Vermelho + Verde)
      analogWrite(RED, 255);
      analogWrite(GREEN, 255);
      analogWrite(BLUE, 0);
      break;
    case 4: // Roxo (Vermelho + Azul)
      analogWrite(RED, 255);
      analogWrite(GREEN, 0);
      analogWrite(BLUE, 255);
      break;
    case 5: // Ciano (Verde + Azul)
      analogWrite(RED, 0);
      analogWrite(GREEN, 255);
      analogWrite(BLUE, 255);
      break;
    case 6: // Branco (todas as cores)
      analogWrite(RED, 255);
      analogWrite(GREEN, 255);
      analogWrite(BLUE, 255);
      break;
    case 7: // Desligado
      analogWrite(RED, 0);
      analogWrite(GREEN, 0);
      analogWrite(BLUE, 0);
      break;
  }
}
