/* ---------- TESTE DE CORRENTE ---------- */
//  ᕙ(`▿´)ᕗ
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

const float VREF = 3.3; // Tensão de referência da RP2040
const int ADC_RESOLUTION = 1023; // Resolução do ADC (10 bits)
const float SENS_RESISTOR = 0.1; // Resistência do shunt em ohms para medição de corrente

//----------------------INCLUSÃO DE BIBLIOTECAS -----------------------------------
#include <SPI.h> //Biblioteca para comunicação SPI com o cartão SD
#include <SD.h>  //Biblioteca para o gerenciamento do cartão SD
File myFile;     //Objeto myFile para o cartão SD

//-----------------------FUNÇÕES-------------------------------------
float lerCorrente() {
  int rawADC = analogRead(CURRENT_PIN); // Lê o valor bruto do ADC
  float voltage = (rawADC / (float)ADC_RESOLUTION) * VREF; // Converte para tensão
  float current = voltage / SENS_RESISTOR; // Calcula a corrente usando a lei de Ohm! :)))))
  return current;
}

void testeCorrenteSensor() {
  Serial.println("Iniciando teste de sensor de corrente...");
  for (int i = 0; i < 10; i++) { // Lê 10 vezes para verificar a consistência
    float current = lerCorrente();
    Serial.print("Leitura de corrente: ");
    Serial.print(current, 2); // Exibe com duas casas decimais
    Serial.println(" A");
    delay(500); // Espera meio segundo
  }
  Serial.println("Teste concluído.");
}

void monitorarCorrente() {
  float current = lerCorrente();
  if (current < 0.1) { // Limite inferior (quase desligado!)
    Serial.println("Alerta: Corrente muito baixa!");
  } else if (current > 2.0) { // Limite superior (cuidado, vai queimar!)
    Serial.println("Alerta: Corrente alta demais!");
  }
  delay(500); // Delayzinho
}

void registraCorrenteSD() {
  float current = lerCorrente();
  myFile = SD.open("current_log.txt", FILE_WRITE);
  if (myFile) {
    myFile.print("Corrente medida: ");
    myFile.print(current, 2);
    myFile.println(" A");
    myFile.close();
    Serial.println("Corrente registrada no cartão SD.");
  } else {
    Serial.println("Erro ao abrir o arquivo para escrita.");
  }
}

//-----------------------SETUP----------------------------------
void setup() {
  Serial.begin(9600);
  if (!SD.begin(10)) { // Inicializa o cartão SD no pino CS (10 por padrão)
    Serial.println("Falha ao inicializar o cartão SD!");
    return;
  }
  Serial.println("Cartão SD inicializado.");

  pinMode(CURRENT_PIN, INPUT); // Configura o pino de corrente como entrada
  testeCorrenteSensor(); // Teste inicial do sensor de corrente
}

//-----------------------LOOP-----------------------------------
void loop() {
  monitorarCorrente(); // Monitora os níveis de corrente
  registraCorrenteSD(); // Registra os dados no cartão SD
}
