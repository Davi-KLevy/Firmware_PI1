/* ---------- TESTE DE TENSÃO ---------- */
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

//----------------------INCLUSÃO DE BIBLIOTECAS -----------------------------------
#include <SPI.h>//Biblioteca para comunicação SPI com o cartão SD
#include <SD.h>//Biblioteca para o gerenciamento do cartão SD
File myFile;//Objeto myFile para o cartão SD

//-----------------------FUNÇÕES-------------------------------------
float lerVoltagem() {
  int rawADC = analogRead(VOLTAGE_PIN); // Lê o valor bruto do ADC
  float voltage = (rawADC / (float)ADC_RESOLUTION) * VREF; // Converte para tensão
  return voltage;
}

void testeVoltagemSensor() {
  Serial.println("Iniciando teste de sensor de tensão...");
  for (int i = 0; i < 10; i++) { // Lê 10 vezes para verificar a consistência
    float voltage = lerVoltagem();
    Serial.print("Leitura de tensão: ");
    Serial.print(voltage, 2); // Exibe com duas casas decimais
    Serial.println(" V");
    delay(500); // Espera meio segundo
  }
  Serial.println("Teste concluído.");
}

void monitorarVoltagem() {
  float voltage = lerVoltagem();
  if (voltage < 2.0) { // Limite inferior
    Serial.println("Alerta: Tensão baixa!");
  } else if (voltage > 3.3) { // Limite superior
    Serial.println("Alerta: Tensão alta!");
  }
  delay(500);
}

void registraVoltagemSD() {
  float voltage = lerVoltagem();
  myFile = SD.open("voltage_log.txt", FILE_WRITE);
  if (myFile) {
    myFile.print("Tensão medida: ");
    myFile.print(voltage, 2);
    myFile.println(" V");
    myFile.close();
    Serial.println("Tensão registrada no cartão SD.");
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

  pinMode(VOLTAGE_PIN, INPUT); // Configura o pino de tensão como entrada
  testeVoltagemSensor(); // Teste inicial do sensor de tensão
}

//-----------------------LOOP-----------------------------------
void loop() {
  monitorarVoltagem(); // Monitora os níveis de tensão
  registraVoltagemSD(); // Registra os dados no cartão SD
}
