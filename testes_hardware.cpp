//-----------------------MAPEAMENTO DE HARDWARE------------------------------------
const uint8_t LED_RBP_PICO = 22, // Pino do LED interno da placa do Raspberry Pi Pico 
              VOLTAGE_PIN = A0, // Pino analógico dedicado à medição de tensão 
              SENS_PIN    = A1, // Pino analógico dedicado à medição dos sensores LDR 
              CURRENT_PIN = A2, // Pino analógico dedicado à medição de corrente 
              LIGHT_CTRL  = 1,  // Pino para controle da intensidade dos LEDs excitadores dos sensores LDRs
              MUX_BIT1    = 2,  // Pino do bit 1 do multiplexador 
              MUX_BIT0    = 3,  // Pino do bit 0 do multiplexador 
              ECHO        = 4,  // Pino do echo do sensor de distância 
              TRIG        = 5,  // Pino de trigger do sensor de distância 
              MOTOR1      = 6,  // PWM para o motor 1 
              IN1         = 7,  // Sentido A do motor 1
              IN2         = 8,  // Sentido B do motor 1 
              IN3         = 9,  // Sentido A do motor 2 
              IN4         = 10, // Sentido B do motor 2 
              MOTOR2      = 11, // PWM para o motor 2 
              GREEN       = 12, // Pino de acionamento do LED verde 
              BLUE        = 13, // Pino de acionamento do LED azul 
              RED         = 14, // Pino de acionamento do LED vermelho 
              BUZZER      = 15, // Pino de acionamento do buzzer 
              BOT         = 16; // Pino do botão 

const float VREF = 3.3; // Tensão de referência da RP2040 
const int ADC_RESOLUTION = 1023; // Resolução do ADC (10 bits)

//----------------------INCLUSÃO DE BIBLIOTECAS -----------------------------------
#include <SPI.h> // Biblioteca para comunicação SPI com o cartão SD
#include <SD.h>  // Biblioteca para o gerenciamento do cartão SD
File myFile;     // Objeto myFile para o cartão SD 

//----------------------MUSIQUINHA DE BATERIA FRACA -----------------------------------
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523

#define QUARTER_NOTE 250  // Nota de 1/4 = 250ms
#define HALF_NOTE 500     // Nota de 1/2 = 500ms
#define WHOLE_NOTE 1000   // Nota inteira = 1000ms

//----------------------FUNÇÕES -----------------------------------
void testeLEDs() {
  Serial.println("Testando LEDs...");
  
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  digitalWrite(RED, HIGH);
  delay(500);
  digitalWrite(RED, LOW);

  digitalWrite(GREEN, HIGH);
  delay(500);
  digitalWrite(GREEN, LOW);

  digitalWrite(BLUE, HIGH);
  delay(500);
  digitalWrite(BLUE, LOW);

  Serial.println("Teste de LEDs concluído!");
}

void testeMotores() {
  Serial.println("Testando motores...");
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(MOTOR1, OUTPUT);

  // Gira no sentido horário
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(MOTOR1, 128); // 50% de velocidade
  delay(1000);

  // Gira no sentido anti-horário
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(MOTOR1, 128); // 50% de velocidade
  delay(1000);

  // Para o motor
  analogWrite(MOTOR1, 0);
  Serial.println("Teste de motores concluído!");
}

void testeBuzzer() {
  Serial.println("Testando buzzer...");
  
  pinMode(BUZZER, OUTPUT);

  for (int freq = 500; freq <= 2000; freq += 500) {
    tone(BUZZER, freq, 500); // Toca uma frequência por 500ms
    delay(500);
  }

  noTone(BUZZER);
  Serial.println("Teste de buzzer concluído!");
}

void testeSensorUltrassonico() {
  Serial.println("Testando sensor ultrassônico...");
  
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH);
  float distance = (duration * 0.034) / 2; // Converte para cm

  Serial.print("Distância medida: ");
  Serial.print(distance);
  Serial.println(" cm");
}

void testeBotao() {
  Serial.println("Testando botão...");

  pinMode(BOT, INPUT_PULLUP); // Usa resistor pull-up interno

  if (digitalRead(BOT) == LOW) {
    Serial.println("Botão pressionado!");
  } else {
    Serial.println("Botão não pressionado.");
  }
}

void testeSD() {
  Serial.println("Testando módulo SD...");

  if (!SD.begin(10)) {
    Serial.println("Erro ao inicializar o cartão SD.");
    return;
  }

  File testFile = SD.open("test.txt", FILE_WRITE);
  if (testFile) {
    testFile.println("Teste de gravação no SD!");
    testFile.close();
    Serial.println("Arquivo escrito com sucesso.");
  } else {
    Serial.println("Erro ao abrir o arquivo para escrita.");
  }
}

// Função para tocar a musiquinha de bateria fraca
void tocarMusicaBateriaFraca() {
  pinMode(BUZZER, OUTPUT); // Configura o pino do buzzer como saída

  int melodia[] = {NOTE_C4, NOTE_E4, NOTE_G4, NOTE_C5}; // Notas básicas
  int duracoes[] = {QUARTER_NOTE, QUARTER_NOTE, HALF_NOTE, WHOLE_NOTE}; // Durações respectivas

  for (int i = 0; i < 4; i++) {
    tone(BUZZER, melodia[i], duracoes[i]); // Toca a nota no buzzer
    delay(duracoes[i] * 1.3);             // Pausa entre as notas
  }

  noTone(BUZZER); // Garante que o buzzer pare ao final
}

// Função para testar a bateria
void testeBateria() {
  Serial.println("Testando a bateria...");

  // Configuração dos pinos necessários
  pinMode(GREEN, OUTPUT);       // LED verde para indicar bateria alta
  pinMode(VOLTAGE_PIN, INPUT);  // Pino analógico para medir tensão da bateria

  // Lê o valor bruto da tensão da bateria (pino analógico)
  int leituraTensao = analogRead(VOLTAGE_PIN);

  // Converte a leitura para a tensão real
  float tensaoBateria = (leituraTensao / (float)ADC_RESOLUTION) * VREF;

  // Printando a tensão da bateria
  Serial.print("Tensão medida: ");
  Serial.print(tensaoBateria);
  Serial.println(" V");

  // Define os limites para indicar o estado da bateria
  if (tensaoBateria > 3.5) {
    Serial.println("Bateria alta");
    digitalWrite(GREEN, HIGH);
    delay(1000);
    digitalWrite(GREEN, LOW);
  } else if (tensaoBateria > 3.2) {
    Serial.println("Bateria média");
  } else {
    Serial.println("Bateria baixa");
    tocarMusicaBateriaFraca(); // Toca a melodia de bateria baixa
  }
}

void setup() {
  Serial.begin(115200);

  //----------------------TESTES INICIAIS-----------------------------------
  testeLEDs();                 // Teste dos LEDs
  testeMotores();              // Teste dos motores
  testeBuzzer();               // Teste do buzzer
  testeSensorUltrassonico();   // Teste do sensor ultrassônico
  testeBotao();                // Teste do botão
  testeSD();                   // Teste do cartão SD
  testeBateria();              // Teste inicial da bateria
}

