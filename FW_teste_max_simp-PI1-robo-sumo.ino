// Firmware simplificado focando em testar sensores, motores e LEDs

#include <Arduino.h> // Biblioteca principal do Arduino
#include <SPI.h>     // Biblioteca para comunicação SPI

// Definição de pinos e constantes
#define MOTOR_PIN 9   // Pino conectado ao motor
#define LED_PIN 13    // Pino conectado ao LED
#define SENSOR_PIN A0 // Pino analógico conectado ao sensor

void setup() {
  // Inicializa a comunicação serial para depuração
  Serial.begin(9600);

  // Configuração dos pinos
  pinMode(MOTOR_PIN, OUTPUT); // Configura o pino do motor como saída
  pinMode(LED_PIN, OUTPUT);   // Configura o pino do LED como saída
  pinMode(SENSOR_PIN, INPUT); // Configura o pino do sensor como entrada

  // Estados iniciais
  digitalWrite(MOTOR_PIN, LOW); // Define o motor como desligado
  digitalWrite(LED_PIN, LOW);   // Define o LED como apagado

  // Mensagem inicial indicando o início dos testes de hardware
  Serial.println("Iniciando testes de hardware...");

  // Liga o LED
  digitalWrite(LED_PIN, HIGH);

  // Liga o motor na máxima velocidade
  analogWrite(MOTOR_PIN, 255);
}

void loop() {
  // Teste do sensor
  Serial.println("Testando Sensor...");   // Mensagem de depuração
  int sensorValue = analogRead(SENSOR_PIN); // Lê o valor do sensor analógico
  Serial.print("Valor do Sensor: ");       // Exibe o valor do sensor
  Serial.println(sensorValue);
  delay(1000);                             // Aguarda 1 segundo

  // Teste de comunicação SPI (loopback ou teste de comunicação, se aplicável)
  Serial.println("Testando SPI...");      // Mensagem de depuração
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0)); // Inicia a transação SPI com configurações específicas
  byte testByte = 0xAA;                   // Byte de teste enviado pelo SPI
  byte receivedByte = SPI.transfer(testByte); // Transfere e recebe um byte via SPI
  SPI.endTransaction();                   // Finaliza a transação SPI

  // Exibe os valores enviados e recebidos via SPI
  Serial.print("Enviado: 0x");
  Serial.print(testByte, HEX);
  Serial.print(", Recebido: 0x");
  Serial.println(receivedByte, HEX);

  delay(1000); // Aguarda 1 segundo antes de reiniciar o loop
}