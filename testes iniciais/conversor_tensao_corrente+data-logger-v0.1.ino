#include <SD.h>
#include <SPI.h>

// Pino de seleção do SD Card
const int chipSelect = 17;

// Configurações para o ACS712 (corrente)
const int ACS712_PIN = A2;    // Pino onde o sensor de corrente está conectado
const float V_REF = 3.3;      // Tensão de referência do Raspberry Pi Pico
const int ADC_RESOLUTION = 4095; // Resolução ADC do Raspberry Pi Pico (12 bits)
const float SENSITIVITY = 0.2481203008; // Sensibilidade do ACS712 (185mV/A para o modelo 5A)
const float ZERO_POINT = V_REF / 2; // Ponto zero do sensor (meia escala)

// Configurações para a medição de tensão
const int VOLTAGE_PIN = A0;    // Pino ADC para o divisor de tensão
const float VOLTAGE_DIVIDER_RATIO = 5.0; // Razão do divisor de tensão (ex.: 100k/20k)

// Variável para a resistência conhecida no circuito
const float LOAD_RESISTANCE = 10.0; // Resistência em ohms

void setup() {
  Serial.begin(9600); // Inicia a comunicação serial

  // Inicializa o cartão SD
  if (!SD.begin(chipSelect)) {
    Serial.println("Erro ao inicializar o cartão SD.");
    while (1);
  }
  Serial.println("Cartão SD inicializado com sucesso!");

 String fileName = getNewNumber();

  // Cria o novo arquivo para escrever as colunas
  File file = SD.open(fileName, FILE_WRITE);

  if (file) {
    file.println("Tempo(s),Voltagem(V),Corrente(A)");
    Serial.print("Arquivo criado: ");
    Serial.println(fileName);
    file.close();
  } else {
    Serial.println("Falha ao criar o arquivo.");
  }
}

void loop() {
  // --- Medição de Corrente ---
  int rawCurrent = analogRead(ACS712_PIN); // Leitura do sensor ACS712
  float voltageMeasured = (rawVoltage / (float)ADC_RESOLUTION) * V_REF; // Converte para tensão no ADC

  float current = (voltageMeasured - ZERO_POINT) / SENSITIVITY; // Calcula corrente em amperes

  unsigned long time = millis() / 1000; // tempo em segundos

  // --- Medição de Tensão ---
  int rawVoltage = analogRead(VOLTAGE_PIN); // Leitura do divisor de tensão
  float circuitVoltage = voltageMeasured * VOLTAGE_DIVIDER_RATIO; // Ajusta pela razão do divisor de tensão

  String fileName = getNewNumber();
   // Abrindo o arquivo para salvar os dados
  File file = SD.open(fileName, FILE_WRITE);

  if (file) {
    // Escrevendo os dados no arquivo CSV
    file.print(time);
    file.print(",");
    file.print(tensao, 2); // 2 casas decimais para tensão
    file.print(",");
    file.println(corrente, 2); // 2 casas decimais para corrente
    file.close(); // Fecha o arquivo

    printData(time, circuitVoltage, current) // mostra os dados que foram salvos no arquivo no monitor serial
  } else printData(time, circuitVoltage, current, true) // mostra os dados que não foram salvos no arquivo no monitor serial

  delay(1000); // Atualiza os dados a cada 1 segundo
}

void printData(unsigned long time, float circuitVoltage, float current, bool error = false){
  if(error) {
    Serial.println("Falha ao abrir o arquivo.");
    Serial.println("Dados não salvos:")
  }
  if(!error) Serial.println(" A -> Dados salvos.");
  Serial.print("time: ");
  Serial.print(time);
  Serial.print(" s, Tensao: ");
  Serial.print(circuitVoltage, 2);
  Serial.print(" V, Corrente: ");
  Serial.print(current, 2);

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

  // faz uma iteração sobre os arquivos dentro do cartão SD
  while (true) {
    File entry = root.openNextFile();
    if (!entry) {
      // não existem mais arquivos, finaliza o loop
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
