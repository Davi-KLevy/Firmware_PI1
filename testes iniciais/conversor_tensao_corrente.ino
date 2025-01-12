// Configurações para o ACS712 (corrente)
const int ACS712_PIN = A0;    // Pino onde o sensor de corrente está conectado
const float V_REF = 3.3;      // Tensão de referência do Raspberry Pi Pico
const int ADC_RESOLUTION = 4095; // Resolução ADC do Raspberry Pi Pico (12 bits)
const float SENSITIVITY = 0.185; // Sensibilidade do ACS712 (185mV/A para o modelo 5A)
const float ZERO_POINT = V_REF / 2; // Ponto zero do sensor (meia escala)

// Configurações para a medição de tensão
const int VOLTAGE_PIN = A1;    // Pino ADC para o divisor de tensão
const float VOLTAGE_DIVIDER_RATIO = 5.0; // Razão do divisor de tensão (ex.: 100k/20k)

// Variável para a resistência conhecida no circuito
const float LOAD_RESISTANCE = 10.0; // Resistência em ohms

void setup() {
  Serial.begin(9600); // Inicia a comunicação serial
}

void loop() {
  // --- Medição de Corrente ---
  int rawCurrent = analogRead(ACS712_PIN); // Leitura do sensor ACS712
  float voltageCurrent = (rawCurrent / (float)ADC_RESOLUTION) * V_REF; // Converte para tensão
  float current = (voltageCurrent - ZERO_POINT) / SENSITIVITY; // Calcula corrente em amperes

  // --- Medição de Tensão ---
  int rawVoltage = analogRead(VOLTAGE_PIN); // Leitura do divisor de tensão
  float voltageMeasured = (rawVoltage / (float)ADC_RESOLUTION) * V_REF; // Converte para tensão no ADC
  float circuitVoltage = voltageMeasured * VOLTAGE_DIVIDER_RATIO; // Ajusta pela razão do divisor de tensão

  // --- Exibir os resultados ---
  Serial.print("Corrente (A): ");
  Serial.print(current, 3); // Exibir corrente com 3 casas decimais
  Serial.print(" | Tensão (V): ");
  Serial.println(circuitVoltage, 2); // Exibir tensão com 2 casas decimais

  delay(1000); // Atualiza os dados a cada 1 segundo
}
