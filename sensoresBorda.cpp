/* ---------- VERSÃO INICIAL DA PROGRAMAÇÃO DOS SENSORES DE BORDA ---------- */
//  ᕙ(`▿´)ᕗ

// Definições de pinos para motores e sensores
const int IN1 = 2, IN2 = 3, IN3 = 4, IN4 = 5, MOTOR1 = 6, MOTOR2 = 7;
const int TRIG = 8, ECHO = 9;
const int EDGE_SENSOR_PINS[] = {A0, A1, A2, A3}; // Pinos analógicos para sensores de borda (AJUSTAR SE NECESSÁRIO!!!)

// Variáveis globais
const float LIMIAR_SENSOR = 800; // Apenas um exemplo!!! Ajustar conforme os testes!
float EDGE_SENSORS[4]; 
float dt;

// Funções auxiliares
void TRIGGER() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
}

void EDGE_SENSORS_MEASURE() {
  for (int i = 0; i < 4; i++) {
    EDGE_SENSORS[i] = analogRead(EDGE_SENSOR_PINS[i]);
  }
}

float distancia(float tempo_us) {
  return tempo_us / 58.0; // Fórmula para converter microsegundos em cm
}

// Define um limiar para os sensores de borda (usado para determinar quando o sensor detecta a borda)
const float LIMIAR_SENSOR = 800; // Apenas um exemplo!!!!! Ajustar de acordo com os testes!

// Função que verifica os sensores dianteiros e para o robô se a borda for detectada
void verificarSensoresDianteiros() {
  // Verifica se qualquer um dos sensores dianteiros está acima do limiar
  if (EDGE_SENSORS[0] > LIMIAR_SENSOR || EDGE_SENSORS[1] > LIMIAR_SENSOR) {
    // Se a borda for dectectada, o robô para imedietadamente
    pararRobo();
  }
}

// Função que verifica os sensores traseiros e faz o robô andar para frente se a borda for detectada
void verificarSensoresTraseiros() {
  // Verifica se qualquer um dos sensores traseiros está acima do limiar
  if (EDGE_SENSORS[2] > LIMIAR_SENSOR || EDGE_SENSORS[3] > LIMIAR_SENSOR) {
    // Se a borda for dectectada, o robô anda para frente
    andarParaFrente();
  }
}

// Função que para o robô imediatamente
void pararRobo() {
  // Coloca o PWM dos motores em 0 para interromper o movimento
  analogWrite(MOTOR1, 0); // Motor 1
  analogWrite(MOTOR2, 0); // Motor 2
}

// Função que faz o robô andar para frente
void andarParaFrente() {
  // Coloca os motores para girar no sentido que move o robô para frente
  digitalWrite(IN1, HIGH); // Define o sentido do motor 1
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); // Define o sentido do motor 2
  digitalWrite(IN4, LOW);
  // Ajusta a velocidade dos motores (AJUSTAR O VALOR DEPOIS)
  analogWrite(MOTOR1, 150); // Motor 1
  analogWrite(MOTOR2, 150); // Motor 2
}

// Configurações iniciais
void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(MOTOR1, OUTPUT); pinMode(MOTOR2, OUTPUT);

  for(int i = 0; i < 4; i++){
    pinMode(EDGE_SENSOR_PINS[i], INPUT);
  }

  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  Serial.begin(9600);
}

// Função para testar a leitura dos sensores
void loop() {
  // Gera um pulso no sensor de distância ultrassônico (HC-SR04) para medir objetos à frente
  TRIGGER();

  // Mede o tempo de resposta do sensor ultrassônico
  dt = pulseIn(ECHO, HIGH);

  // Calcula a distância baseada no tempo de resposta e na velocidade do som
  distancia(dt);

  // Atualiza os valores dos sensores de borda, coletando novas leituras
  EDGE_SENSORS_MEASURE();

  // Verifica os sensores dianteiros para evitar sair da borda pela frente
  verificarSensoresDianteiros();

  // Verifica os sensores traseiros para evitar sair da borda pela parte de trás
  verificarSensoresTraseiros();

  // Envia os valores dos sensores de borda para o monitor serial, para debug
  Serial.print("Sensores da borda: ");
  Serial.print(EDGE_SENSORS[0]); // Valor do sensor dianteiro esquerdo
  Serial.print(",");
  Serial.print(EDGE_SENSORS[1]); // Valor do sensor dianteiro direito
  Serial.print(",");
  Serial.print(EDGE_SENSORS[2]); // Valor do sensor traseiro esquerdo
  Serial.print(",");
  Serial.println(EDGE_SENSORS[3]); // Valor do sensor traseiro direito

  delay(250);
}