#include <Arduino.h>

// Mapeamento de pinos
const uint8_t TRIG = 5;         // Pino Trigger do sensor ultrassônico para iniciar a medição de distância
const uint8_t ECHO = 4;         // Pino Echo do sensor ultrassônico para capturar o tempo do pulso de retorno
const uint8_t MUX_BIT0 = 3;     // Bit 0 de controle do multiplexador (para seleção do sensor de borda)
const uint8_t MUX_BIT1 = 2;     // Bit 1 de controle do multiplexador (para seleção do sensor de borda)
const uint8_t SENS_PIN = A1;    // Pino analógico conectado aos sensores de borda (LDRs)
const uint8_t MOTOR1 = 6;       // Pino PWM para controle da velocidade do motor 1
const uint8_t MOTOR2 = 11;      // Pino PWM para controle da velocidade do motor 2
const uint8_t IN1 = 7;          // Pino para definir o sentido A do motor 1
const uint8_t IN2 = 8;          // Pino para definir o sentido B do motor 1
const uint8_t IN3 = 9;          // Pino para definir o sentido A do motor 2
const uint8_t IN4 = 10;         // Pino para definir o sentido B do motor 2

// Constantes
const float VelocidadeSom_mporus = 0.000340; // Velocidade do som em metros por microssegundo
const float DISTANCIA_MINIMA_CM = 10.0;      // Distância mínima em cm para considerar que o objeto está sendo empurrado
const int LIMIAR_BORDA = 500;               // Valor de referência para detectar bordas usando os sensores

// Variáveis
float EDGE_SENSORS[4] = {0, 0, 0, 0};       // Array para armazenar as leituras dos sensores de borda

// Função para configurar os pinos utilizados
void configurarPinos() {
  pinMode(TRIG, OUTPUT);       // Configura o pino Trigger como saída
  pinMode(ECHO, INPUT);        // Configura o pino Echo como entrada
  pinMode(MUX_BIT0, OUTPUT);   // Configura o Bit 0 do multiplexador como saída
  pinMode(MUX_BIT1, OUTPUT);   // Configura o Bit 1 do multiplexador como saída
  pinMode(SENS_PIN, INPUT);    // Configura o pino do sensor analógico como entrada
  pinMode(IN1, OUTPUT);        // Configura o pino IN1 como saída
  pinMode(IN2, OUTPUT);        // Configura o pino IN2 como saída
  pinMode(IN3, OUTPUT);        // Configura o pino IN3 como saída
  pinMode(IN4, OUTPUT);        // Configura o pino IN4 como saída
  pinMode(MOTOR1, OUTPUT);     // Configura o pino MOTOR1 como saída PWM
  pinMode(MOTOR2, OUTPUT);     // Configura o pino MOTOR2 como saída PWM
  digitalWrite(IN1, LOW);      // Inicializa o pino IN1 em nível baixo
  digitalWrite(IN2, LOW);      // Inicializa o pino IN2 em nível baixo
  digitalWrite(IN3, LOW);      // Inicializa o pino IN3 em nível baixo
  digitalWrite(IN4, LOW);      // Inicializa o pino IN4 em nível baixo
}

// Função para medir valores dos sensores de borda usando o multiplexador
void medirSensoresDeBorda() {
  digitalWrite(MUX_BIT0, LOW); digitalWrite(MUX_BIT1, LOW); delayMicroseconds(10); // Seleciona o sensor 0
  EDGE_SENSORS[0] = analogRead(SENS_PIN); // Lê o valor do sensor 0

  digitalWrite(MUX_BIT0, HIGH); digitalWrite(MUX_BIT1, LOW); delayMicroseconds(10); // Seleciona o sensor 1
  EDGE_SENSORS[1] = analogRead(SENS_PIN); // Lê o valor do sensor 1

  digitalWrite(MUX_BIT0, LOW); digitalWrite(MUX_BIT1, HIGH); delayMicroseconds(10); // Seleciona o sensor 2
  EDGE_SENSORS[2] = analogRead(SENS_PIN); // Lê o valor do sensor 2

  digitalWrite(MUX_BIT0, HIGH); digitalWrite(MUX_BIT1, HIGH); delayMicroseconds(10); // Seleciona o sensor 3
  EDGE_SENSORS[3] = analogRead(SENS_PIN); // Lê o valor do sensor 3
}

// Função para verificar se alguma borda foi detectada
bool verificarBorda() {
  for (int i = 0; i < 4; i++) {              // Percorre os valores dos sensores de borda
    if (EDGE_SENSORS[i] < LIMIAR_BORDA) {    // Compara o valor do sensor com o limiar
      return true;                           // Retorna verdadeiro se uma borda for detectada
    }
  }
  return false;                              // Retorna falso se nenhuma borda for detectada
}

// Função para calcular a distância usando o sensor ultrassônico
float calcularDistancia() {
  digitalWrite(TRIG, LOW); delayMicroseconds(2);       // Garante que o Trigger esteja em nível baixo
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);     // Envia um pulso de 10µs no Trigger
  digitalWrite(TRIG, LOW);                             // Retorna o Trigger para nível baixo
  float tempo = pulseIn(ECHO, HIGH);                   // Mede o tempo do pulso no Echo
  return (tempo * VelocidadeSom_mporus) / 2 * 100;     // Converte o tempo em distância (cm)
}

// Função para mover o robô para frente
void moverFrente() {
  digitalWrite(IN1, HIGH);  // Ativa o motor 1 no sentido A
  digitalWrite(IN2, LOW);   // Desativa o sentido B do motor 1
  digitalWrite(IN3, HIGH);  // Ativa o motor 2 no sentido A
  digitalWrite(IN4, LOW);   // Desativa o sentido B do motor 2
  analogWrite(MOTOR1, 200); // Define a velocidade do motor 1
  analogWrite(MOTOR2, 200); // Define a velocidade do motor 2
}

// Função para mover o robô para trás
void moverTras() {
  digitalWrite(IN1, LOW);  // Desativa o sentido A do motor 1
  digitalWrite(IN2, HIGH); // Ativa o motor 1 no sentido B
  digitalWrite(IN3, LOW);  // Desativa o sentido A do motor 2
  digitalWrite(IN4, HIGH); // Ativa o motor 2 no sentido B
  analogWrite(MOTOR1, 200); // Define a velocidade do motor 1
  analogWrite(MOTOR2, 200); // Define a velocidade do motor 2
}

// Função para parar os motores
void pararMotores() {
  digitalWrite(IN1, LOW);   // Desativa o motor 1 no sentido A
  digitalWrite(IN2, LOW);   // Desativa o motor 1 no sentido B
  digitalWrite(IN3, LOW);   // Desativa o motor 2 no sentido A
  digitalWrite(IN4, LOW);   // Desativa o motor 2 no sentido B
  analogWrite(MOTOR1, 0);   // Define a velocidade do motor 1 como 0
  analogWrite(MOTOR2, 0);   // Define a velocidade do motor 2 como 0
}

// Programa principal
void setup() {
  configurarPinos();            // Configura os pinos do robô
  Serial.begin(9600);           // Inicializa a comunicação serial
}

void loop() {
  moverFrente();                // Robô começa a andar para frente
  
  medirSensoresDeBorda();       // Lê os valores dos sensores de borda
  if (verificarBorda()) {       // Verifica se uma borda foi detectada
    pararMotores();             // Para o robô se houver borda
    Serial.println("Borda detectada. Robô parado.");
    delay(1000);                // Aguarda 1 segundo
    return;                     // Finaliza o loop atual
  }

  float distancia = calcularDistancia(); // Mede a distância do objeto à frente
  Serial.print("Distância do objeto: ");
  Serial.println(distancia);

  if (distancia > DISTANCIA_MINIMA_CM) { // Verifica se o objeto está muito distante
    pararMotores();                      // Para o robô
    Serial.println("Objeto muito distante. Recuando...");
    moverTras();                         // Anda para trás
    delay(1000);                         // Aguarda 1 segundo
    pararMotores();                      // Para novamente
  } else if (distancia <= DISTANCIA_MINIMA_CM) { // Verifica se o objeto está a uma distância ideal
    pararMotores();                              // Para o robô
    Serial.println("Objeto empurrado. Desligando...");
    while (true);                                // Finaliza o robô
  }