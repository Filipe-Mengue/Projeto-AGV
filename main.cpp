#include <Arduino.h>

// ================= HARDWARE =================
const int motorE_PWM = 9;     // Motor Esquerdo
const int motorD_PWM = 10;    // Motor Direito
const int ledPin = 13;        // LED de Debug

#define TRIG_D 3
#define ECHO_D 2
#define TRIG_E 5
#define ECHO_E 4

// ================= PID =================
// Aumentei levemente o Kp para forçar mais a saída da parede
float Kp = 5.82 ; 
float Ki = 0.0305 ; 
float Kd = 0.3 ; 

float erro = 0, erroAnterior = 0;
float P = 0, I = 0, D = 0;
float PIDout = 0;

// Limites
const float LIMIT_PID = 150; // Permite correção mais agressiva
const float LIMIT_I = 80.0;

// ================= VELOCIDADE =================
//int baseVel = 160;           // Reduzi um pouco a base para dar mais margem de manobra
const int MIN_SPEED = 80;    // Mínimo para roda girar
const int MAX_PWM = 250;
int baseVel_D = 175;           // Reduzi um pouco a base para dar mais margem de manobra
int baseVel_E = 180;           // Reduzi um pouco a base para dar mais margem de manobra

// Tempo
unsigned long tempoAnterior = 0;
float dt = 0;

// ================= FILTRO E MEMÓRIA =================
const int WIN = 5;
float bufE[WIN], bufD[WIN];
int idxE = 0, idxD = 0;

// Variáveis para guardar a última leitura válida caso o sensor falhe
float lastValidD = 20.0;
float lastValidE = 20.0;

// Média Móvel
float meanBuffer(float *buf) {
  float s = 0;
  for (int i = 0; i < WIN; i++) s += buf[i];
  return s / (float)WIN;
}

void initBuffers() {
  for (int i = 0; i < WIN; i++) {
    bufE[i] = 20.0;
    bufD[i] = 20.0;
  }
}

// ==== NOVA FUNÇÃO DE LEITURA (MAIS SEGURA) ====
float medirDistanciaD(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);

  long duracao = pulseIn(echo, HIGH, 5000L);

  // Se não detectou parede → devolve distância GRANDE
  if (duracao == 0) {
    return 100.0;  // parede muito distante → usado para sair da curva
  }

  float distancia = (duracao * 0.0273) / 2.0;
  return distancia;
}

float medirDistanciaE(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);

  long duracao = pulseIn(echo, HIGH, 5000L);

  // Se não detectou parede → devolve distância GRANDE
  if (duracao == 0) {
    return 100.0;  // parede muito distante → usado para sair da curva
  }

  float distancia = (duracao * 0.0273) / 2.0;
  return distancia;
}

void setup() {
  Serial.begin(115200);
  pinMode(motorE_PWM, OUTPUT);
  pinMode(motorD_PWM, OUTPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(TRIG_D, OUTPUT);
  pinMode(ECHO_D, INPUT);
  pinMode(TRIG_E, OUTPUT);
  pinMode(ECHO_E, INPUT);

  initBuffers();
  tempoAnterior = millis();
  
  Serial.println("=== INICIANDO (MODO ROBUSTO A FALHAS) ===");
  delay(1000);
}

void loop() {
  unsigned long now = millis();
  dt = (now - tempoAnterior) / 1000.0;
  if (dt == 0) return;
  tempoAnterior = now;

  
  float dD = medirDistanciaD(TRIG_D, ECHO_D);
  float dE = medirDistanciaE(TRIG_E, ECHO_E);


  // 2. Filtro Média
  bufD[idxD] = dD; idxD = (idxD + 1) % WIN;
  bufE[idxE] = dE; idxE = (idxE + 1) % WIN;
  
  float sensorD = meanBuffer(bufD);
  float sensorE = meanBuffer(bufE);

  // 3. Cálculo PID
  // Erro POSITIVO = Perto da Esquerda (D > E) -> Precisa virar Direita
  // Erro NEGATIVO = Perto da Direita (D < E) -> Precisa virar Esquerda
  erro = sensorD - sensorE; 

  P = erro * Kp;
  I += erro * Ki * dt;
  if (I > LIMIT_I) I = LIMIT_I;
  if (I < -LIMIT_I) I = -LIMIT_I;
  D = Kd * (erro - erroAnterior) / dt;
  erroAnterior = erro;

  PIDout = P + I + D;

  // Trava PID
  if (PIDout > LIMIT_PID) PIDout = LIMIT_PID;
  if (PIDout < -LIMIT_PID) PIDout = -LIMIT_PID;

  // 4. Motores
  // PID > 0: Virar Direita (Acelera Esquerda, Freia Direita)
  int pwmEsquerda = baseVel_E + PIDout;
  int pwmDireita  = baseVel_D - PIDout ;

  pwmEsquerda = constrain(pwmEsquerda, MIN_SPEED, MAX_PWM);
  pwmDireita  = constrain(pwmDireita,  MIN_SPEED, MAX_PWM);

if(PIDout > 0)
{

  analogWrite(motorE_PWM, 250);
  analogWrite(motorD_PWM, pwmDireita);

}
else
{
  analogWrite(motorE_PWM, pwmEsquerda);
  analogWrite(motorD_PWM, 245);

}
  

  Serial.print("E:"); Serial.print(sensorE);
  Serial.print(" | D:"); Serial.print(sensorD);
  Serial.print(" | Err:"); Serial.print(erro);
  Serial.print(" | PID:"); Serial.print(PIDout);
  Serial.print(" | ME:"); Serial.print(pwmEsquerda);
  Serial.print(" | MD:"); Serial.println(pwmDireita);
  
  delay(5);
}
