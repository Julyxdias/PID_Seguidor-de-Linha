// ============================================================
//  PID Seguidor de Linha — Grupo 7 | Karolaine | 2026-04-09
//  Hardware: Arduino UNO + L298N + 2x Motor DC
//            + Sensor IR 4 canais (LM339)
//  Controle PID SIMPLES — VERSÃO 3
//  Novidades: zona morta + amplificação em curva + vel. adaptativa
// ============================================================

// ─────────────────────────────────────────────
//  PINAGEM — Sensores IR
//
//  pino 11 → S0 (mais à DIREITA)   peso +3
//  pino 7  → S1                    peso +1
//  pino 8  → S2                    peso -1
//  pino 12 → S3 (mais à ESQUERDA)  peso -3
//  VCC → 5V  |  GND → GND comum
// ─────────────────────────────────────────────
const int NUM_SENSORES = 4;
const int PINOS_SENSOR[NUM_SENSORES] = {11, 7, 8, 12};
const float PESOS[NUM_SENSORES]      = {3.0, 1.0, -1.0, -3.0};

// ─────────────────────────────────────────────
//  PINAGEM — L298N
//
//  Motor DIREITO:   IN1=2 | IN2=3 | ENA=5  (PWM ~)
//  Motor ESQUERDO:  IN3=6 | IN4=9 | ENB=10 (PWM ~)
// ─────────────────────────────────────────────
const int IN1 = 2;
const int IN2 = 3;
const int ENA = 5;
const int IN3 = 6;
const int IN4 = 9;
const int ENB = 10;

// ─────────────────────────────────────────────
//  CONFIGURAÇÃO DOS SENSORES
//  true  → LED APAGADO sobre a linha preta (seu caso ✓)
//  false → LED ACESO sobre a linha preta
// ─────────────────────────────────────────────
const bool LINHA_ATIVA_EM_LOW = true;

// ─────────────────────────────────────────────
//  PARÂMETROS PID
//
//  Kp → força da correção
//  Kd → amorte as oscilações (tremida na reta → reduz)
//  Ki → deixe 0.0 até tudo estar estável
// ─────────────────────────────────────────────
float Kp = 60.0;
float Ki = 0.0;
float Kd = 3.0;

// ─────────────────────────────────────────────
//  PARÂMETROS DE CONTROLE
// ─────────────────────────────────────────────
const int   PWM_MAX   = 255;
const int   PWM_MIN   = 0;
const float SAIDA_LIM = 200.0;
const long  TS_MS     = 10;
const float Ts        = TS_MS / 1000.0;

// ─────────────────────────────────────────────
//  VELOCIDADE ADAPTATIVA
//  Sobe/desce esses valores para ajustar velocidade
//
//  VEL_RETA       → velocidade na reta (máximo)
//  VEL_CURVA_SUAVE → velocidade em curva suave
//  VEL_CURVA_S    → velocidade em curva fechada/S
// ─────────────────────────────────────────────
const int VEL_RETA        = 255;  // reta — máximo
const int VEL_CURVA_SUAVE = 130;  // curva suave
const int VEL_CURVA_S     = 50;  // curva fechada/S

// ─────────────────────────────────────────────
//  ZONA MORTA E AMPLIFICAÇÃO DE CURVA
//
//  ZONA_MORTA     → erros menores que isso são ignorados (evita tremida)
//  LIMIAR_CURVA_S → erros maiores que isso são amplificados (melhora curva)
//  AMP_CURVA      → fator de amplificação em curva fechada
// ─────────────────────────────────────────────
const float ZONA_MORTA     = 0.3;
const float LIMIAR_CURVA_S = 0.8;
const float AMP_CURVA      = 3.5;

// ─────────────────────────────────────────────
//  SENTIDO DOS MOTORES
//  Se um motor girar ao contrário: troque 1 por -1
// ─────────────────────────────────────────────
const int DIR_DIR = 1;
const int DIR_ESQ = 1;

// ─────────────────────────────────────────────
//  VARIÁVEIS DE ESTADO
// ─────────────────────────────────────────────
float erro_anterior      = 0.0;
float integral           = 0.0;
float ultimo_erro_valido = 0.0;
bool  linha_perdida      = false;
unsigned long tempo_linha_perdida = 0;  
unsigned long ultimo_tempo = 0;

// ─────────────────────────────────────────────
//  LEITURA DOS SENSORES — média ponderada
// ─────────────────────────────────────────────
float ler_sensores() {
  float soma_ponderada = 0.0;
  float soma_ativos    = 0.0;
  int   leituras[NUM_SENSORES];

  for (int i = 0; i < NUM_SENSORES; i++) {
    int val = digitalRead(PINOS_SENSOR[i]);
    bool sobre_linha = LINHA_ATIVA_EM_LOW ? (val == LOW) : (val == HIGH);
    leituras[i]     = sobre_linha ? 1 : 0;
    soma_ponderada += leituras[i] * PESOS[i];
    soma_ativos    += leituras[i];
  }

  // Debug Serial
  Serial.print("[");
  for (int i = 0; i < NUM_SENSORES; i++) {
    Serial.print(leituras[i]);
    if (i < NUM_SENSORES - 1) Serial.print("|");
  }
  Serial.print("] ");

  // Linha perdida — usa último erro amplificado para busca
  if (soma_ativos == 0.0) {
    linha_perdida = true;
    return ultimo_erro_valido * 3.5;
  }

  linha_perdida = false;
  float erro = soma_ponderada / soma_ativos;
  ultimo_erro_valido = erro;
  return erro;
}

// ─────────────────────────────────────────────
//  CONTROLE DOS MOTORES
// ─────────────────────────────────────────────
void motor_direito(int vel) {
  vel = constrain(vel, -PWM_MAX, PWM_MAX);
  if (vel >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
  else          { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); vel = -vel; }
  analogWrite(ENA, vel);
}

void motor_esquerdo(int vel) {
  vel = constrain(vel, -PWM_MAX, PWM_MAX);
  if (vel >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
  else          { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); vel = -vel; }
  analogWrite(ENB, vel);
}

void parar_motores() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  for (int i = 0; i < NUM_SENSORES; i++) {
    pinMode(PINOS_SENSOR[i], INPUT);
  }

  parar_motores();

  Serial.begin(9600);
  Serial.println("=== PID Seguidor de Linha — Grupo 7 v3 ===");
  Serial.print("Kp="); Serial.print(Kp);
  Serial.print(" Ki="); Serial.print(Ki);
  Serial.print(" Kd="); Serial.println(Kd);
  Serial.print("Reta="); Serial.print(VEL_RETA);
  Serial.print(" CurvaSuave="); Serial.print(VEL_CURVA_SUAVE);
  Serial.print(" CurvaS="); Serial.println(VEL_CURVA_S);
  Serial.println("Iniciando em 3s...");
  delay(3000);

  erro_anterior      = 0.0;
  integral           = 0.0;
  ultimo_erro_valido = 0.0;
  linha_perdida      = false;
  ultimo_tempo       = millis();
}

// ─────────────────────────────────────────────
//  LOOP PRINCIPAL
// ─────────────────────────────────────────────
void loop() {
  unsigned long agora = millis();
  if (agora - ultimo_tempo < (unsigned long)TS_MS) return;
  ultimo_tempo = agora;

  float e = ler_sensores();

  // Conta quanto tempo está sem linha
  if (linha_perdida) {
    tempo_linha_perdida += TS_MS;
  } else {
    tempo_linha_perdida = 0;
  }

  // Perdeu linha por mais de 200ms → para e gira no lugar
  if (tempo_linha_perdida > 100) {
    int sentido = (ultimo_erro_valido > 0) ? 1 : -1;
    motor_direito (DIR_DIR *  sentido * 160);
    motor_esquerdo(DIR_ESQ * -sentido * 160);
    Serial.println(">>> BUSCANDO LINHA <<<");
    return;
  }

  // Zona morta
  if (abs(e) < ZONA_MORTA) {
    e = 0.0;
  } else if (abs(e) >= LIMIAR_CURVA_S) {
    e = e * AMP_CURVA;
  }

  // PID
  float P = Kp * e;
  integral += e * Ts;
  integral = constrain(integral, -30.0, 30.0);
  float I = Ki * integral;
  float D = Kd * (e - erro_anterior) / Ts;
  erro_anterior = e;

  float u = P + I + D;
  u = constrain(u, -SAIDA_LIM, SAIDA_LIM);

  // Velocidade adaptativa
  float erro_abs = abs(e);
  int vel_base;
  if (erro_abs < 0.5) {
    vel_base = VEL_RETA;
  } else if (erro_abs < 1.5) {
    vel_base = VEL_CURVA_SUAVE;
  } else {
    vel_base = VEL_CURVA_S;
  }

  int vel_dir = constrain((int)(vel_base - u), PWM_MIN, PWM_MAX);
  int vel_esq = constrain((int)(vel_base + u), PWM_MIN, PWM_MAX);

  motor_direito (DIR_DIR * vel_dir);
  motor_esquerdo(DIR_ESQ * vel_esq);

  Serial.print("e=");   Serial.print(e,      2);
  Serial.print(" u=");  Serial.print(u,       1);
  Serial.print(" vB="); Serial.print(vel_base);
  Serial.print(" vD="); Serial.print(vel_dir);
  Serial.print(" vE="); Serial.print(vel_esq);
  if (linha_perdida) Serial.print(" *** LINHA PERDIDA ***");
  Serial.println();
}
// ─────────────────────────────────────────────
//  GUIA DE TUNING RÁPIDO
//
//  TREMIDA NA RETA:
//    Aumenta ZONA_MORTA de 0.3 para 0.5
//    Reduz Kd de 3 para 2 ou 1
//
//  NÃO VIRA BEM NA CURVA:
//    Aumenta AMP_CURVA de 1.5 para 2.0
//    Reduz VEL_CURVA_S de 140 para 120
//    Aumenta Kp de 60 para 70 ou 80
//
//  SAI DA LINHA NA CURVA S:
//    Reduz VEL_CURVA_S de 140 para 110
//    Aumenta AMP_CURVA de 1.5 para 2.0
//
//  QUER MAIS VELOCIDADE NA RETA:
//    Sobe VEL_RETA (já está em 255 = máximo)
//    Para mais velocidade real: use bateria 12V no L298N
//
//  DERIVA EM RETA LONGA:
//    Ki = 0.1 (só após tudo estar estável)
// ─────────────────────────────────────────────