// ============================================================
//  PID Seguidor de Linha — Grupo 7 | Karolaine | 2026-04-09
//  Hardware: Arduino Uno + L298N + 2x Motor DC
//            + Sensor IR 4 canais
//  Controle PID com discretização Tustin (Ts = 10ms)
// ============================================================

// ─────────────────────────────────────────────
//  PINAGEM — Sensores IR (4 canais digitais)
//
//  OUT1 → D8   (S0 — mais à esquerda)
//  OUT2 → D9   (S1)
//  OUT3 → D10  (S2)
//  OUT4 → D11  (S3 — mais à direita)
//  VCC  → 5V   |   GND → GND
// ─────────────────────────────────────────────
const int NUM_SENSORES = 4;
const int PINOS_SENSOR[NUM_SENSORES] = {0, 8, 9, 10};
// D8  = sensor mais à esquerda  (S0)
// D11 = sensor mais à direita   (S3)

// Pesos para a média ponderada
// S0=-3, S1=-1, S2=+1, S3=+3  → erro de -3 a +3
const float PESOS[NUM_SENSORES] = {-3.0, -1.0, 1.0, 3.0};

// ─────────────────────────────────────────────
//  PINAGEM — L298N (U3)
//
//  Motor A (esquerdo):  IN1=D1 | IN2=D2 | ENA=D5
//  Motor B (direito):   IN3=D3 | IN4=D4 | ENB=D6
//  +5V → 5V Arduino    |   GND → GND
// ─────────────────────────────────────────────
const int IN1 = 1;    // Motor esquerdo — direção A
const int IN2 = 2;    // Motor esquerdo — direção B
const int ENA = 5;    // Motor esquerdo — PWM (Enable A)
const int IN3 = 3;    // Motor direito  — direção A
const int IN4 = 4;    // Motor direito  — direção B
const int ENB = 6;    // Motor direito  — PWM (Enable B)

// ─────────────────────────────────────────────
//  THRESHOLD do sensor digital
//  FC-51 já entrega sinal digital (0 ou 1).
//  Se o seu módulo entrega analógico, mantenha o threshold.
//  Se for puramente digital (LOW/HIGH), use digitalRead().
// ─────────────────────────────────────────────
const bool SENSOR_DIGITAL = true;   // true = usa digitalRead()
                                     // false = usa analogRead() + threshold
const int  THRESHOLD      = 500;    // usado apenas se SENSOR_DIGITAL = false

// Lógica de detecção:
// LINHA_ATIVA_EM_LOW = true  → sensor retorna LOW quando está sobre a linha
// LINHA_ATIVA_EM_LOW = false → sensor retorna HIGH quando está sobre a linha
const bool LINHA_ATIVA_EM_LOW = true;

// ─────────────────────────────────────────────
//  PARÂMETROS PID
// ─────────────────────────────────────────────
float Kp = 40.0;
float Ki =  0.5;
float Kd = 10.0;

// ─────────────────────────────────────────────
//  PARÂMETROS DE CONTROLE
// ─────────────────────────────────────────────
const int   VELOCIDADE_BASE = 160;    // PWM base dos motores (0–255)
const int   PWM_MAX         = 255;
const int   PWM_MIN         = 0;
const float SAIDA_LIM       = 200.0;  // anti-windup: limita saída Tustin
const long  TS_MS           = 10;     // período de amostragem [ms]
const int DIR_ESQ =  1;  // troca para -1 para inverter
const int DIR_DIR = -1;  // troca para  1 para inverter

// ─────────────────────────────────────────────
//  VARIÁVEIS DE ESTADO — Tustin
// ─────────────────────────────────────────────
const float Ts = TS_MS / 1000.0;

float a0, a1, a2;     // coeficientes da equação de diferenças
float u_k   = 0.0;    // saída PID atual
float u_km1 = 0.0;    // saída PID no instante anterior
float e_k   = 0.0;    // erro atual
float e_km1 = 0.0;    // erro em k-1
float e_km2 = 0.0;    // erro em k-2

float ultimo_erro_valido = 0.0;   // para fallback (linha perdida)
bool  linha_perdida      = false;

unsigned long ultimo_tempo = 0;

// ─────────────────────────────────────────────
//  LEITURA DOS 4 SENSORES — média ponderada
//
//  Retorna erro lateral:
//    0.0  = linha centrada
//    < 0  = linha à esquerda  → vira para esquerda
//    > 0  = linha à direita   → vira para direita
// ─────────────────────────────────────────────
float ler_sensores() {
  float soma_ponderada = 0.0;
  float soma_ativos    = 0.0;
  int   leituras[NUM_SENSORES];

  // Lê todos os canais
  for (int i = 0; i < NUM_SENSORES; i++) {
    bool sobre_linha;

    if (SENSOR_DIGITAL) {
      int val = digitalRead(PINOS_SENSOR[i]);
      sobre_linha = LINHA_ATIVA_EM_LOW ? (val == LOW) : (val == HIGH);
    } else {
      int val = analogRead(PINOS_SENSOR[i]);
      sobre_linha = LINHA_ATIVA_EM_LOW ? (val < THRESHOLD) : (val >= THRESHOLD);
    }

    leituras[i] = sobre_linha ? 1 : 0;
    soma_ponderada += leituras[i] * PESOS[i];
    soma_ativos    += leituras[i];
  }

  // Debug dos sensores individuais
  Serial.print("[");
  for (int i = 0; i < NUM_SENSORES; i++) {
    Serial.print(leituras[i]);
    if (i < NUM_SENSORES - 1) Serial.print("|");
  }
  Serial.print("] ");

  // Nenhum sensor ativo → linha perdida → fallback
  if (soma_ativos == 0.0) {
    linha_perdida = true;
    // Usa dobro do último erro conhecido para redirecionar o robô
    return ultimo_erro_valido * 2.0;
  }

  linha_perdida = false;
  float erro = soma_ponderada / soma_ativos;
  ultimo_erro_valido = erro;
  return erro;
}

// ─────────────────────────────────────────────
//  CONTROLE DOS MOTORES
// ─────────────────────────────────────────────
void motor_esquerdo(int vel) {
  vel = constrain(vel, -PWM_MAX, PWM_MAX);
  if (vel >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
  else          { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); vel = -vel; }
  analogWrite(ENA, vel);
}

void motor_direito(int vel) {
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
  // Pinos de motor
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  // Pinos dos sensores
  for (int i = 0; i < NUM_SENSORES; i++) {
    if (SENSOR_DIGITAL) pinMode(PINOS_SENSOR[i], INPUT);
    // analogRead não precisa de pinMode, mas não prejudica
  }

  // Coeficientes Tustin:
  // C(s) = Kp + Ki/s + Kd·s  →  bilinear s = 2(z-1)/(Ts(z+1))
  a0 =  Kp + Ki * Ts / 2.0 + 2.0 * Kd / Ts;
  a1 = -Kp + Ki * Ts / 2.0 - 4.0 * Kd / Ts;
  a2 =  2.0 * Kd / Ts;

  parar_motores();

  Serial.begin(9600);
  Serial.println("=== PID Seguidor de Linha 4CH — Grupo 7 ===");
  Serial.print("Kp="); Serial.print(Kp);
  Serial.print("  Ki="); Serial.print(Ki);
  Serial.print("  Kd="); Serial.println(Kd);
  Serial.print("Tustin  a0="); Serial.print(a0);
  Serial.print("  a1="); Serial.print(a1);
  Serial.print("  a2="); Serial.println(a2);
  Serial.println("Sensores: [S0(esq) | S1 | S2 | S3(dir)]");
  Serial.println("Iniciando em 2s...");
  delay(2000);

  ultimo_tempo = millis();
}

// ─────────────────────────────────────────────
//  LOOP PRINCIPAL — executa a cada Ts = 10 ms
// ─────────────────────────────────────────────
void loop() {
  unsigned long agora = millis();
  if (agora - ultimo_tempo < (unsigned long)TS_MS) return;
  ultimo_tempo = agora;

  // 1. Lê os 4 sensores → erro lateral por média ponderada
  e_k = ler_sensores();

  // 2. PID discreto (Tustin): u[k] = u[k-1] + a0·e[k] + a1·e[k-1] + a2·e[k-2]
  u_k = u_km1 + a0 * e_k + a1 * e_km1 + a2 * e_km2;

  // 3. Anti-windup por clamping da saída
  u_k = constrain(u_k, -SAIDA_LIM, SAIDA_LIM);

  // 4. Velocidades individuais dos motores
  //    Erro > 0 (linha à direita) → aumenta motor esq, reduz motor dir
  int vel_esq = constrain((int)(VELOCIDADE_BASE + u_k), PWM_MIN, PWM_MAX);
  int vel_dir = constrain((int)(VELOCIDADE_BASE - u_k), PWM_MIN, PWM_MAX);

  motor_esquerdo(DIR_ESQ * vel_esq);
  motor_direito (DIR_DIR * vel_dir);

  // 5. Atualiza histórico
  u_km1 = u_k;
  e_km2 = e_km1;
  e_km1 = e_k;

  // 6. Serial Monitor para tuning
  Serial.print("e=");  Serial.print(e_k,  2);
  Serial.print(" u="); Serial.print(u_k,  1);
  Serial.print(" L="); Serial.print(vel_esq);
  Serial.print(" R="); Serial.print(vel_dir);
  if (linha_perdida) Serial.print(" *** LINHA PERDIDA ***");
  Serial.println();
}

// ─────────────────────────────────────────────
//  GUIA DE TUNING — passo a passo
//
//  1. Comece: Kp=20, Ki=0, Kd=0
//     → Robô segue a linha mas oscila?  Reduza Kp.
//     → Robô segue devagar/lento?       Aumente Kp.
//
//  2. Adicione Kd (ex: Kd=5) para reduzir oscilações.
//     Aumente até suavizar sem deixar lento.
//
//  3. Adicione Ki (ex: Ki=0.2) só se houver erro residual
//     em retas longas. Ki pequeno — evite windup.
//
//  4. Se a linha for preta sobre branco:
//     LINHA_ATIVA_EM_LOW = true  (FC-51 padrão)
//     Se a linha for branca sobre preto:
//     LINHA_ATIVA_EM_LOW = false
// ─────────────────────────────────────────────
