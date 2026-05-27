# Carrinho Seguidor de Linha — Controle PID
### Grupo 7 · Eng. da Computação · 2026

Projeto de um robô seguidor de linha com controlador PID embarcado no Arduino UNO.
O sistema detecta a posição de uma linha preta por meio de 4 sensores IR e ajusta
a velocidade dos motores em tempo real para manter o carrinho sobre a pista.

---

## Estrutura do repositório

```
├── arduino/
│   └── pid_seguidor_linha_v3.ino       # Código embarcado — versão final
│
├── matlab/
│   ├── pid_grupo7_v3_final.m           # Validação teórica do PID (planta, malha fechada, Bode, Routh)
│   ├── comprovacao_eficiencia_pid.m    # Simulação Arduino v3 — 4 cenários (reta, curva, S, perturbação)
│   └── comparativo_sem_pid_vs_pid.m   # Comparativo: sem controle vs PID v3
│
└── README.md
```

---

## Hardware

| Componente | Modelo | Função |
|---|---|---|
| Microcontrolador | Arduino UNO | Executa o PID a 100 Hz (Ts = 10 ms) |
| Driver de motor | L298N Dual H-Bridge | Converte PWM em tensão para os motores |
| Sensor de linha | IR 4 canais (LM339) | Detecta posição lateral da linha preta |
| Motores | 2× Motor DC TT | Tração diferencial |
| Bateria lógica | 6V (jack) | Alimenta Arduino e sensores |
| Bateria motores | 9V dedicada | Alimenta L298N |

### Dimensões físicas

| Parâmetro | Valor |
|---|---|
| Largura da placa | 120 mm |
| Comprimento da placa | 185 mm |
| Massa total | 600 g |
| Momento de inércia J | 0,002431 kg·m² |
| Espaçamento entre sensores | 10 mm |

### Pinagem

**Sensores IR → Arduino**

| Sensor | Pino | Peso PID | Posição |
|---|---|---|---|
| S0 (mais à direita) | D11 | +3 | −33 mm |
| S1 | D7 | +1 | −11 mm |
| S2 | D8 | −1 | +11 mm |
| S3 (mais à esquerda) | D12 | −3 | +33 mm |

**Arduino → L298N**

| Pino Arduino | L298N | Função |
|---|---|---|
| D2 | IN1 | Direção Motor Direito |
| D3 | IN2 | Direção Motor Direito |
| D5 (PWM) | ENA | Velocidade Motor Direito |
| D6 | IN3 | Direção Motor Esquerdo |
| D9 | IN4 | Direção Motor Esquerdo |
| D10 (PWM) | ENB | Velocidade Motor Esquerdo |

---

## Parâmetros PID — versão final (v3)

| Parâmetro | Valor | Descrição |
|---|---|---|
| Kp | 60,0 | Força da correção proporcional |
| Ki | 0,0 | Desativado — integrador natural da planta elimina erro estacionário |
| Kd | 3,0 | Amortecimento das oscilações |
| Ts | 10 ms | Período de amostragem |
| ZONA_MORTA | 0,3 | Erros menores que isso são ignorados (elimina tremida na reta) |
| LIMIAR_CURVA_S | 0,8 | A partir daqui o erro é amplificado |
| AMP_CURVA | 3,5 | Fator de amplificação em curvas fechadas |
| SAIDA_LIM | 200 | Limite máximo da saída do PID (constrain) |
| VEL_RETA | 255 PWM | Velocidade em reta |
| VEL_CURVA_SUAVE | 130 PWM | Velocidade em curva suave |
| VEL_CURVA_S | 50 PWM | Velocidade em curva fechada / S |

---

## Como funciona

### Leitura dos sensores

Cada sensor retorna `0` ou `1`. A posição lateral da linha é calculada
por média ponderada:

```
e = Σ(Sᵢ · Wᵢ) / Σ(Sᵢ)       pesos: {+3, +1, -1, -3}
```

O erro varia de −3 (linha totalmente à esquerda) a +3 (totalmente à direita).
Zero significa linha centrada.

### Algoritmo PID (Euler progressivo)

```
// Zona morta + amplificação
if |e| < 0.3  →  e = 0               (elimina tremida)
if |e| >= 0.8 →  e = e × 3.5        (amplifica curva)

// PID
P = Kp × e
integral += e × Ts
integral = clamp(integral, -30, +30) // anti-windup
D = Kd × (e - e_anterior) / Ts      // Euler progressivo
u = P + I + D
u = clamp(u, -200, +200)            // saturação

// Velocidade adaptativa
vel_base = 255  se |e| < 0.5        (reta)
vel_base = 130  se |e| < 1.5        (curva suave)
vel_base =  50  se |e| >= 1.5       (curva fechada / S)

// Motores
PWM_dir = clamp(vel_base - u, 0, 255)
PWM_esq = clamp(vel_base + u, 0, 255)
```

### Busca autônoma (linha perdida)

Se nenhum sensor detectar a linha por mais de **100 ms**, o carrinho
gira no lugar usando o sinal do último erro válido para reencontrar a pista.

---

## Modelagem matemática

**Planta (2ª Lei de Newton rotacional):**
```
J·θ''(t) + b·θ'(t) = u(t)

G(s) = 1 / (J·s² + b·s)
```

**Controlador PID com filtro derivativo (N = 100):**
```
C(s) = Kp + Ki/s + Kd·s·N/(s+N)
```

**Malha fechada:**
```
H(s) = (Kd·s² + Kp·s + Ki) / (J·s³ + (b+Kd)·s² + Kp·s + Ki)
```

**Critério de Routh-Hurwitz:**
```
(b + Kd) · Kp > J · Ki
(0,05 + 3) · 60 = 183,0  >  0,002431 · 0 = 0  ✓  ESTÁVEL
```

**Discretização — Euler progressivo:**
```
a0 =  Kp + Kd/Ts        =  360
a1 = -Kp - 2·Kd/Ts      = -660
a2 =  Kd/Ts             =  300
```

---

## Scripts MATLAB

### `pid_grupo7_v3_final.m`
Validação teórica completa do sistema. Gera 6 gráficos:
- Resposta ao degrau (contínuo)
- Contínuo vs discreto Euler
- Euler vs Tustin (comparação histórica)
- Lugar das raízes
- Diagrama de Bode
- Velocidade adaptativa + zona morta

### `comprovacao_eficiencia_pid.m`
Simulação do Arduino v3 replicando **exatamente** o código embarcado
(zona morta, amplificação, constrain, velocidade adaptativa, Euler).
Testa 4 cenários: reta, curva suave, curva em S e perturbação repentina.
Compara com a versão anterior (v1, Kp=15) em cada cenário.

### `comparativo_sem_pid_vs_pid.m`
Comparativo direto entre sistema sem controle (planta aberta) e com PID v3.
Mostra que sem o PID o sistema diverge — o robô sai da pista.

---

## Evolução do projeto

| Versão | Problema | Solução |
|---|---|---|
| v1 | `setup()` duplicado, Kp=15, Tustin instável, motores invertidos | — |
| v2 | PID simples, `setup()` corrigido, velocidade adaptativa básica | — |
| v3 | **Versão final** — zona morta, amplificação ×3,5, busca por giro, Euler | ✓ |

### Principais problemas resolvidos

- **Zigue-zague caótico** → potenciômetro do S0 recalibrado + Tustin substituído por Euler
- **Não virava na curva** → AMP_CURVA = 3,5 + redução de VEL_CURVA_S para 50
- **Tremida na reta** → zona morta ±0,3 adicionada
- **Perdia linha na curva S** → busca por giro após 100 ms + velocidade adaptativa
- **Motores sem força** → Kp: 15 → 60, SAIDA_LIM: 120 → 200

---

## Guia de tuning rápido

| Sintoma | Solução |
|---|---|
| Tremida na reta | Aumentar `ZONA_MORTA` de 0,3 para 0,5 |
| Não vira na curva | Aumentar `AMP_CURVA` ou `Kp` |
| Sai da linha na curva S | Reduzir `VEL_CURVA_S` |
| Lento demais na reta | Aumentar `VEL_RETA` (máx 255) |
| Deriva em reta longa | Ativar `Ki = 0,1` |
| Não retorna após perder linha | Reduzir tempo de busca de 100 ms para 80 ms |

---

## Resultados

| Métrica | Valor |
|---|---|
| Erro estacionário | 0,000000 |
| Assentamento (modelo) | 0,186 s |
| Overshoot (modelo teórico) | 77,1%* |
| Estabilidade | ESTÁVEL — Routh ✓ |
| Margem de fase | 9,25° |
| Resultado prático | Pista completa (retas + curvas + S) |

*Overshoot do modelo linear contínuo sem saturação.
Na prática é suprimido pelo `constrain(u, ±200)` e pela zona morta.

---

*Disciplina de Sistemas de Controle · 2026*
