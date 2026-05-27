%% =========================================================================
%  MODELAGEM MATEMATICA — CARRINHO SEGUIDOR DE LINHA COM PID
%  Hardware: Raspberry Pi Pico + Sensor IR 8ch + L298N + 2x Motor TT DC
%
%  Compativel com MATLAB R2014b ou superior (Control System Toolbox padrao).
%
%  Equacoes:
%    Planta:      G(s) = 1 / (J*s^2 + b*s)        [2a lei de Newton rotacional]
%    Controlador: C(s) = Kp + Ki/s + Kd*s          [PID ideal]
%    Malha fechada:
%      H(s) = C(s)*G(s) / [1 + C(s)*G(s)]
%           = (Kd*s^2 + Kp*s + Ki) / (J*s^3 + (b+Kd)*s^2 + Kp*s + Ki)
%
%  Estabilidade (Routh-Hurwitz):
%    Condicoes: Kp>0, Ki>0, Kd>0, (b+Kd)*Kp > J*Ki
% =========================================================================

clear; close all; clc;

%% ── 1. PARAMETROS FISICOS DO ROBO ─────────────────────────────────────────
%
%  Medicoes reais do carrinho:
%    Peso total      : 600 g   = 0.600 kg
%    Largura         : 18.5 cm = 0.185 m
%    Comprimento     : 20.4 cm = 0.204 m
%    Dist. entre rodas: 13.3 cm = 0.133 m
%
%  Momento de inercia — chassi retangular girando no eixo vertical central:
%    J = (1/12) * M * (L_r^2 + W_r^2)
%
%    L_r = distancia entre rodas (governa a dinamica de rotacao do carrinho)
%    W_r = comprimento do chassi (frente a tras)
%
%    J = (1/12) * 0.600 * (0.133^2 + 0.204^2) = 0.002965 kg*m^2
%
%  Coeficiente de atrito viscoso b:
%    Estimativa inicial conservadora para motores TT com caixa de reducao.
%    Para refinar: coloque o carrinho em superficie plana, aplique PWM fixo,
%    meça a velocidade angular de regime (rad/s) e calcule:
%      b = (ganho_torque * PWM_duty) / omega_regime
%    Tipicamente entre 0.03 e 0.10 N*m*s para motores TT.

M   = 0.600;    % massa total [kg]              — 600 g
L_r = 0.133;    % distancia entre rodas [m]     — 13.3 cm
W_r = 0.204;    % comprimento do chassi [m]     — 20.4 cm

% Momento de inercia calculado
J = (1/12) * M * (L_r^2 + W_r^2);   % = 0.002965 kg*m^2

% Coeficiente de atrito viscoso (estimativa inicial)
b = 0.05;       % [N*m*s]  — ajuste experimentalmente se necessario

fprintf('=== Parametros fisicos (medidos) ===\n');
fprintf('M   = %.4f kg   (600 g)\n',   M);
fprintf('L_r = %.4f m    (13.3 cm — distancia entre rodas)\n', L_r);
fprintf('W_r = %.4f m    (20.4 cm — comprimento do chassi)\n', W_r);
fprintf('J   = %.6f kg*m^2\n', J);
fprintf('b   = %.4f N*m*s  (estimativa; refine experimentalmente)\n\n', b);

%% ── 2. FUNCAO DE TRANSFERENCIA DA PLANTA ──────────────────────────────────
%
%  Derivacao:
%    J*d2theta/dt2 + b*dtheta/dt = tau(t)
%    Laplace -> (J*s^2 + b*s)*Theta(s) = U(s)
%    G(s) = 1 / (J*s^2 + b*s) = 1 / [s*(J*s + b)]
%
%  Polo em s=0 = integrador natural:
%    Sem realimentacao, qualquer torque constante gera rotacao infinita.

planta = tf(1, [J, b, 0]);

fprintf('=== Planta G(s) ===\n');
display(planta);

p_planta = pole(planta);
p_nao_zero = p_planta(abs(p_planta) > 1e-10);
fprintf('Polo 1: s =  0       (integrador natural — diverge sem controle)\n');
fprintf('Polo 2: s = %.4f  (polo de atrito: -b/J = -%.4f)\n\n', real(p_nao_zero(1)), b/J);

%% ── 3. GANHOS PID ─────────────────────────────────────────────────────────

Kp = 1.5;
Ki = 0.20;
Kd = 0.80;

% Verificacao de Routh-Hurwitz para o polinomio: J*s^3 + (b+Kd)*s^2 + Kp*s + Ki
val_lhs = (b + Kd) * Kp;
val_rhs = J * Ki;

fprintf('=== Verificacao de estabilidade (Routh-Hurwitz) ===\n');
fprintf('Condicao: (b+Kd)*Kp > J*Ki\n');
fprintf('(b+Kd)*Kp = %.6f\n', val_lhs);
fprintf('J*Ki       = %.6f\n', val_rhs);
if val_lhs > val_rhs
    fprintf('Condicao satisfeita -> sistema ESTAVEL para estes ganhos.\n\n');
else
    fprintf('[ATENCAO] Sistema INSTAVEL! Reduza Ki ou aumente Kd.\n\n');
end

%% ── 4. CONTROLADOR PID E MALHA FECHADA ────────────────────────────────────
%
%  C(s) = (Kd*s^2 + Kp*s + Ki) / s
%
%  H(s) = C(s)*G(s) / [1 + C(s)*G(s)]
%       = (Kd*s^2 + Kp*s + Ki) / (J*s^3 + (b+Kd)*s^2 + Kp*s + Ki)

controlador  = tf([Kd, Kp, Ki], [1, 0]);
malha_aberta = controlador * planta;
sys_MF       = feedback(malha_aberta, 1);   % realimentacao negativa unitaria

fprintf('=== Controlador PID C(s) ===\n');
display(controlador);

fprintf('=== Malha aberta C(s)*G(s) ===\n');
display(malha_aberta);

fprintf('=== Malha fechada H(s) ===\n');
display(sys_MF);

% Polos da malha fechada
polos = pole(sys_MF);
fprintf('=== Polos da malha fechada ===\n');
for i = 1:length(polos)
    if abs(imag(polos(i))) < 1e-8
        fprintf('  polo %d: %+.4f           (real)\n', i, real(polos(i)));
    else
        fprintf('  polo %d: %+.4f %+.4fi    (complexo)\n', ...
                i, real(polos(i)), imag(polos(i)));
    end
end

if all(real(polos) < 0)
    fprintf('Todos os polos no semiplano esquerdo -> ESTAVEL.\n\n');
else
    fprintf('[ATENCAO] Polo(s) no semiplano direito -> INSTAVEL!\n\n');
end

% Valor de regime e erro estacionario
% Usa dcgain() — compativel com todas as versoes do MATLAB
% Equivale ao teorema do valor final: lim[s->0] s*H(s)*(1/s) = H(0)
val_regime        = dcgain(sys_MF);
erro_estacionario = abs(1 - val_regime);
fprintf('Valor de regime (dcgain): %.6f\n', val_regime);
fprintf('Erro estacionario       : %.6f\n\n', erro_estacionario);

%% ── 5. RESPOSTA AO DEGRAU ─────────────────────────────────────────────────

t = 0:0.001:5;

figure('Name', 'Analise PID — Seguidor de Linha', ...
       'NumberTitle', 'off', 'Position', [50, 50, 1200, 800]);

% Plot 1: Degrau — PID vs sem controle
subplot(2, 3, [1, 2]);

[y_MA, ~] = step(tf(1, [J, b, 0]), t);
y_MA = min(y_MA, 3);

[y_MF, ~] = step(sys_MF, t);

plot(t, y_MA, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Sem controle (malha aberta)');
hold on;
plot(t, y_MF, 'b-',  'LineWidth', 2.0, ...
     'DisplayName', sprintf('PID: Kp=%.1f  Ki=%.2f  Kd=%.1f', Kp, Ki, Kd));
yline(1, 'k:', 'LineWidth', 1, 'DisplayName', 'Setpoint');
xlabel('Tempo (s)');
ylabel('Posicao normalizada \theta(t)');
title('Resposta ao degrau: PID vs sem controle');
legend('Location', 'northeast');
grid on;
xlim([0, 5]);
ylim([-0.2, 2.5]);

% Plot 2: Lugar das raizes
subplot(2, 3, 3);
rlocus(malha_aberta);
title('Lugar das raizes C(s) G(s)');
grid on;
hold on;
plot(real(polos), imag(polos), 'bx', 'MarkerSize', 12, 'LineWidth', 2);
legend({'Lugar das raizes', 'Polos MF'}, 'Location', 'southeast');

% Plot 3: Bode da malha aberta
subplot(2, 3, 4);
bode(malha_aberta);
title('Bode — malha aberta');
grid on;

% Plot 4: Nyquist
subplot(2, 3, 5);
nyquist(malha_aberta);
title('Nyquist — malha aberta');
grid on;

% Plot 5: Sensibilidade a variacao de Kp
% Cores como vetores RGB [R G B] — compativel com R2014b e superior
subplot(2, 3, 6);
hold on;
kp_values = [0.5, 1.0, 1.5, 2.5, 4.0];
cores_rgb = [0.62 0.79 0.88;
             0.42 0.68 0.84;
             0.19 0.51 0.74;
             0.03 0.32 0.61;
             0.89 0.29 0.29];
for i = 1:length(kp_values)
    C_var = tf([Kd, kp_values(i), Ki], [1, 0]);
    H_var = feedback(C_var * planta, 1);
    if all(real(pole(H_var)) < 0)
        [y_var, ~] = step(H_var, t);
        plot(t, y_var, 'Color', cores_rgb(i, :), 'LineWidth', 1.5, ...
             'DisplayName', sprintf('Kp = %.1f', kp_values(i)));
    else
        fprintf('Kp = %.1f gera sistema instavel — omitido.\n', kp_values(i));
    end
end
yline(1, 'k:', 'LineWidth', 1, 'HandleVisibility', 'off');
xlabel('Tempo (s)');
ylabel('\theta(t)');
title('Sensibilidade a Kp (Ki e Kd fixos)');
legend('Location', 'northeast', 'FontSize', 8);
grid on;
xlim([0, 3]);
ylim([-0.2, 2.2]);

%% ── 6. METRICAS DE DESEMPENHO ─────────────────────────────────────────────
%
%  CORRECAO DE COMPATIBILIDADE:
%    stepinfo().SteadyStateValue nao existe antes do R2016a.
%    Usamos dcgain() calculado na secao 4 — funciona em todas as versoes.
%
%  SettlingTime retorna NaN quando:
%    (a) o sistema nao assenta dentro do tempo t simulado, ou
%    (b) ha um polo muito rapido que o criterio de 2% nao captura.
%    Nesses casos o aviso abaixo orienta o usuario.

info = stepinfo(sys_MF, 'SettlingTimeThreshold', 0.02);

fprintf('=== Metricas de desempenho (resposta ao degrau) ===\n');
fprintf('Tempo de subida  (10%%->90%%)  : %.4f s\n', info.RiseTime);
fprintf('Tempo de pico                : %.4f s\n', info.PeakTime);
fprintf('Overshoot maximo             : %.2f %%\n', info.Overshoot);

if isnan(info.SettlingTime)
    fprintf('Tempo de assentamento (2%%)  : nao atingido em %.0f s\n', t(end));
    fprintf('  -> Aumente o vetor t (ex: t = 0:0.001:20) ou revise os ganhos.\n');
else
    fprintf('Tempo de assentamento (2%%)  : %.4f s\n', info.SettlingTime);
end

fprintf('Valor de regime  (dcgain)    : %.6f\n', val_regime);
fprintf('Erro estacionario            : %.6f\n\n', erro_estacionario);

%% ── 7. MARGENS DE ESTABILIDADE ────────────────────────────────────────────

[Gm, Pm, Wcg, Wcp] = margin(malha_aberta);
Gm_dB = 20 * log10(Gm);

fprintf('=== Margens de estabilidade ===\n');
if isinf(Gm)
    fprintf('Margem de ganho : Inf dB  (curva de ganho nao cruza 0 dB)\n');
else
    fprintf('Margem de ganho : %.2f dB   (freq. = %.4f rad/s)\n', Gm_dB, Wcg);
end
fprintf('Margem de fase  : %.2f graus (freq. = %.4f rad/s)\n', Pm, Wcp);

ok_ganho = isinf(Gm) || Gm_dB > 6;
ok_fase  = Pm > 45;
if ok_ganho && ok_fase
    fprintf('Margens adequadas para robotica (>6 dB e >45 graus).\n\n');
else
    fprintf('[ATENCAO] Margens insuficientes — sistema sensivel a perturbacoes.\n\n');
end

%% ── 8. DISCRETIZACAO PARA O RASPBERRY PI PICO ─────────────────────────────
%
%  Metodo de Tustin: s <- (2/Ts)*(z-1)/(z+1)
%
%  PID discreto — equacao de diferencas:
%    u[k] = u[k-1] + a0*e[k] + a1*e[k-1] + a2*e[k-2]
%  onde:
%    a0 =  Kp + Ki*Ts/2 + Kd/Ts
%    a1 = -Kp + Ki*Ts/2 - 2*Kd/Ts
%    a2 =  Kd/Ts

Ts = 0.010;   % 10 ms = 100 Hz

sys_MF_disc = c2d(sys_MF, Ts, 'tustin');

a0 =  Kp + Ki*Ts/2 + Kd/Ts;
a1 = -Kp + Ki*Ts/2 - 2*Kd/Ts;
a2 =  Kd/Ts;

fprintf('=== PID discreto (Tustin, Ts = %g ms = %g Hz) ===\n', Ts*1000, 1/Ts);
fprintf('a0 = %+.6f\n', a0);
fprintf('a1 = %+.6f\n', a1);
fprintf('a2 = %+.6f\n', a2);
fprintf('\nEquacao de diferencas implementada no Pico:\n');
fprintf('u[k] = u[k-1] + (%.4f)*e[k] + (%.4f)*e[k-1] + (%.4f)*e[k-2]\n\n', a0, a1, a2);

% Comparacao continuo vs discreto
figure('Name', 'Continuo vs Discreto (Tustin)', 'NumberTitle', 'off');
[y_cont, t_cont] = step(sys_MF, t);
[y_disc, t_disc] = step(sys_MF_disc, 0:Ts:5);

plot(t_cont, y_cont, 'b-', 'LineWidth', 2,   'DisplayName', 'Continuo H(s)');
hold on;
stairs(t_disc, y_disc, 'r--', 'LineWidth', 1.5, ...
       'DisplayName', sprintf('Discreto H(z), Ts=%g ms', Ts*1000));
yline(1, 'k:', 'LineWidth', 1, 'HandleVisibility', 'off');
xlabel('Tempo (s)');
ylabel('\theta(t)');
title(sprintf('Continuo vs Discreto — Tustin (Ts = %g ms)', Ts*1000));
legend('Location', 'northeast');
grid on;

y_cont_ds  = interp1(t_cont, y_cont, t_disc, 'linear', 'extrap');
erro_disc  = max(abs(y_cont_ds - y_disc));
fprintf('Erro maximo continuo vs discreto: %.6f\n', erro_disc);
if erro_disc < 0.02
    fprintf('Discretizacao adequada para Ts = %g ms.\n\n', Ts*1000);
else
    fprintf('[ATENCAO] Erro de discretizacao alto (%.4f). Reduza Ts.\n\n', erro_disc);
end

%% ── 9. RESUMO FINAL ───────────────────────────────────────────────────────

fprintf('============================================================\n');
fprintf(' RESUMO — cole estes valores no MicroPython do Pico\n');
fprintf('============================================================\n');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);
fprintf('Kd = %.4f\n', Kd);
fprintf('\n# PID discreto (Tustin, Ts = %g ms):\n', Ts*1000);
fprintf('# u[k] = u[k-1] + a0*e[k] + a1*e[k-1] + a2*e[k-2]\n');
fprintf('a0 = %.6f\n', a0);
fprintf('a1 = %.6f\n', a1);
fprintf('a2 = %.6f\n', a2);
fprintf('\n# Limite do integrador (anti-windup):\n');
fprintf('INTEGRAL_LIMIT = %.1f\n', 10/Ki);
fprintf('============================================================\n');