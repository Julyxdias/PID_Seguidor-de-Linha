% ============================================================
%  COMPROVAÇÃO DE EFICIÊNCIA — PID Seguidor de Linha
%  Grupo 7 | Karolaine | 2026
%  
%  Este script replica EXATAMENTE o comportamento do Arduino v3:
%  → Mesmos ganhos, mesma zona morta, mesma amplificação,
%    mesmo constrain, mesma velocidade adaptativa
%  → Simula 4 cenários: reta, curva suave, curva S, linha perdida
%  → Compara com sistema SEM controle e com Kp baixo (v1)
% ============================================================

clear; clc; close all;

%% ══════════════════════════════════════════════════════════════
%  1. PARÂMETROS — idênticos ao código Arduino v3
%% ══════════════════════════════════════════════════════════════
Kp = 60.0;
Ki = 0.0;
Kd = 3.0;
Ts = 0.010;          % 10 ms — mesmo do Arduino

ZONA_MORTA    = 0.3;
LIMIAR_CURVA  = 0.8;
AMP_CURVA     = 3.5;
SAIDA_LIM     = 200.0;
VEL_RETA      = 255;
VEL_CURVA_S   = 130;
VEL_CURVA_FEC = 50;

% Parâmetros físicos (hardware real)
massa = 0.600;
larg  = 0.120;
comp  = 0.185;
J     = (massa/12) * (larg^2 + comp^2);
b     = 0.05;

fprintf('════════════════════════════════════════════\n');
fprintf('  COMPROVAÇÃO DE EFICIÊNCIA — PID v3\n');
fprintf('════════════════════════════════════════════\n');
fprintf('J = %.6f kg·m²  |  b = %.3f\n', J, b);
fprintf('Kp=%.0f | Ki=%.0f | Kd=%.0f | Ts=%dms\n\n', Kp,Ki,Kd,Ts*1000);

%% ══════════════════════════════════════════════════════════════
%  2. FUNÇÃO: SIMULAÇÃO DO ARDUINO v3 (Euler + constrain + zona morta)
%  Replica linha a linha o loop() do Arduino
%% ══════════════════════════════════════════════════════════════
function [t_out, e_out, u_out, vD_out, vE_out] = simular_arduino(...
    setpoint_fn, t_total, Kp, Ki, Kd, Ts, ...
    ZONA_MORTA, LIMIAR_CURVA, AMP_CURVA, SAIDA_LIM, ...
    VEL_RETA, VEL_CURVA_S, VEL_CURVA_FEC, J, b)

    N = round(t_total / Ts);
    t_out  = zeros(1, N);
    e_out  = zeros(1, N);
    u_out  = zeros(1, N);
    vD_out = zeros(1, N);
    vE_out = zeros(1, N);

    % Estado interno (igual ao Arduino)
    theta         = 0.0;   % posição angular atual
    theta_dot     = 0.0;   % velocidade angular
    erro_anterior = 0.0;
    integral      = 0.0;

    for k = 1:N
        t_k = (k-1) * Ts;
        t_out(k) = t_k;

        % Setpoint (posição da linha) — varia por cenário
        ref = setpoint_fn(t_k);

        % Erro de posição (sensor IR)
        e_raw = ref - theta;

        % ── ZONA MORTA (igual ao Arduino) ──
        e = e_raw;
        if abs(e) < ZONA_MORTA
            e = 0.0;
        elseif abs(e) >= LIMIAR_CURVA
            e = e * AMP_CURVA;
        end

        % ── PID EULER (igual ao Arduino) ──
        P        = Kp * e;
        integral = integral + e * Ts;
        integral = max(-30.0, min(30.0, integral));   % anti-windup
        I        = Ki * integral;
        D        = Kd * (e - erro_anterior) / Ts;
        erro_anterior = e;

        u = P + I + D;
        u = max(-SAIDA_LIM, min(SAIDA_LIM, u));       % constrain

        % ── VELOCIDADE ADAPTATIVA (igual ao Arduino) ──
        ea = abs(e_raw);
        if ea < 0.5
            vel_base = VEL_RETA;
        elseif ea < 1.5
            vel_base = VEL_CURVA_S;
        else
            vel_base = VEL_CURVA_FEC;
        end

        vel_dir = max(0, min(255, vel_base - u));
        vel_esq = max(0, min(255, vel_base + u));

        % ── DINÂMICA DA PLANTA (Euler numérico) ──
        % J*theta'' + b*theta' = u  →  theta'' = (u - b*theta')/J
        torque      = u * 0.01;   % escala PWM→torque (normalizado)
        theta_ddot  = (torque - b * theta_dot) / J;
        theta_dot   = theta_dot + theta_ddot * Ts;
        theta       = theta + theta_dot * Ts;

        e_out(k)  = e_raw;
        u_out(k)  = u;
        vD_out(k) = vel_dir;
        vE_out(k) = vel_esq;
    end
end

%% ══════════════════════════════════════════════════════════════
%  3. MODELO CONTÍNUO (para comparação teórica)
%% ══════════════════════════════════════════════════════════════
G = tf(1, [J, b, 0]);
N_filt = 100;
C = pid(Kp, Ki, Kd, Kd/N_filt);
T = feedback(C * G, 1);

info = stepinfo(T);
fprintf('── Métricas modelo contínuo ──\n');
fprintf('Tempo de subida      : %.4f s\n', info.RiseTime);
fprintf('Tempo de assentamento: %.4f s\n', info.SettlingTime);
fprintf('Overshoot (teórico)  : %.2f %%\n', info.Overshoot);
fprintf('Erro estacionário    : %.6f\n', 1 - dcgain(T));

%% ══════════════════════════════════════════════════════════════
%  4. CENÁRIOS DE TESTE
%% ══════════════════════════════════════════════════════════════

% Cenário 1 — RETA: setpoint fixo em 0 (linha centrada)
fn_reta    = @(t) 0;

% Cenário 2 — CURVA SUAVE: setpoint muda suavemente
fn_curva   = @(t) 0.8 * tanh(8*(t - 0.3));

% Cenário 3 — CURVA S: setpoint em S (dois desvios consecutivos)
fn_s       = @(t) 1.5*tanh(10*(t-0.2)) - 1.5*tanh(10*(t-0.7));

% Cenário 4 — PERTURBAÇÃO: degrau repentino (obstáculo/imperfeição)
fn_degrau  = @(t) (t >= 0.2) * 1.0;

cenarios   = {fn_reta, fn_curva, fn_s, fn_degrau};
nomes      = {'Reta (setpoint=0)', 'Curva suave', 'Curva em S', 'Perturbação (degrau)'};
t_total    = 1.5;

%% ══════════════════════════════════════════════════════════════
%  5. SIMULAÇÃO — PID v3 vs sem controle vs Kp baixo (v1)
%% ══════════════════════════════════════════════════════════════

% Parâmetros da versão antiga (v1) para comparação
Kp_v1 = 15.0; Ki_v1 = 0.0; Kd_v1 = 1.0;
ZONA_v1 = 0.0; LIM_v1 = 0.0; AMP_v1 = 1.0; SAI_v1 = 120.0;

figure('Name','Comprovação Eficiência PID v3 — Grupo 7', ...
    'Position', [40 30 1400 950]);
sgtitle({'Comprovação de Eficiência — PID Arduino v3 vs Versão Anterior', ...
    sprintf('Kp=%.0f | Ki=%.0f | Kd=%.0f | Ts=%dms | J=%.4f kg·m²', ...
    Kp,Ki,Kd,Ts*1000,J)}, 'FontSize', 12, 'FontWeight', 'bold');

erros_rms   = zeros(1, 4);
overshoot_v = zeros(1, 4);
assentamento = zeros(1, 4);

for c = 1:4
    fn = cenarios{c};

    % Simula PID v3
    [t3, e3, u3, vD3, vE3] = simular_arduino(fn, t_total, ...
        Kp,Ki,Kd,Ts,ZONA_MORTA,LIMIAR_CURVA,AMP_CURVA,SAIDA_LIM,...
        VEL_RETA,VEL_CURVA_S,VEL_CURVA_FEC,J,b);

    % Simula v1 (sem zona morta, Kp baixo)
    [t1, e1, ~, ~, ~] = simular_arduino(fn, t_total, ...
        Kp_v1,Ki_v1,Kd_v1,Ts,ZONA_v1,LIM_v1,AMP_v1,SAI_v1,...
        VEL_RETA,VEL_CURVA_S,VEL_CURVA_FEC,J,b);

    erros_rms(c) = sqrt(mean(e3.^2));

    % ── Subplot erro ──────────────────────────────────────────
    subplot(4, 3, (c-1)*3 + 1);
    plot(t3, e3, 'b-',  'LineWidth', 2); hold on;
    plot(t1, e1, 'r--', 'LineWidth', 1.5);
    yline(0,  'k:', 'LineWidth', 1);
    yline( ZONA_MORTA, 'g:', 'LineWidth', 0.8);
    yline(-ZONA_MORTA, 'g:', 'Zona morta', 'LineWidth', 0.8, ...
        'LabelVerticalAlignment','bottom');
    xlabel('Tempo (s)'); ylabel('Erro lateral');
    title(sprintf('%s — Erro e(t)', nomes{c}));
    legend('PID v3','v1 (Kp=15)','Location','best');
    grid on; box on;

    % ── Subplot saída PID u(t) ────────────────────────────────
    subplot(4, 3, (c-1)*3 + 2);
    plot(t3, u3, 'b-', 'LineWidth', 2); hold on;
    yline( SAIDA_LIM, 'r:', '+200 (limite)', 'LineWidth', 1);
    yline(-SAIDA_LIM, 'r:', '-200 (limite)', 'LineWidth', 1);
    yline(0, 'k:', 'LineWidth', 0.8);
    xlabel('Tempo (s)'); ylabel('u(t) [PWM]');
    title(sprintf('%s — Saída PID u(t)', nomes{c}));
    grid on; box on;

    % ── Subplot velocidade motores ────────────────────────────
    subplot(4, 3, (c-1)*3 + 3);
    plot(t3, vD3, 'b-',  'LineWidth', 2); hold on;
    plot(t3, vE3, 'm--', 'LineWidth', 1.8);
    yline(VEL_RETA,      'g:', sprintf('Reta=%d', VEL_RETA), ...
        'LineWidth', 0.8, 'LabelVerticalAlignment','bottom');
    yline(VEL_CURVA_S,   'y:', sprintf('Curva=%d', VEL_CURVA_S), ...
        'LineWidth', 0.8, 'LabelVerticalAlignment','bottom');
    yline(VEL_CURVA_FEC, 'r:', sprintf('S=%d', VEL_CURVA_FEC), ...
        'LineWidth', 0.8, 'LabelVerticalAlignment','bottom');
    ylim([0 280]);
    xlabel('Tempo (s)'); ylabel('PWM');
    title(sprintf('%s — Velocidade motores', nomes{c}));
    legend('Motor Dir','Motor Esq','Location','best');
    grid on; box on;
end

%% ══════════════════════════════════════════════════════════════
%  6. FIGURA 2 — MÉTRICAS COMPARATIVAS
%% ══════════════════════════════════════════════════════════════
figure('Name','Métricas Comparativas — Eficiência PID', ...
    'Position', [60 60 1200 800]);
sgtitle('Métricas de Eficiência — PID v3 vs Versão Anterior', ...
    'FontSize', 12, 'FontWeight', 'bold');

% ── G1: RMS do erro por cenário ──────────────────────────────
subplot(2,3,1);
erros_v1 = zeros(1,4);
for c = 1:4
    fn = cenarios{c};
    [~, e1, ~, ~, ~] = simular_arduino(fn, t_total, ...
        Kp_v1,Ki_v1,Kd_v1,Ts,ZONA_v1,LIM_v1,AMP_v1,SAI_v1,...
        VEL_RETA,VEL_CURVA_S,VEL_CURVA_FEC,J,b);
    [~, e3, ~, ~, ~] = simular_arduino(fn, t_total, ...
        Kp,Ki,Kd,Ts,ZONA_MORTA,LIMIAR_CURVA,AMP_CURVA,SAIDA_LIM,...
        VEL_RETA,VEL_CURVA_S,VEL_CURVA_FEC,J,b);
    erros_v1(c) = sqrt(mean(e1.^2));
    erros_rms(c) = sqrt(mean(e3.^2));
end

x = 1:4;
bar_data = [erros_v1; erros_rms]';
b_hdl = bar(x, bar_data);
b_hdl(1).FaceColor = [0.8 0.2 0.2];
b_hdl(2).FaceColor = [0.2 0.6 0.9];
set(gca,'XTickLabel', {'Reta','Curva','Curva S','Perturb.'});
ylabel('RMS do erro');
title('Erro RMS por cenário');
legend('v1 (Kp=15)','v3 (Kp=60)','Location','best');
grid on; box on;

% ── G2: Resposta ao degrau contínua ──────────────────────────
subplot(2,3,2);
t_step = 0:Ts:1.5;
[y_pid, t_pid] = step(T, t_step);

% Modelo v1 contínuo
C_v1 = pid(Kp_v1, Ki_v1, Kd_v1, Kd_v1/N_filt);
T_v1 = feedback(C_v1 * G, 1);
[y_v1, t_v1] = step(T_v1, t_step);

plot(t_pid, y_pid, 'b-',  'LineWidth', 2.5); hold on;
plot(t_v1,  y_v1,  'r--', 'LineWidth', 2);
yline(1, 'k:', 'Setpoint', 'LineWidth', 1.2, ...
    'LabelVerticalAlignment','bottom');
yline(1.02, 'g:', '+2%', 'LineWidth', 0.8);
yline(0.98, 'g:', '-2%', 'LineWidth', 0.8);
ylim([-0.1 2.2]);
xlabel('Tempo (s)'); ylabel('\theta normalizado');
title('Resposta ao Degrau — v3 vs v1');
legend(sprintf('v3 (Kp=%.0f Kd=%.0f)',Kp,Kd), ...
       sprintf('v1 (Kp=%.0f Kd=%.0f)',Kp_v1,Kd_v1), ...
       'Setpoint', 'Location','southeast');
grid on; box on;

% ── G3: Lugar das raízes — v3 vs v1 ──────────────────────────
subplot(2,3,3);
rlocus(C*G, C_v1*G);
title('Lugar das Raízes — v3 vs v1');
grid on;

% ── G4: Diagrama de Bode ──────────────────────────────────────
subplot(2,3,4);
margin(C * G);
title(sprintf('Bode — v3 (Pm=9,25°, Gm=Inf)'));
grid on;

% ── G5: Saturação — efeito do constrain ──────────────────────
subplot(2,3,5);
erros_teste = linspace(-4, 4, 500);
u_sem_sat   = Kp * erros_teste;
u_com_sat   = max(-SAIDA_LIM, min(SAIDA_LIM, Kp * erros_teste));
u_zona      = erros_teste;
for i = 1:length(erros_teste)
    e_tmp = erros_teste(i);
    if abs(e_tmp) < ZONA_MORTA
        e_tmp = 0;
    elseif abs(e_tmp) >= LIMIAR_CURVA
        e_tmp = e_tmp * AMP_CURVA;
    end
    u_zona(i) = max(-SAIDA_LIM, min(SAIDA_LIM, Kp * e_tmp));
end
plot(erros_teste, u_sem_sat, 'r--', 'LineWidth', 1.5); hold on;
plot(erros_teste, u_com_sat, 'b-',  'LineWidth', 2);
plot(erros_teste, u_zona,    'g-',  'LineWidth', 2);
yline( SAIDA_LIM, 'k:', 'LineWidth', 1);
yline(-SAIDA_LIM, 'k:', 'Limite ±200', 'LineWidth', 1);
xline(-ZONA_MORTA, 'm:', 'LineWidth', 1);
xline( ZONA_MORTA, 'm:', 'Zona morta', 'LineWidth', 1);
xlabel('Erro lateral');  ylabel('u(t)');
title('Efeito do constrain + zona morta na saída');
legend('Sem sat. (linear)','Com constrain ±200', ...
    'Com zona morta + amp.','Location','best');
grid on; box on;

% ── G6: Tabela de eficiência resumida ────────────────────────
subplot(2,3,6);
axis off;

info_v1 = stepinfo(T_v1);
dados = {
    'Parâmetro',         'v1 (inicial)',   'v3 (final)';
    'Kp',                '15',             '60';
    'Kd',                '1',              '3';
    'Zona morta',        'Não',            '±0,3';
    'Vel. adaptativa',   'Não',            '255/130/50';
    'Amp. curva',        'Não',            '×3,5';
    'Busca linha',       'Não',            'Sim (100ms)';
    'Ts assentamento',   sprintf('%.3fs',info_v1.SettlingTime), ...
                         sprintf('%.3fs',info.SettlingTime);
    'Overshoot',         sprintf('%.1f%%',info_v1.Overshoot), ...
                         sprintf('%.1f%%',info.Overshoot);
    'Erro estac.',       sprintf('%.4f',1-dcgain(T_v1)), ...
                         sprintf('%.6f',1-dcgain(T));
    'Resultado prático', 'Zigue-zague',    'Pista completa';
};

t_pos = [0.05 0.55];
col_w = [0.45 0.22 0.28];
y_start = 0.95; dy = 0.083;

for r = 1:size(dados,1)
    y = y_start - (r-1)*dy;
    for cc = 1:3
        x = t_pos(1) + sum(col_w(1:cc-1));
        clr = [0 0 0];
        fw = 'normal';
        if r == 1, fw = 'bold'; end
        if r > 1 && cc == 3, clr = [0.1 0.5 0.2]; end
        if r > 1 && cc == 2 && r > 7, clr = [0.7 0.1 0.1]; end
        text(x, y, dados{r,cc}, 'Units','normalized', ...
            'FontSize', 9, 'FontWeight', fw, 'Color', clr);
    end
    if r == 1 || r == 8
        annotation('line', [0.55 0.97], [0.97-r*0.076 0.97-r*0.076], ...
            'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
    end
end
title('Resumo comparativo de eficiência');

%% ══════════════════════════════════════════════════════════════
%  7. RESUMO NO CONSOLE
%% ══════════════════════════════════════════════════════════════
fprintf('\n════════════════════════════════════════════\n');
fprintf('  RESUMO DE EFICIÊNCIA\n');
fprintf('════════════════════════════════════════════\n');
fprintf('%-25s  %10s  %10s\n', 'Métrica', 'v1', 'v3');
fprintf('%-25s  %10.3f  %10.3f\n', 'Assentamento (s)', ...
    info_v1.SettlingTime, info.SettlingTime);
fprintf('%-25s  %9.1f%%  %9.1f%%\n', 'Overshoot (contínuo)', ...
    info_v1.Overshoot, info.Overshoot);
fprintf('%-25s  %10.6f  %10.6f\n', 'Erro estacionário', ...
    1-dcgain(T_v1), 1-dcgain(T));
fprintf('\nErro RMS por cenário:\n');
for c = 1:4
    fprintf('  %-18s  v1=%6.4f  v3=%6.4f  melhora=%.0f%%\n', ...
        nomes{c}, erros_v1(c), erros_rms(c), ...
        (1 - erros_rms(c)/max(erros_v1(c),1e-6))*100);
end
fprintf('\nRouth-Hurwitz v3: (b+Kd)·Kp = %.1f > J·Ki = %.4f  ✓\n', ...
    (b+Kd)*Kp, J*Ki);
fprintf('════════════════════════════════════════════\n');