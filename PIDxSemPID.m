% ============================================================
%  COMPARATIVO: Sem Controle vs PID v3
%  Grupo 7 | Karolaine | 2026
%  Mostra claramente por que o PID é necessário
% ============================================================

clear; clc; close all;

%% ── 1. PARÂMETROS ────────────────────────────────────────────
massa = 0.600; larg = 0.120; comp = 0.185;
J = (massa/12) * (larg^2 + comp^2);
b = 0.05;

Kp = 60.0; Ki = 0.0; Kd = 3.0;
Ts = 0.010; N = 100;

fprintf('J = %.6f kg·m²  |  b = %.3f\n', J, b);
fprintf('Kp=%.0f | Ki=%.0f | Kd=%.0f | Ts=%dms\n\n', Kp,Ki,Kd,Ts*1000);

%% ── 2. MODELOS CONTÍNUOS ─────────────────────────────────────
G = tf(1, [J, b, 0]);           % planta (sem controle)
C = pid(Kp, Ki, Kd, Kd/N);     % controlador PID v3
T = feedback(C * G, 1);         % malha fechada com PID

%% ── 3. MÉTRICAS ──────────────────────────────────────────────
info = stepinfo(T);
fprintf('── Com PID ──\n');
fprintf('Assentamento : %.4f s\n', info.SettlingTime);
fprintf('Overshoot    : %.2f %%\n', info.Overshoot);
fprintf('Erro estac.  : %.6f\n\n', 1 - dcgain(T));

%% ── 4. FIGURA ÚNICA — comparativo completo ───────────────────
t = 0:Ts:3;

figure('Name','Sem PID vs Com PID — Grupo 7','Position',[60 40 1280 880]);
sgtitle({'Sem Controle vs PID v3 — Carrinho Seguidor de Linha | Grupo 7', ...
    sprintf('Kp=%.0f | Ki=%.0f | Kd=%.0f | J=%.4f kg·m² | b=%.2f', ...
    Kp,Ki,Kd,J,b)}, 'FontSize',13,'FontWeight','bold');

% ── G1: Resposta ao Degrau — comparativo principal ───────────
subplot(2,3,[1 2]);
[y_pid, t_pid] = step(T, t);

% Sem controle: simula planta com entrada degrau unitária
% G(s) = 1/(Js²+bs) → rampa crescente (instável a degrau)
[y_sem, t_sem] = step(G, t);
y_sem_clip = min(y_sem, 4.5);   % limita para visualização

plot(t_pid, y_pid,      'b-',  'LineWidth', 3);   hold on;
plot(t_sem, y_sem_clip, 'r--', 'LineWidth', 2.5);
yline(1,    'k:',  'Setpoint', 'LineWidth', 1.5, ...
    'LabelVerticalAlignment','bottom','FontSize',11);
yline(1.02, 'g--', '+2%',  'LineWidth', 0.8);
yline(0.98, 'g--', '-2%',  'LineWidth', 0.8);

% Anotações
xline(info.SettlingTime, 'm--', ...
    sprintf('Ts=%.3fs', info.SettlingTime), ...
    'LineWidth',1.2,'LabelVerticalAlignment','bottom','FontSize',10);

text(0.5, 3.8, 'SEM CONTROLE: diverge → robô sai da pista', ...
    'Color','r','FontSize',10,'FontWeight','bold');
text(0.5, 1.35, sprintf('COM PID: assenta em %.3fs', info.SettlingTime), ...
    'Color','b','FontSize',10,'FontWeight','bold');

ylim([-0.2 4.8]);
xlabel('Tempo (s)','FontSize',11);
ylabel('\theta normalizado','FontSize',11);
title('Resposta ao Degrau — Sem Controle vs Com PID v3','FontSize',12);
legend(sprintf('Com PID (Kp=%.0f, Kd=%.0f)',Kp,Kd), ...
    'Sem controle (diverge)', 'Setpoint', 'Location','northeast','FontSize',10);
grid on; box on;


% Erro sem controle (não converge → erro permanente = 1-y_sem)
erro_sem = 1 - min(y_sem, 5);

plot(t_pid, erro_pid, 'b-',  'LineWidth', 2.5); hold on;
plot(t_sem, erro_sem, 'r--', 'LineWidth', 2);
yline(0,  'k:', 'Erro zero', 'LineWidth',1.2, ...
    'LabelVerticalAlignment','bottom');
ylim([-0.5 2]);
xlabel('Tempo (s)','FontSize',10);
ylabel('e(t) = setpoint − saída','FontSize',10);
title('Erro ao Longo do Tempo','FontSize',11);
legend('Com PID','Sem controle','Location','best','FontSize',9);
grid on; box on;

% ── G5: Tabela resumo visual ──────────────────────────────────
subplot(2,3,6);
axis off;

linhas = {
    'Parâmetro',        'Sem controle',     'Com PID v3';
    'Erro estacionário','Nunca zero',        '0,000000 ✓';
    'Assentamento',     'Não assenta',       '0,186 s';
    'Overshoot',        '—',                 '77,1% (modelo)*';
    'Estabilidade',     'INSTÁVEL',          'ESTÁVEL (Routh ✓)';
    'Margem de fase',   '< 0°',              '9,25°';
    'Polo em s=0',      'Integra sem limite','Controlado pelo PID';
    'Resultado prático','Robô sai da pista', 'Pista completa ✓';
};

dy = 0.105; y0 = 0.97;
for r = 1:size(linhas,1)
    y = y0 - (r-1)*dy;
    fw = 'normal'; fs = 9;
    if r == 1, fw = 'bold'; fs = 10; end

    clr_c2 = [0.6 0.1 0.1];
    clr_c3 = [0.1 0.45 0.15];
    if r == 1, clr_c2 = [0 0 0]; clr_c3 = [0 0 0]; end

    text(0.02, y, linhas{r,1}, 'Units','normalized', ...
        'FontSize',fs,'FontWeight',fw,'Color',[0 0 0]);
    text(0.42, y, linhas{r,2}, 'Units','normalized', ...
        'FontSize',fs,'FontWeight',fw,'Color',clr_c2);
    text(0.72, y, linhas{r,3}, 'Units','normalized', ...
        'FontSize',fs,'FontWeight',fw,'Color',clr_c3);
end

% Linhas separadoras
annotation('line',[0.69 0.97],[0.885 0.885],'Color',[0.8 0.8 0.8]);
annotation('line',[0.69 0.97],[0.78  0.78 ],'Color',[0.8 0.8 0.8]);

text(0.02, 0.03, '* Overshoot do modelo linear — suprimido na prática pelo constrain ±200', ...
    'Units','normalized','FontSize',7.5,'Color',[0.5 0.5 0.5]);

title('Resumo: Sem Controle vs Com PID v3','FontSize',11);

%% ── 5. RESUMO CONSOLE ────────────────────────────────────────
fprintf('════════════════════════════════════════════\n');
fprintf('  SEM CONTROLE vs COM PID v3\n');
fprintf('════════════════════════════════════════════\n');
fprintf('%-22s  %-18s  %-18s\n','Métrica','Sem controle','Com PID v3');
fprintf('%-22s  %-18s  %-18s\n','Resultado','Sai da pista','Pista completa');
fprintf('════════════════════════════════════════════\n');