%% ============================================================
% 教程 29：MSD 系统控制 — PID vs LQR 对比
% 目标：同一个被控对象，用两种控制器分别设计，看效果差异
%
% 【两种控制策略】
%   PID  — 经典三参数 (Kp, Ki, Kd)，直观调参
%          不需要模型，看响应曲线调即可
%   LQR  — 最优控制 (Q, R 权值矩阵)，数学优化
%          需要状态空间模型，自动算出最优增益
%
% 【被控对象】
%   质量-弹簧-阻尼: G(s) = 1/(s² + 0.5s + 10)
%   状态 x = [位移; 速度]，控制 u = 力
%
% 【本课内容】
%   1. PID 参数整定
%   2. LQR 设计 (Q/R 权值调参)
%   3. Simulink 模型：PID / LQR 一键切换 + 传感器噪声
%   4. 多种参考信号：阶跃 / 正弦 / 方波
% ============================================================

clear; close all;

fprintf('============================================\n');
fprintf('  教程 29：MSD 系统控制策略对比\n');
fprintf('============================================\n\n');

%% ---------- 系统参数 ----------
m = 1.0;  c = 0.5;  k_s = 10.0;
A = [0, 1; -k_s/m, -c/m];   B = [0; 1/m];
C = [1, 0];                  D = 0;

fprintf('【被控对象】G(s) = 1/(%.0fs² + %.1fs + %.0f)\n', m, c, k_s);
fprintf('  开环极点: %.2f ± j%.2f  (欠阻尼振荡)\n\n', real(eig(A)), abs(imag(eig(A))));

%% ---------- 第 1 步：PID 控制器 ----------
Kp = 50;  Ki = 20;  Kd = 5;  N_filt = 100;
fprintf('【PID 参数】\n');
fprintf('  Kp = %.0f  — 比例：有误差立即反应\n', Kp);
fprintf('  Ki = %.0f  — 积分：累积误差，消除静差\n', Ki);
fprintf('  Kd = %.0f  — 微分：预判趋势，抑制超调\n\n', Kd);

%% ---------- 第 2 步：LQR 控制器 ----------
Q = diag([100, 1]);   % 位置权重 >> 速度权重
R = 0.1;              % 控制代价
K_lqr = lqr(A, B, Q, R);
fprintf('【LQR 设计】\n');
fprintf('  Q = diag(100, 1) — 重点惩罚位置偏差\n');
fprintf('  R = 0.1 — 允许用较大的控制力\n');
fprintf('  K = [%.2f, %.2f] — 最优状态反馈增益\n', K_lqr);
fprintf('  闭环极点: %.2f ± j%.2f  (比开环更快更稳)\n\n', real(eig(A-B*K_lqr)), abs(imag(eig(A-B*K_lqr))));

%% ---------- 第 3 步：Simulink 模型 — PID vs LQR ----------

mdl = 'tutorial29_control_compare';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

% --- 三种参考信号 ---
add_block('simulink/Sources/Step', [mdl '/Step Ref'], ...
    'Position', [30, 30, 80, 55]);
set_param([mdl '/Step Ref'], 'Time', '0.2', 'After', '1');

add_block('simulink/Sources/Sine Wave', [mdl '/Sine Ref'], ...
    'Position', [30, 85, 80, 110]);
set_param([mdl '/Sine Ref'], 'Frequency', '5', 'Amplitude', '0.5');

add_block('simulink/Sources/Pulse Generator', [mdl '/Square Ref'], ...
    'Position', [30, 140, 80, 165]);
set_param([mdl '/Square Ref'], 'Period', '2', 'Amplitude', '1');

% --- 信号切换（两级串联，三种信号任选）---
add_block('simulink/Signal Routing/Manual Switch', [mdl '/SignalSelect1'], ...
    'Position', [150, 55, 190, 100]);
add_block('simulink/Signal Routing/Manual Switch', [mdl '/SignalSelect2'], ...
    'Position', [150, 130, 190, 175]);

add_line(mdl, 'Step Ref/1', 'SignalSelect1/1');
add_line(mdl, 'Sine Ref/1', 'SignalSelect1/2');
add_line(mdl, 'SignalSelect1/1', 'SignalSelect2/2');
add_line(mdl, 'Square Ref/1', 'SignalSelect2/1');

% --- PID 控制器 + 误差计算 ---
add_block('simulink/Math Operations/Sum', [mdl '/PID Error'], ...
    'Position', [270, 40, 300, 80]);
set_param([mdl '/PID Error'], 'Inputs', '|+-');

add_block('simulink/Continuous/PID Controller', [mdl '/PID Ctrl'], ...
    'Position', [340, 40, 410, 90]);
set_param([mdl '/PID Ctrl'], 'P', num2str(Kp), 'I', num2str(Ki), ...
    'D', num2str(Kd), 'N', num2str(N_filt));

add_line(mdl, 'SignalSelect2/1', 'PID Error/1');
add_line(mdl, 'PID Error/1', 'PID Ctrl/1');

% --- LQR 控制器 (MATLAB Function) ---
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/LQR Ctrl'], ...
    'Position', [270, 130, 360, 200]);

sf = sfroot();
mc = sf.find('-isa', 'Stateflow.Machine', 'Name', mdl);
if ~isempty(mc)
    ch = mc.find('-isa', 'Stateflow.EMChart');
    if ~isempty(ch)
        ch.Script = sprintf(['function u = LQR_Ctrl(state, ref)\n' ...
            '%% LQR 最优控制律: u = -K1*(x1-ref) - K2*x2\n' ...
            'K = [%.2f, %.2f];\n' ...
            'u = -K(1)*(state(1)-ref) - K(2)*state(2);\n' ...
            'u = max(min(u, 10), -10);   %% 限幅\n'], K_lqr(1), K_lqr(2));
    end
end

add_line(mdl, 'SignalSelect2/1', 'LQR Ctrl/2');     % ref → LQR

% --- 控制器切换 ---
add_block('simulink/Signal Routing/Manual Switch', [mdl '/Ctrl Select'], ...
    'Position', [460, 70, 500, 110]);

add_line(mdl, 'PID Ctrl/1', 'Ctrl Select/1');
add_line(mdl, 'LQR Ctrl/1', 'Ctrl Select/2');

% --- 被控对象 (State-Space) ---
add_block('simulink/Continuous/State-Space', [mdl '/MSD Plant'], ...
    'Position', [580, 50, 660, 120]);
set_param([mdl '/MSD Plant'], ...
    'A', sprintf('[0, 1; %.1f, %.1f]', -k_s/m, -c/m), ...
    'B', '[0; 1]', ...
    'C', 'eye(2)', ...         % 输出 [位移; 速度]
    'D', '[0; 0]', ...
    'X0', '[0; 0]');

add_line(mdl, 'Ctrl Select/1', 'MSD Plant/1');

% --- 传感器噪声 ---
add_block('simulink/Sources/Band-Limited White Noise', [mdl '/Sensor Noise'], ...
    'Position', [580, 170, 630, 195]);

add_block('simulink/Math Operations/Sum', [mdl '/Add Noise'], ...
    'Position', [710, 60, 740, 100]);
set_param([mdl '/Add Noise'], 'Inputs', '|++', 'IconShape', 'round');

add_line(mdl, 'MSD Plant/1', 'Add Noise/1');
add_line(mdl, 'Sensor Noise/1', 'Add Noise/2');

% --- 反馈回路 ---
add_block('simulink/Signal Routing/Demux', [mdl '/Demux'], ...
    'Position', [790, 65, 820, 105]);
set_param([mdl '/Demux'], 'Outputs', '2');

add_line(mdl, 'Add Noise/1', 'Demux/1');
add_line(mdl, 'Demux/1', 'PID Error/2');              % 位移 → PID(-)

% Mux 合并 [位移; 速度] 给 LQR
add_block('simulink/Signal Routing/Mux', [mdl '/State Mux'], ...
    'Position', [860, 155, 885, 190]);
set_param([mdl '/State Mux'], 'Inputs', '2');

add_line(mdl, 'Demux/1', 'State Mux/1');
add_line(mdl, 'Demux/2', 'State Mux/2');
add_line(mdl, 'State Mux/1', 'LQR Ctrl/1');          % [x1;x2] → LQR

% --- 示波器 ---
add_block('simulink/Sinks/Scope', [mdl '/Response'], ...
    'Position', [870, 30, 930, 75]);
set_param([mdl '/Response'], 'NumInputPorts', '2');

add_line(mdl, 'Add Noise/1', 'Response/1');           % 实际输出
add_line(mdl, 'SignalSelect2/1', 'Response/2');       % 参考信号

fprintf('【Simulink 模型】PID/LQR 手动切换 + 噪声 + 多信号源\n');
fprintf('  双击 Manual Switch 切换控制器和信号类型\n\n');

%% ---------- 第 4 步：阶跃响应对比 ----------
[y_open, t_open] = step(tf([1], [m, c, k_s]), 5);
sys_cl = ss(A - B*K_lqr, B, C, D);
[y_lqr, t_lqr] = step(sys_cl, 5);

figure('Name', 't29: PID vs LQR', 'Position', [50, 50, 800, 400]);
plot(t_open, y_open, 'Color', [0.6, 0.6, 0.6], 'LineWidth', 1.5); hold on;
plot(t_lqr, y_lqr, 'r', 'LineWidth', 1.5);
legend('开环 (无控制)', 'LQR 闭环', 'Location', 'best');
title('阶跃响应：LQR 闭环 vs 开环');
xlabel('时间 (s)'); ylabel('位移 (m)'); grid on;

S_open = stepinfo(y_open, t_open);
S_lqr  = stepinfo(y_lqr, t_lqr);
fprintf('  开环  — 上升时间: %.2fs  调节时间: %.2fs  超调: %.0f%%\n', ...
    S_open.RiseTime, S_open.SettlingTime, S_open.Overshoot);
fprintf('  LQR   — 上升时间: %.2fs  调节时间: %.2fs  超调: %.0f%%\n', ...
    S_lqr.RiseTime, S_lqr.SettlingTime, S_lqr.Overshoot);

fprintf('\n========================================\n');
fprintf('  教程 29 完成！\n');
fprintf('========================================\n\n');
fprintf('动手实验：\n');
fprintf('  1. 打开 tutorial29_control_compare.slx\n');
fprintf('  2. 双击 Manual Switch 切换 PID / LQR\n');
fprintf('  3. 切换 Step / Sine / Square 信号\n');
fprintf('  4. 改 Q=diag(1000,1) 或 R=0.01 看 LQR 增益变化\n');
fprintf('  5. 把 Kp 改大 2 倍，观察超调量的变化\n');

save_system(mdl, fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']));
close_system(mdl, 0);
