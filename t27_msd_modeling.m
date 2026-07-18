%% ============================================================
% 教程 27：MSD 系统 — 四种建模方法对比
% 目标：同一个物理系统，用四种不同的数学方式描述
%
% 【质量-弹簧-阻尼 (MSD) 系统】
%   物理方程：m·x" + c·x' + k·x = F(t)
%   一个质量块 m，连着一根弹簧 k 和一个阻尼器 c，
%   受到外力 F(t) 后产生位移 x(t)。
%
% 【四种建模方法】
%   1. 传递函数 G(s)     — 频域分析首选，SISO 系统最简洁
%   2. 状态空间 (A,B,C,D) — 现代控制基础，可处理 MIMO
%   3. 离散化 (ZOH/Tustin) — 数字控制器需要把连续模型变离散
%   4. 数值积分 (Euler/RK4) — 非线性系统、实时仿真用
%
% 【四种方法描述的是同一个系统，结果应该一致】
% ============================================================

clear; close all;

fprintf('============================================\n');
fprintf('  教程 27：MSD 系统建模方法对比\n');
fprintf('============================================\n\n');

%% ---------- 系统参数 ----------
m = 1.0;    % 质量 (kg)
c = 0.5;    % 阻尼系数 (N·s/m)
k = 10.0;   % 弹簧刚度 (N/m)

fprintf('【物理系统】m=%.1f kg, c=%.1f Ns/m, k=%.0f N/m\n', m, c, k);
fprintf('  微分方程: m·x" + c·x'' + k·x = F(t)\n\n');

%% ---------- 第 1 步：方法① — 传递函数 ----------
G_tf = tf([1], [m, c, k]);  % G(s) = 1/(s² + 0.5s + 10)
fprintf('【方法① 传递函数】\n');
fprintf('  G(s) = 1 / (%.0fs² + %.1fs + %.0f)\n', m, c, k);
fprintf('  极点: %.2f ± j%.2f  → 自然频率 ωn=%.2f rad/s\n\n', ...
    real(pole(G_tf)), abs(imag(pole(G_tf))), sqrt(k/m));

%% ---------- 第 2 步：方法② — 状态空间 ----------
A = [0, 1; -k/m, -c/m];           % 状态矩阵
B = [0; 1/m];                      % 输入矩阵
C = [1, 0];                        % 输出矩阵（只看位移）
D = 0;

fprintf('【方法② 状态空间】ẋ = Ax + Bu,  y = Cx\n');
fprintf('  A = [0, 1; %.0f, %.1f]\n', -k/m, -c/m);
fprintf('  B = [0; %.0f]\n', 1/m);
fprintf('  C = [1, 0]  — 只观测位移\n\n');

%% ---------- 第 3 步：Simulink 模型 — TF vs SS 对比 ----------

mdl = 'tutorial27_msd_models';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

% --- 信号源：阶跃 + 正弦 + 手动切换 ---
add_block('simulink/Sources/Step', [mdl '/Step Input'], ...
    'Position', [50, 80, 100, 110]);
set_param([mdl '/Step Input'], 'Time', '0.5', 'After', '1');

add_block('simulink/Sources/Sine Wave', [mdl '/Sine Input'], ...
    'Position', [50, 160, 100, 190]);
set_param([mdl '/Sine Input'], 'Frequency', num2str(sqrt(k/m)), 'Amplitude', '1');

add_block('simulink/Signal Routing/Manual Switch', [mdl '/Signal Select'], ...
    'Position', [160, 100, 210, 170]);

add_line(mdl, 'Step Input/1', 'Signal Select/1');
add_line(mdl, 'Sine Input/1', 'Signal Select/2');

% --- 传递函数模型 ---
add_block('simulink/Continuous/Transfer Fcn', [mdl '/TF G_s'], ...
    'Position', [280, 80, 360, 150]);
set_param([mdl '/TF G_s'], ...
    'Numerator', '[1]', ...
    'Denominator', sprintf('[%.1f %.1f %.1f]', m, c, k));

add_line(mdl, 'Signal Select/1', 'TF G_s/1');

% --- 状态空间模型 ---
add_block('simulink/Continuous/State-Space', [mdl '/SS Model'], ...
    'Position', [280, 200, 360, 270]);
set_param([mdl '/SS Model'], ...
    'A', sprintf('[0 1;%.4f %.4f]', -k/m, -c/m), ...
    'B', '[0;1]', ...
    'C', 'eye(2)', ...        % 输出 [位移; 速度]
    'D', '[0;0]', ...
    'X0', '[0;0]');

add_line(mdl, 'Signal Select/1', 'SS Model/1');

% --- 分离 SS 输出的位移和速度 ---
add_block('simulink/Signal Routing/Demux', [mdl '/Demux'], ...
    'Position', [390, 200, 410, 270]);
set_param([mdl '/Demux'], 'Outputs', '2');

add_line(mdl, 'SS Model/1', 'Demux/1');

% --- 示波器：TF vs SS 位移对比 ---
add_block('simulink/Sinks/Scope', [mdl '/TF vs SS Position'], ...
    'Position', [440, 90, 520, 160]);
set_param([mdl '/TF vs SS Position'], 'NumInputPorts', '2');

add_line(mdl, 'TF G_s/1', 'TF vs SS Position/1');     % TF 输出 → 端口1
add_line(mdl, 'Demux/1', 'TF vs SS Position/2');       % SS 位移 → 端口2

% --- SS 速度单独观测 ---
add_block('simulink/Sinks/Scope', [mdl '/SS Velocity'], ...
    'Position', [440, 220, 520, 260]);

add_line(mdl, 'Demux/2', 'SS Velocity/1');             % SS 速度 → 示波器

% --- 信号记录：导出到工作区做 MATLAB 分析 ---
add_block('simulink/Sinks/To Workspace', [mdl '/y_tf'], ...
    'Position', [580, 90, 650, 130]);
set_param([mdl '/y_tf'], 'VariableName', 'y_tf_sim');

add_block('simulink/Sinks/To Workspace', [mdl '/y_ss'], ...
    'Position', [580, 180, 650, 220]);
set_param([mdl '/y_ss'], 'VariableName', 'y_ss_sim');

add_line(mdl, 'TF G_s/1', 'y_tf/1');
add_line(mdl, 'Demux/1', 'y_ss/1');

fprintf('【Simulink 模型】TF vs SS 两条路径并行对比\n');
fprintf('  切换 Manual Switch 可换阶跃/正弦输入\n\n');

%% ---------- 第 4 步：方法④ — 数值积分对比 ----------
dt = 0.001;  t_end = 5;
t = 0:dt:t_end;  N = length(t);
F = zeros(1, N);  F(t >= 0.5) = 5;  % 阶跃力 5N

% --- Euler 法 (最简单，一阶精度) ---
x_euler = zeros(1, N);  v_euler = zeros(1, N);
for n = 1:N-1
    a = (F(n) - c*v_euler(n) - k*x_euler(n)) / m;
    v_euler(n+1) = v_euler(n) + a*dt;
    x_euler(n+1) = x_euler(n) + v_euler(n)*dt;
end

% --- RK4 法 (四阶精度，最常用) ---
x_rk4 = zeros(1, N);  v_rk4 = zeros(1, N);
for n = 1:N-1
    f = @(xv) (F(n) - c*xv(2) - k*xv(1)) / m;
    k1v = f([x_rk4(n), v_rk4(n)]) * dt;      k1x = v_rk4(n) * dt;
    k2v = f([x_rk4(n)+k1x/2, v_rk4(n)+k1v/2]) * dt;  k2x = (v_rk4(n)+k1v/2) * dt;
    k3v = f([x_rk4(n)+k2x/2, v_rk4(n)+k2v/2]) * dt;  k3x = (v_rk4(n)+k2v/2) * dt;
    k4v = f([x_rk4(n)+k3x, v_rk4(n)+k3v]) * dt;      k4x = (v_rk4(n)+k3v) * dt;
    v_rk4(n+1) = v_rk4(n) + (k1v+2*k2v+2*k3v+k4v) / 6;
    x_rk4(n+1) = x_rk4(n) + (k1x+2*k2x+2*k3x+k4x) / 6;
end

% --- ode45 (MATLAB 标准解法，参考基准) ---
[t_ode, y_ode] = ode45(@(t, y) [y(2); (5 - c*y(2) - k*y(1))/m], t, [0; 0]);

%% ---------- 第 5 步：绘图对比 ----------
figure('Name', 't27: 四种建模方法对比', 'Position', [50, 50, 1000, 500]);

subplot(1,2,1); hold on;
plot(t, x_euler, 'b', 'LineWidth', 1);
plot(t, x_rk4, 'r', 'LineWidth', 1.5);
plot(t_ode, y_ode(:,1), 'k--', 'LineWidth', 1);
legend('Euler (dt=1ms)', 'RK4 (dt=1ms)', 'ode45 (参考)', 'Location', 'best');
title('位移响应对比：Euler vs RK4 vs ode45');
xlabel('时间 (s)'); ylabel('位移 (m)'); grid on;

subplot(1,2,2); hold on;
plot(t, abs(x_euler - y_ode(:,1)'), 'b', 'LineWidth', 1);
plot(t, abs(x_rk4 - y_ode(:,1)'), 'r', 'LineWidth', 1);
legend('Euler 误差', 'RK4 误差', 'Location', 'best');
title('积分误差（对数坐标）');
xlabel('时间 (s)'); ylabel('|误差| (m)'); grid on;
set(gca, 'YScale', 'log');

fprintf('\n【数值积分精度】\n');
fprintf('  Euler 最大误差: %.2e m\n', max(abs(x_euler - y_ode(:,1)')));
fprintf('  RK4   最大误差: %.2e m\n', max(abs(x_rk4 - y_ode(:,1)')));

fprintf('\n========================================\n');
fprintf('  教程 27 完成！\n');
fprintf('========================================\n\n');
fprintf('动手实验：\n');
fprintf('  1. 打开 tutorial27_msd_models.slx，切换阶跃/正弦输入\n');
fprintf('  2. 观察 TF 和 SS 两条曲线完全重合 — 两种方法等价\n');
fprintf('  3. 把 dt 从 1ms 改成 10ms，看 Euler 法的误差如何增大\n');
fprintf('  4. 改系统参数 m=5, c=2，重新运行看响应变化\n');

save_system(mdl, fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']));
close_system(mdl, 0);
