%% ============================================================
% 教程 12：Kalman 滤波 — 带噪声的 Luenberger 观测器
%
% 【从 Luenberger 到 Kalman】
%   t11 的 Luenberger 观测器假设：y = Cx（无测量噪声）
%   现实中：传感器有噪声、系统有扰动
%
%   真实系统：  ẋ = Ax + Bu + w      (w: 过程噪声)
%              y = Cx + v            (v: 测量噪声)
%
%   Kalman 滤波和 Luenberger 观测器结构完全相同：
%     ẋ̂ = Ax̂ + Bu + L(y - ŷ)
%
%   唯一的区别是 L 怎么来：
%     Luenberger: L = place(A', C', p)'     (手动指定收敛速度)
%     Kalman:     L = lqe(A, G, C, Q, R)    (噪声统计自动优化)
%
% ┌─────────────────────────────────────────────────────────┐
% │ 一、Kalman 的核心思想                                    │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │   Q: 过程噪声协方差 — "模型有多不靠谱？"                 │
% │      Q 大 → 不相信模型 → L 大 → 更相信测量              │
% │                                                         │
% │   R: 测量噪声协方差 — "传感器有多不准？"                 │
% │      R 大 → 不相信传感器 → L 小 → 更相信模型            │
% │                                                         │
% │   关键比：L 的大小由 Q/R 的比例决定                      │
% │     Q/R 大 → 传感器相对靠谱 → L 大 → 快速跟踪           │
% │     Q/R 小 → 传感器不太准 → L 小 → 平滑滤波             │
% └─────────────────────────────────────────────────────────┘
%
% ┌─────────────────────────────────────────────────────────┐
% │ 二、Kalman vs Luenberger — 一个对比实验                  │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │   场景：测量噪声 σ_v = 0.05 (位移传感器有噪声)           │
% │         Luenberger 用极点 [-6, -8] → L = [13.5; 37.2]   │
% │         Kalman 用 Q=0.01, R=0.05² → L 自动计算          │
% │                                                         │
% │   Luenberger 问题：L 太大 → 噪声被放大进估计             │
% │   Kalman 优势：L 自动调节 → 噪声被滤掉                   │
% └─────────────────────────────────────────────────────────┘
%
% 【本课目标】
%   1. 理解过程噪声 Q 和测量噪声 R 的含义
%   2. 用 lqe() 设计稳态 Kalman 滤波器
%   3. 对比 Luenberger vs Kalman 在噪声下的表现
%   4. 理解 Q/R 调节 = "信任模型 vs 信任传感器"
% ============================================================

clear; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

%% ===== 系统参数 =====

m = 1.0;  c = 0.5;  k = 4.0;

A = [   0,     1  ;
      -k/m,  -c/m ];

B = [  0  ;
      1/m ];

C = [ 1, 0 ];
D = 0;
G = eye(2);    % 过程噪声通过矩阵（每个状态都有独立噪声）

fprintf('============================================\n');
fprintf('  教程 12：Kalman 滤波\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：分析噪声对 Luenberger 的影响 =====

% Luenberger 观测器（和 t11 一样）
p_obs = [-6, -8];
L_luen = place(A', C', p_obs)';

% Kalman 滤波器
% 测量噪声：位移传感器标准差 σ_v = 0.05 → R = σ² = 0.0025
% 过程噪声：模型不确定性（小的随机扰动）
R_kal = 0.0025;       % 测量噪声协方差
Q_kal = diag([0.01, 0.1]);  % 过程噪声协方差（速度扰动 > 位移扰动）

[L_kal, P_kal, E_kal] = lqe(A, G, C, Q_kal, R_kal);

fprintf('【增益对比】\n');
fprintf('  Luenberger (极点=[-6,-8])   L = [ %7.3f;  %7.3f ]\n', L_luen(1), L_luen(2));
fprintf('  Kalman     (Q₀=%.2f, R=%.4f) L = [ %7.3f;  %7.3f ]\n\n', ...
    Q_kal(1,1), R_kal, L_kal(1), L_kal(2));

fprintf('  观察：Luenberger 的 L 更大 → 对噪声敏感\n');
fprintf('        Kalman 的 L 更小 → 更信任模型，抑制噪声\n\n');

% 验证 Kalman 的误差动态
A_kal_err = A - L_kal*C;
kalPoles = eig(A_kal_err);
fprintf('  Kalman 等效极点：s₁ = %.2f, s₂ = %.2f\n', kalPoles(1), kalPoles(2));
fprintf('  比 Luenberger 的极点 [-6, -8] 更靠近虚轴 → 收敛慢但更平滑\n\n');

%% ===== 第 2 步：搭建 Simulink 模型（对比实验）=====

mdl = 'tutorial12_kalman';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), save_system(mdl, fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx'])); close_system(mdl, 0); end
new_system(mdl, 'Model');
open_system(mdl);

% ┌──────────────────────── 模型结构 ─────────────────────────┐
% │                                                           │
% │  r ─→ (+) ─→ [Plant] ─→ (+) ─→ y_noisy                   │
% │          ↑                ↑                               │
% │          └── [K] ← x̂_hat ← [Filter] ← y_noisy              │
% │                                                           │
% │  其中 Filter = Luenberger (上半) 或 Kalman (下半)         │
% └───────────────────────────────────────────────────────────┘

% --- 参考输入 ---
add_block('simulink/Sources/Step', [mdl '/Setpoint'], ...
    'Position', [50, 60, 130, 100]);
set_param([mdl '/Setpoint'], 'Time', '0.5', 'Before', '0', 'After', '1');

% --- 过程噪声（加到 Plant 输入）---
add_block('simulink/Sources/Band-Limited White Noise', [mdl '/ProcessNoise_w'], ...
    'Position', [530, 40, 580, 70]);

% --- 测量噪声（加到 Plant 输出）---
add_block('simulink/Sources/Band-Limited White Noise', [mdl '/MeasNoise_v'], ...
    'Position', [530, 130, 580, 160]);

% ===== Plant (真实系统，带噪声) =====
add_block('simulink/Continuous/State-Space', [mdl '/Plant'], ...
    'Position', [210, 70, 280, 120]);
set_param([mdl '/Plant'], ...
    'A', mat2str(A), 'B', mat2str(B), ...
    'C', 'eye(2)', 'D', 'zeros(2,1)', ...
    'X0', '[0.2; 0]');

% Plant 输出 = [x1; x2]
% x1 加上测量噪声 = y_noisy

add_block('simulink/Math Operations/Add', [mdl '/Add_noise'], ...
    'Position', [390, 95, 420, 125]);
set_param([mdl '/Add_noise'], 'Inputs', '|++');

% Demux to get x1 from plant (for adding noise)
add_block('simulink/Signal Routing/Demux', [mdl '/Demux_plant'], ...
    'Position', [310, 65, 330, 125]);
set_param([mdl '/Demux_plant'], 'Outputs', '2');

% ===== 反馈控制器 =====
% 使用真实的 Kalman 估算做反馈
add_block('simulink/Math Operations/Add', [mdl '/Error_r_minus_Kx'], ...
    'Position', [130, 80, 160, 110]);
set_param([mdl '/Error_r_minus_Kx'], 'Inputs', '|+-');

% K feedback gains (用 t10 的极点配置 K)
zeta_d = 0.707;  wn_d = 4.0;
p_ctrl = roots([1, 2*zeta_d*wn_d, wn_d^2]);
K_ctrl = place(A, B, p_ctrl);

% ===== 两套滤波器对比 =====
% 行1：Luenberger (快速但噪声敏感)
% 行2：Kalman (平滑但略慢)

rowY = [280, 400];
L_mats = {L_luen, L_kal};
filterLabels = {'Luenberger', 'Kalman'};

for iFilt = 1:2
    y0 = rowY(iFilt);
    L_mat = L_mats{iFilt};
    tag = filterLabels{iFilt};

    % 观测器状态空间
    A_obs = A - L_mat*C;
    B_obs = [B, L_mat];
    C_obs = eye(2);
    D_obs = zeros(2, 2);

    % Mux for [u; y_noisy]
    add_block('simulink/Signal Routing/Mux', [mdl '/Mux_' tag], ...
        'Position', [440, y0-5, 460, y0+55]);
    set_param([mdl '/Mux_' tag], 'Inputs', '2', 'DisplayOption', 'bar');

    % 观测器
    add_block('simulink/Continuous/State-Space', [mdl '/Filter_' tag], ...
        'Position', [510, y0-5, 570, y0+55]);
    set_param([mdl '/Filter_' tag], ...
        'A', mat2str(A_obs), 'B', mat2str(B_obs), ...
        'C', mat2str(C_obs), 'D', mat2str(D_obs), ...
        'X0', '[0; 0]');

    % Demux 估计状态
    add_block('simulink/Signal Routing/Demux', [mdl '/Demux_' tag], ...
        'Position', [620, y0-10, 640, y0+70]);
    set_param([mdl '/Demux_' tag], 'Outputs', '2');

    % 反馈增益
    add_block('simulink/Math Operations/Gain', [mdl '/K1_' tag], ...
        'Position', [680, y0-5, 715, y0+20]);
    set_param([mdl '/K1_' tag], 'Gain', num2str(K_ctrl(1)));

    add_block('simulink/Math Operations/Gain', [mdl '/K2_' tag], ...
        'Position', [680, y0+35, 715, y0+60]);
    set_param([mdl '/K2_' tag], 'Gain', num2str(K_ctrl(2)));

    % Sum Kx
    add_block('simulink/Math Operations/Add', [mdl '/SumK_' tag], ...
        'Position', [760, y0, 790, y0+55]);
    set_param([mdl '/SumK_' tag], 'Inputs', '|++', 'IconShape', 'round');

    % To Workspace
    add_block('simulink/Sinks/To Workspace', [mdl '/ws_y_' tag], ...
        'Position', [850, y0-5, 900, y0+20]);
    set_param([mdl '/ws_y_' tag], 'VariableName', ['y_' tag]);

    add_block('simulink/Sinks/To Workspace', [mdl '/ws_x2_' tag], ...
        'Position', [850, y0+35, 900, y0+60]);
    set_param([mdl '/ws_x2_' tag], 'VariableName', ['x2_' tag]);

    % --- 连线 ---
    % u → Mux port 1, y_noisy → Mux port 2
    add_line(mdl, 'Error_r_minus_Kx/1', ['Mux_' tag '/1']);
    add_line(mdl, 'Add_noise/1', ['Mux_' tag '/2']);
    add_line(mdl, ['Mux_' tag '/1'], ['Filter_' tag '/1']);
    add_line(mdl, ['Filter_' tag '/1'], ['Demux_' tag '/1']);
    add_line(mdl, ['Demux_' tag '/1'], ['K1_' tag '/1']);
    add_line(mdl, ['Demux_' tag '/2'], ['K2_' tag '/1']);
    add_line(mdl, ['K1_' tag '/1'], ['SumK_' tag '/1']);
    add_line(mdl, ['K2_' tag '/1'], ['SumK_' tag '/2']);

    % To Workspace
    add_line(mdl, ['Demux_' tag '/1'], ['ws_y_' tag '/1']);
    add_line(mdl, ['Demux_' tag '/2'], ['ws_x2_' tag '/1']);
end

% 反馈：用 Kalman 估计做控制（更好的选择）
% 将 Kalman 的 SumK 接到 Error 的负端
add_line(mdl, 'SumK_Kalman/1', 'Error_r_minus_Kx/2');

% ===== 顶层连线 =====

% Setpoint → Error
add_line(mdl, 'Setpoint/1', 'Error_r_minus_Kx/1');

% Error → Plant
add_line(mdl, 'Error_r_minus_Kx/1', 'Plant/1');

% Plant → Demux_plant
add_line(mdl, 'Plant/1', 'Demux_plant/1');

% x1 (Demux_plant/1) → Add_noise
add_line(mdl, 'Demux_plant/1', 'Add_noise/1');

% 测量噪声 → Add_noise
add_line(mdl, 'MeasNoise_v/1', 'Add_noise/2');

% 过程噪声直接加到 Plant 输入（在 Error 和 Plant 之间用 Sum）
% 需要插入一个加法器
add_block('simulink/Math Operations/Add', [mdl '/Add_process_noise'], ...
    'Position', [180, 70, 210, 100]);
set_param([mdl '/Add_process_noise'], 'Inputs', '|++', 'IconShape', 'round');

% 重新布 Error → Plant 的线
delete_line(mdl, 'Error_r_minus_Kx/1', 'Plant/1');
add_line(mdl, 'Error_r_minus_Kx/1', 'Add_process_noise/1');
add_line(mdl, 'ProcessNoise_w/1', 'Add_process_noise/2');
add_line(mdl, 'Add_process_noise/1', 'Plant/1');

% u (不含噪声) 也要送到滤波器 Mux
% 重新连接：需要把 Error_r_minus_Kx 的输出（纯 u）送到 Mux
% 已经连了：'Error_r_minus_Kx/1' → Mux_Luenberger/1、Mux_Kalman/1

% 同时需要 y_noisy → 两个 Mux
% 已经连了：'Add_noise/1' → Mux_Luenberger/2、Mux_Kalman/2

% Scope
add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [920, 70, 970, 150]);
set_param([mdl '/Scope'], 'NumInputPorts', '3');

% x1 true, x1 Luenberger, x1 Kalman
add_line(mdl, 'Demux_plant/1', 'Scope/1');
add_line(mdl, 'Demux_Luenberger/1', 'Scope/2');
add_line(mdl, 'Demux_Kalman/1', 'Scope/3');

% To Workspace for true states
add_block('simulink/Sinks/To Workspace', [mdl '/ws_x1true'], ...
    'Position', [850, 170, 900, 195]);
set_param([mdl '/ws_x1true'], 'VariableName', 'x1_true');
add_block('simulink/Sinks/To Workspace', [mdl '/ws_x2true'], ...
    'Position', [850, 210, 900, 235]);
set_param([mdl '/ws_x2true'], 'VariableName', 'x2_true');
add_block('simulink/Sinks/To Workspace', [mdl '/ws_y_noisy'], ...
    'Position', [850, 250, 900, 275]);
set_param([mdl '/ws_y_noisy'], 'VariableName', 'y_noisy_data');

add_line(mdl, 'Demux_plant/1', 'ws_x1true/1');
add_line(mdl, 'Demux_plant/2', 'ws_x2true/1');
add_line(mdl, 'Add_noise/1', 'ws_y_noisy/1');

Simulink.BlockDiagram.arrangeSystem(mdl);

fprintf('  [OK] 模型搭建完成\n');

%% ===== 第 3 步：运行仿真 =====

fprintf('\n=== 运行仿真 ===\n');
set_param(mdl, 'StopTime', '5');
simOut = sim(mdl);

%% ===== 第 4 步：绘图分析 =====

figure('Name', 't12: Kalman 滤波 vs Luenberger 观测器', ...
    'Position', [50, 50, 1100, 800]);
t = simOut.tout;

% --- 子图 1：位移估计对比 ---
subplot(3, 1, 1);
x1t = getSimData(simOut, 'x1_true', t);
x1L = getSimData(simOut, 'y_Luenberger', t);
x1K = getSimData(simOut, 'y_Kalman', t);
plot(t, x1t, 'k', 'LineWidth', 2); hold on;
plot(t, x1L, 'b--', 'LineWidth', 1.5);
plot(t, x1K, 'r-', 'LineWidth', 2); hold off;
legend('真实 x₁', 'Luenberger 估计', 'Kalman 估计', 'Location', 'southeast');
title('位移估计 — Kalman 更平滑（红线波动更小）');
xlabel('时间 (s)'); ylabel('位移 (m)'); grid on;

% 计算均方根误差
rmse_L = sqrt(mean((x1t - x1L).^2));
rmse_K = sqrt(mean((x1t - x1K).^2));
fprintf('  位移估计 RMSE: Luenberger = %.4f, Kalman = %.4f\n', rmse_L, rmse_K);

% --- 子图 2：速度估计对比 ---
subplot(3, 1, 2);
x2t = getSimData(simOut, 'x2_true', t);
x2L = getSimData(simOut, 'x2_Luenberger', t);
x2K = getSimData(simOut, 'x2_Kalman', t);
plot(t, x2t, 'k', 'LineWidth', 2); hold on;
plot(t, x2L, 'b--', 'LineWidth', 1.5);
plot(t, x2K, 'r-', 'LineWidth', 2); hold off;
legend('真实 x₂', 'Luenberger 估计', 'Kalman 估计', 'Location', 'southeast');
title('速度估计 — Luenberger 噪声放大明显（蓝虚线剧烈抖动）');
xlabel('时间 (s)'); ylabel('速度 (m/s)'); grid on;

rmse_L2 = sqrt(mean((x2t - x2L).^2));
rmse_K2 = sqrt(mean((x2t - x2K).^2));
fprintf('  速度估计 RMSE: Luenberger = %.4f, Kalman = %.4f\n', rmse_L2, rmse_K2);

% --- 子图 3：噪声测量 vs Kalman 输出 ---
subplot(3, 1, 3);
y_n = getSimData(simOut, 'y_noisy_data', t);
plot(t, y_n, 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5); hold on;
plot(t, x1t, 'k:', 'LineWidth', 2);
plot(t, x1K, 'r-', 'LineWidth', 2); hold off;
legend('测量值 y (带噪声)', '真实 x₁', 'Kalman 估计', ...
    'Location', 'southeast');
title('滤波效果 — 灰线是带噪声的测量，红线是 Kalman 滤波后');
xlabel('时间 (s)'); ylabel('幅值'); grid on;

sgtitle('教程 12：Kalman 滤波 — 在噪声中找出真实信号');

%% ===== 第 5 步：理论总结 =====

fprintf('\n========================================\n');
fprintf('  教程 12 完成！\n');
fprintf('========================================\n\n');

fprintf('【理论总结】\n\n');

fprintf('  1. Kalman = Luenberger + 噪声模型\n');
fprintf('     结构完全相同：ẋ̂ = Ax̂ + Bu + L(y - ŷ)\n');
fprintf('     L 的来源不同：极点配置 → Riccati 方程\n\n');

fprintf('  2. Q 和 R 的调节法则\n');
fprintf('     ├─ Q↑ (不相信模型) → L↑ → 快速跟踪，但噪声大\n');
fprintf('     ├─ R↑ (不相信传感器) → L↓ → 平滑，但跟踪慢\n');
fprintf('     └─ Q/R 比例决定信任分配\n\n');

fprintf('  3. 你的结果说明了什么\n');
fprintf('     Luenberger: L=[%.1f, %.1f] → 快速 (极点=-6,-8)\n', L_luen(1), L_luen(2));
fprintf('     Kalman:     L=[%.1f, %.1f] → 平滑 (极点=%.1f,%.1f)\n', ...
    L_kal(1), L_kal(2), kalPoles(1), kalPoles(2));
fprintf('     RMSE 对比：位移 Kalman 胜出 %.0f%%，速度胜出 %.0f%%\n\n', ...
    100*(rmse_L-rmse_K)/rmse_L, 100*(rmse_L2-rmse_K2)/rmse_L2);

fprintf('  4. 工程应用\n');
fprintf('     - 无人机姿态估计 (IMU+GPS 融合) — Kalman 滤波的标准应用\n');
fprintf('     - 电机无传感器控制 — 从电流估计转子位置\n');
fprintf('     - 电池 SOC 估计 — 从电压/电流估计荷电状态\n');
fprintf('     - 目标跟踪 — 从雷达/摄像头噪声数据中跟踪目标\n\n');

fprintf('  5. 扩展 Kalman 滤波 (EKF) — 非线性系统的 Kalman\n');
fprintf('     标准 Kalman 要求线性系统\n');
fprintf('     EKF 对非线性系统做局部线性化 → 每个时刻重新算 A, C\n');
fprintf('     无人机、机器人、SLAM 的核心算法！\n\n');

fprintf('【动手实验】\n\n');
fprintf('  1. 改大 R (传感器更不准)\n');
fprintf('     >> R = 0.01; >> [L,~,~] = lqe(A,G,C,Q_kal,R)\n');
fprintf('     L 变得更小 → 滤波器更不信任传感器\n\n');
fprintf('  2. 改大 Q (相信模型更不准)\n');
fprintf('     >> Q = diag([0.1, 1]); >> [L,~,~] = lqe(A,G,C,Q,R_kal)\n');
fprintf('     L 变得更大 → 滤波器更依赖测量值\n\n');
fprintf('  3. 极端对比\n');
fprintf('     R→0 (传感器完美)：Kalman 退化为 Luenberger 快极点\n');
fprintf('     Q→0 (模型完美)：可以用很小的 L，完全信任模型\n\n');

fprintf('  ┌─────────────────────────────────────┐\n');
fprintf('  │  至此，现代控制理论三件套完成！      │\n');
fprintf('  │  t10 状态反馈 (K) — 控制设计        │\n');
fprintf('  │  t11 状态观测器 (L) — 状态估计       │\n');
fprintf('  │  t12 Kalman 滤波 — 带噪声的最优估计  │\n');
fprintf('  │                                     │\n');
fprintf('  │  下一阶段：机电系统建模              │\n');
fprintf('  │  t13: DC 电机建模与控制              │\n');
fprintf('  └─────────────────────────────────────┘\n');

% ============================================================
% 辅助函数
