%% ============================================================
% 教程 13：DC 电机建模与控制
%
% 【物理模型】
%
%      电域 (KVL)                机械域 (Newton)
%   L·di/dt + R·i = u - e     J·dω/dt + b·ω = T
%   反电动势: e = Ke·ω         电磁转矩: T = Kt·i
%
%   → 耦合：电流产生转矩 (Kt·i)，转速产生反电动势 (Ke·ω)
%
% 【状态空间】
%   状态: x₁ = i (电流), x₂ = ω (转速)
%
%   d/dt[i] = [-R/L   -Ke/L ]·[i] + [1/L]·u
%   d/dt[ω]   [Kt/J    -b/J ] [ω]   [ 0 ]
%
%        y  = [ 0     1    ]·[i]      (测量转速)
%                            [ω]
%
% 【电机参数】小型直流电机（类似编码器电机/机器人关节电机）
%   R = 2.0 Ω      电枢电阻
%   L = 0.002 H    电枢电感
%   Ke = 0.015 V/(rad/s)  反电动势常数
%   Kt = 0.015 N·m/A      转矩常数 (SI: Kt = Ke)
%   J = 0.0005 kg·m²      转子惯量
%   b = 0.00001 N·m/(rad/s) 粘滞摩擦(很小)
%
% 【本课目标】
%   1. DC 电机状态空间建模
%   2. 开环特性分析（时间常数、稳态）
%   3. 级联 PI 控制（工业标准：电流环 + 速度环）
%   4. LQR 状态反馈控制（现代方法，连接 t10）
% ============================================================

clear; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

%% ===== 参数定义 =====

R  = 2.0;      % 电阻 (Ω)
L  = 0.002;    % 电感 (H)
Ke = 0.015;    % 反电动势常数 V/(rad/s)
Kt = 0.015;    % 转矩常数 N·m/A (SI: =Ke)
J  = 0.0005;   % 惯量 (kg·m²)
b  = 0.00001;  % 摩擦系数 N·m/(rad/s)

% 状态空间矩阵
A_motor = [-R/L,   -Ke/L ;
            Kt/J,    -b/J ];

B_motor = [ 1/L ;
             0  ];

C_motor = [ 0, 1 ];   % 测量转速
D_motor = 0;

fprintf('============================================\n');
fprintf('  教程 13：DC 电机建模与控制\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：开环分析 =====

% 电气时间常数 vs 机械时间常数
tau_e = L/R;        % 电气时间常数
tau_m = J/b;        % 机械时间常数 (摩擦很小 → 很大)

openPoles = eig(A_motor);

fprintf('【开环特性分析】\n');
fprintf('  电气时间常数 τ_e = L/R = %.3f ms\n', tau_e*1000);
fprintf('  机械时间常数 τ_m = J/b = %.1f s\n\n', tau_m);

fprintf('  开环极点：\n');
fprintf('    s₁ = %.1f  (电气极点 — 快速衰减)\n', openPoles(1));
fprintf('    s₂ = %.2f (机械极点 — 近似纯积分)\n\n', openPoles(2));

% 稳态分析
% 稳态时 di/dt=0, dω/dt=0:
%   0 = -R·i - Ke·ω + u  →  i = (u - Ke·ω)/R
%   0 = Kt·i - b·ω       →  ω = Kt·i/b
%   联立: ω_ss = Kt·u / (R·b + Ke·Kt) ≈ u/Ke  (因为 b≈0)
omega_ss_per_volt = Kt / (R*b + Ke*Kt);
fprintf('  稳态转速：约 %.1f rad/s per Volt (≈ %.0f rpm/V)\n', ...
    omega_ss_per_volt, omega_ss_per_volt * 60/(2*pi));

% DC gain from voltage to speed
G_dc = tf(ss(A_motor, B_motor, C_motor, D_motor));
[dc_gain, ~] = dcgain(G_dc);
fprintf('  DC gain: %.2f (rad/s)/V\n\n', dc_gain);

fprintf('  问题：开环响应太依赖参数变化，需要反馈控制。\n\n');

%% ===== 第 2 步：级联 PI 控制设计（工业标准）= ====

% ┌──────────────── 级联结构 ────────────────┐
% │                                           │
% │  ω_ref → [PI_speed] → i_ref → [PI_cur] → u → [Motor] → ω
% │              ↑                                    │
% │              └─────────── ω ──────────────────────┘
% │                         ↑                         │
% │              ┌────────── i ──────────────┐        │
% │              └───────────────────────────┘        │
% └───────────────────────────────────────────────────┘

% 电流环（内环）— 带宽 ~500 Hz
% 用极点对消：PI 零点 = 电气极点
wc_current = 2*pi*500;  % 电流环带宽 (rad/s)
Kp_cur = wc_current * L;
Ki_cur = wc_current * R;

% 速度环（外环）— 带宽 ~20 Hz
% 将电流闭环近似为 1 阶系统后设计
wc_speed = 2*pi*20;     % 速度环带宽 (rad/s)
Kp_spd = wc_speed * J / Kt;
Ki_spd = wc_speed * b / Kt;
% 通常 b 很小 → Ki 可以稍微大一点以增加刚度
Ki_spd = Ki_spd + 0.1 * Kp_spd;

fprintf('【级联 PI 控制器设计】\n');
fprintf('  电流环 (内环, ~500Hz): Kp=%.3f, Ki=%.1f\n', Kp_cur, Ki_cur);
fprintf('  速度环 (外环, ~20Hz):  Kp=%.4f, Ki=%.4f\n\n', Kp_spd, Ki_spd);

%% ===== 第 3 步：LQR 状态反馈设计（现代方法）=====

% Q 矩阵：速度精度最重要
Q = diag([0.1, 100]);   % [电流权重, 速度权重]
R_lqr = 1;              % 电压成本
[K_lqr, S, E_lqr] = lqr(A_motor, B_motor, Q, R_lqr);

fprintf('【LQR 状态反馈设计】\n');
fprintf('  Q = diag([0.1, 100]), R = 1 (速度精度优先)\n');
fprintf('  K_lqr = [ %.4f,  %.4f ]\n', K_lqr(1), K_lqr(2));
fprintf('  闭环极点: %.1f, %.1f (比开环快得多)\n\n', E_lqr(1), E_lqr(2));

%% ===== 第 4 步：搭建 Simulink =====

mdl = 'tutorial13_dc_motor';
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

% --- 参考输入 ---
add_block('simulink/Sources/Step', [mdl '/Setpoint rpm'], ...
    'Position', [50, 60, 130, 100]);
set_param([mdl '/Setpoint rpm'], ...
    'Time', '0.01', 'Before', '0', 'After', '100');  % 100 rad/s ≈ 955 rpm

% ===== 行 1：开环（无控制）=====
add_block('simulink/Continuous/State-Space', [mdl '/Motor_OL'], ...
    'Position', [220, 60, 290, 110]);
set_param([mdl '/Motor_OL'], ...
    'A', mat2str(A_motor), 'B', mat2str(B_motor), ...
    'C', mat2str(C_motor), 'D', mat2str(D_motor), ...
    'X0', '[0; 0]');

% 速度 → ToWS
add_block('simulink/Sinks/To Workspace', [mdl '/ws_OL'], ...
    'Position', [360, 70, 420, 100]);
set_param([mdl '/ws_OL'], 'VariableName', 'omega_OL');

add_line(mdl, 'Setpoint rpm/1', 'Motor_OL/1');
add_line(mdl, 'Motor_OL/1', 'ws_OL/1');

% ===== 行 2：级联 PI（用增益+积分器手搭，避开 PID block 版本差异）=====

% --- 速度误差 ---
add_block('simulink/Math Operations/Add', [mdl '/Err_Spd'], ...
    'Position', [120, 180, 150, 210]);
set_param([mdl '/Err_Spd'], 'Inputs', '|+-');

% 速度环 PI: Kp_spd * e_spd + Ki_spd * ∫e_spd
add_block('simulink/Math Operations/Gain', [mdl '/Kp_spd'], ...
    'Position', [190, 160, 230, 190]);
set_param([mdl '/Kp_spd'], 'Gain', num2str(Kp_spd));

add_block('simulink/Math Operations/Gain', [mdl '/Ki_spd'], ...
    'Position', [190, 200, 230, 230]);
set_param([mdl '/Ki_spd'], 'Gain', num2str(Ki_spd));

add_block('simulink/Continuous/Integrator', [mdl '/Int_spd'], ...
    'Position', [260, 200, 310, 240]);
set_param([mdl '/Int_spd'], 'InitialCondition', '0');

add_block('simulink/Math Operations/Add', [mdl '/Sum_spd'], ...
    'Position', [350, 175, 380, 215]);
set_param([mdl '/Sum_spd'], 'Inputs', '|++', 'IconShape', 'round');

% --- 电流误差 ---
add_block('simulink/Math Operations/Add', [mdl '/Err_Cur'], ...
    'Position', [430, 180, 460, 210]);
set_param([mdl '/Err_Cur'], 'Inputs', '|+-');

% 电流环 PI: Kp_cur * e_cur + Ki_cur * ∫e_cur
add_block('simulink/Math Operations/Gain', [mdl '/Kp_cur'], ...
    'Position', [500, 160, 540, 190]);
set_param([mdl '/Kp_cur'], 'Gain', num2str(Kp_cur));

add_block('simulink/Math Operations/Gain', [mdl '/Ki_cur'], ...
    'Position', [500, 200, 540, 230]);
set_param([mdl '/Ki_cur'], 'Gain', num2str(Ki_cur));

add_block('simulink/Continuous/Integrator', [mdl '/Int_cur'], ...
    'Position', [570, 200, 620, 240]);
set_param([mdl '/Int_cur'], 'InitialCondition', '0');

add_block('simulink/Math Operations/Add', [mdl '/Sum_cur'], ...
    'Position', [660, 175, 690, 215]);
set_param([mdl '/Sum_cur'], 'Inputs', '|++', 'IconShape', 'round');

% 电压限幅（保护硬件）
add_block('simulink/Discontinuities/Saturation', [mdl '/Sat_V'], ...
    'Position', [730, 180, 760, 210]);
set_param([mdl '/Sat_V'], 'UpperLimit', '12', 'LowerLimit', '-12');

% Motor for PI control — output both i and ω
add_block('simulink/Continuous/State-Space', [mdl '/Motor_PI'], ...
    'Position', [800, 180, 870, 230]);
set_param([mdl '/Motor_PI'], ...
    'A', mat2str(A_motor), 'B', mat2str(B_motor), ...
    'C', 'eye(2)', 'D', 'zeros(2,1)', ...
    'X0', '[0; 0]');

% Demux for current and speed
add_block('simulink/Signal Routing/Demux', [mdl '/Demux_PI'], ...
    'Position', [910, 175, 930, 235]);
set_param([mdl '/Demux_PI'], 'Outputs', '2');

% ToWS
add_block('simulink/Sinks/To Workspace', [mdl '/ws_PI'], ...
    'Position', [970, 180, 1030, 210]);
set_param([mdl '/ws_PI'], 'VariableName', 'omega_PI');

add_block('simulink/Sinks/To Workspace', [mdl '/ws_PI_cur'], ...
    'Position', [970, 220, 1030, 250]);
set_param([mdl '/ws_PI_cur'], 'VariableName', 'i_PI');

% --- PI 速度环连线 ---
add_line(mdl, 'Setpoint rpm/1', 'Err_Spd/1');

% 速度误差 → Kp 和 Ki
add_line(mdl, 'Err_Spd/1', 'Kp_spd/1');
add_line(mdl, 'Err_Spd/1', 'Ki_spd/1');

% Ki_spd → Integrator → Sum
add_line(mdl, 'Ki_spd/1', 'Int_spd/1');
add_line(mdl, 'Kp_spd/1', 'Sum_spd/1');
add_line(mdl, 'Int_spd/1', 'Sum_spd/2');

% Sum_spd → i_ref → Err_Cur (+)
add_line(mdl, 'Sum_spd/1', 'Err_Cur/1');

% --- PI 电流环连线 ---
add_line(mdl, 'Err_Cur/1', 'Kp_cur/1');
add_line(mdl, 'Err_Cur/1', 'Ki_cur/1');

add_line(mdl, 'Ki_cur/1', 'Int_cur/1');
add_line(mdl, 'Kp_cur/1', 'Sum_cur/1');
add_line(mdl, 'Int_cur/1', 'Sum_cur/2');

% Sum_cur → Sat → Motor
add_line(mdl, 'Sum_cur/1', 'Sat_V/1');
add_line(mdl, 'Sat_V/1', 'Motor_PI/1');

% Motor → Demux
add_line(mdl, 'Motor_PI/1', 'Demux_PI/1');

% --- 反馈连线 ---
% speed feedback (Demux_PI port 2 = ω) → Err_Spd (-)
add_line(mdl, 'Demux_PI/2', 'Err_Spd/2');

% current feedback (Demux_PI port 1 = i) → Err_Cur (-)
add_line(mdl, 'Demux_PI/1', 'Err_Cur/2');

% --- To Workspace ---
add_line(mdl, 'Demux_PI/2', 'ws_PI/1');       % speed
add_line(mdl, 'Demux_PI/1', 'ws_PI_cur/1');   % current

% ===== 行 3：LQR 状态反馈 =====

% Error: r - ω
add_block('simulink/Math Operations/Add', [mdl '/Err_LQR'], ...
    'Position', [120, 320, 150, 350]);
set_param([mdl '/Err_LQR'], 'Inputs', '|+-');

% Integrator for zero steady-state error (integral action in LQR)
add_block('simulink/Continuous/Integrator', [mdl '/Int_Err'], ...
    'Position', [200, 340, 250, 380]);
set_param([mdl '/Int_Err'], 'InitialCondition', '0');

% Gains
add_block('simulink/Math Operations/Gain', [mdl '/K_lqr1'], ...
    'Position', [400, 320, 440, 355]);
set_param([mdl '/K_lqr1'], 'Gain', num2str(K_lqr(1)));

add_block('simulink/Math Operations/Gain', [mdl '/K_lqr2'], ...
    'Position', [400, 370, 440, 395]);
set_param([mdl '/K_lqr2'], 'Gain', num2str(K_lqr(2)));

% Integral gain (for steady-state error elimination)
Ki_lqr = 5;  % 手动调出
add_block('simulink/Math Operations/Gain', [mdl '/Ki_int'], ...
    'Position', [320, 340, 360, 370]);
set_param([mdl '/Ki_int'], 'Gain', num2str(Ki_lqr));

% Sum for control law: u = -K1*i - K2*ω + Ki*∫e  + ω_ref feedforward
add_block('simulink/Math Operations/Add', [mdl '/Sum_u'], ...
    'Position', [500, 325, 535, 395]);
set_param([mdl '/Sum_u'], 'Inputs', '|+++-', 'IconShape', 'round');

% Feedforward: u_ff = ω_ref * (Ke + R*b/Kt) ≈ ω_ref * Ke (简化)
add_block('simulink/Math Operations/Gain', [mdl '/FF_Gain'], ...
    'Position', [350, 280, 400, 310]);
set_param([mdl '/FF_Gain'], 'Gain', num2str(Ke));  % u_ff ≈ Ke·ω_ref

% Sum for feedforward + feedback
add_block('simulink/Math Operations/Add', [mdl '/Sum_FF'], ...
    'Position', [580, 320, 610, 355]);
set_param([mdl '/Sum_FF'], 'Inputs', '|++');

% Motor for LQR
add_block('simulink/Continuous/State-Space', [mdl '/Motor_LQR'], ...
    'Position', [660, 320, 730, 370]);
set_param([mdl '/Motor_LQR'], ...
    'A', mat2str(A_motor), 'B', mat2str(B_motor), ...
    'C', 'eye(2)', 'D', 'zeros(2,1)', ...
    'X0', '[0; 0]');

% Demux LQR
add_block('simulink/Signal Routing/Demux', [mdl '/Demux_LQR'], ...
    'Position', [770, 315, 790, 375]);
set_param([mdl '/Demux_LQR'], 'Outputs', '2');

% ToWS
add_block('simulink/Sinks/To Workspace', [mdl '/ws_LQR'], ...
    'Position', [830, 330, 890, 360]);
set_param([mdl '/ws_LQR'], 'VariableName', 'omega_LQR');

% --- 连线 ---
add_line(mdl, 'Setpoint rpm/1', 'Err_LQR/1');
add_line(mdl, 'Demux_LQR/2', 'Err_LQR/2');   % speed feedback
add_line(mdl, 'Err_LQR/1', 'Int_Err/1');
add_line(mdl, 'Int_Err/1', 'Ki_int/1');

% Current → K1
add_line(mdl, 'Demux_LQR/1', 'K_lqr1/1');
% Speed → K2
add_line(mdl, 'Demux_LQR/2', 'K_lqr2/1');

% Sum_u: port1=Ki*∫e, port2=feedforward, port3=K1*i, port4=K2*ω (both negative)
% Using |+++- means ports 1,2,3 are +, port 4 is -
add_line(mdl, 'Ki_int/1', 'Sum_u/1');

% Feedforward
add_line(mdl, 'Setpoint rpm/1', 'FF_Gain/1');
add_line(mdl, 'FF_Gain/1', 'Sum_u/2');

% K1*current and K2*speed as negative (ports 3 and 4)
% |+++- means port3 is +, port4 is -. For negative feedback we'd want both negative.
% Let me fix: I should use a different structure.
% Actually the sign convention is: u = Ke*ω_ref + Ki*∫(ω_ref-ω) - K1*i - K2*ω
% Let me redo with proper signs.

delete_block([mdl '/Sum_u']);

% New structure: separate positive and negative terms
% u_pos = Ke*ω_ref + Ki*∫(ω_ref-ω)
% u_neg = -K1*i - K2*ω
% u = u_pos + u_neg

add_block('simulink/Math Operations/Add', [mdl '/Sum_pos'], ...
    'Position', [520, 280, 550, 310]);
set_param([mdl '/Sum_pos'], 'Inputs', '|++', 'IconShape', 'round');

add_block('simulink/Math Operations/Add', [mdl '/Sum_neg'], ...
    'Position', [520, 370, 550, 400]);
set_param([mdl '/Sum_neg'], 'Inputs', '|++', 'IconShape', 'round');

add_block('simulink/Math Operations/Add', [mdl '/Sum_total'], ...
    'Position', [600, 330, 630, 370]);
set_param([mdl '/Sum_total'], 'Inputs', '|+-', 'IconShape', 'round');

% Positive terms
add_line(mdl, 'Ki_int/1', 'Sum_pos/1');
add_line(mdl, 'FF_Gain/1', 'Sum_pos/2');

% Negative terms
add_line(mdl, 'K_lqr1/1', 'Sum_neg/1');
add_line(mdl, 'K_lqr2/1', 'Sum_neg/2');

% Total: u_pos - u_neg
add_line(mdl, 'Sum_pos/1', 'Sum_total/1');
add_line(mdl, 'Sum_neg/1', 'Sum_total/2');

% To Motor
add_line(mdl, 'Sum_total/1', 'Motor_LQR/1');
add_line(mdl, 'Motor_LQR/1', 'Demux_LQR/1');
add_line(mdl, 'Demux_LQR/2', 'ws_LQR/1');

% --- 电压限幅 LQR ---
add_block('simulink/Discontinuities/Saturation', [mdl '/Sat_V_LQR'], ...
    'Position', [570, 320, 600, 350]);
set_param([mdl '/Sat_V_LQR'], ...
    'UpperLimit', '12', 'LowerLimit', '-12');

delete_line(mdl, 'Sum_total/1', 'Motor_LQR/1');
add_line(mdl, 'Sum_total/1', 'Sat_V_LQR/1');
add_line(mdl, 'Sat_V_LQR/1', 'Motor_LQR/1');

Simulink.BlockDiagram.arrangeSystem(mdl);
fprintf('  [OK] 模型搭建完成\n');

%% ===== 第 5 步：运行仿真 =====

fprintf('\n=== 运行仿真 ===\n');
set_param(mdl, 'StopTime', '1.5');
simOut = sim(mdl);

%% ===== 第 6 步：结果分析 =====

figure('Name', 't13: DC 电机建模与控制', 'Position', [50, 50, 1000, 800]);
t = simOut.tout;

% --- 子图 1：转速响应 ---
subplot(3, 1, 1);
w_OL = getSimData(simOut, 'omega_OL', t);
w_PI = getSimData(simOut, 'omega_PI', t);
w_LQR = getSimData(simOut, 'omega_LQR', t);

plot(t, w_OL, 'k--', 'LineWidth', 1.5); hold on;
plot(t, w_PI, 'b', 'LineWidth', 2);
plot(t, w_LQR, 'r', 'LineWidth', 2);
yline(100, ':', 'Color', [0.5 0.5 0.5]);
hold off;
legend('开环', '级联 PI', 'LQR + 积分', '目标 100 rad/s', ...
    'Location', 'southeast');
title('转速响应 — 三种控制方案对比');
xlabel('时间 (s)'); ylabel('转速 ω (rad/s)'); grid on;

idx_s = find(abs(w_PI-100) < 2, 1, 'first');
if ~isempty(idx_s)
    fprintf('  PI 稳定时间: ~%.3f s\n', t(idx_s));
else
    fprintf('  PI 未在仿真时间内稳定\n');
end

% --- 子图 2：电流响应 (PI) ---
subplot(3, 1, 2);
i_PI_dat = getSimData(simOut, 'i_PI', t);
plot(t, i_PI_dat, 'b', 'LineWidth', 2);
title('电流响应 (级联 PI) — 启动时大电流加速，稳态电流很小');
xlabel('时间 (s)'); ylabel('电流 i (A)'); grid on;

% --- 子图 3：三种方法的控制特点 ---
subplot(3, 1, 3);
text(0.1, 0.8, '开环: 缓慢上升，依赖电机自身特性');
text(0.1, 0.5, '级联 PI: 快速响应 + 电流保护 (工业标准)');
text(0.1, 0.2, 'LQR+积分: 最优反馈 + 零稳态误差 (现代方法)');
axis off;

sgtitle('教程 13：DC 电机建模与控制');

%% ===== 第 7 步：总结 =====

fprintf('\n========================================\n');
fprintf('  教程 13 完成！\n');
fprintf('========================================\n\n');

fprintf('【DC 电机控制 — 方法对比】\n\n');

fprintf('  1. 开环控制\n');
fprintf('     u = ω_ref * Ke  (固定电压)\n');
fprintf('     缺点：负载变化 → 转速漂移；无电流保护\n\n');

fprintf('  2. 级联 PI (工业标准)\n');
fprintf('     内环电流 PI → 外环速度 PI\n');
fprintf('     优点：电流限幅保护硬件、抗负载扰动、调试直观\n');
fprintf('     每个电机驱动器都在干这件事\n\n');

fprintf('  3. LQR 状态反馈 + 积分\n');
fprintf('     u = Ke·ω_ref + Ki·∫e - K1·i - K2·ω\n');
fprintf('     优点：理论最优、MIMO 天然支持\n');
fprintf('     缺点：需要电流传感器（或观测器）\n\n');

fprintf('  4. 实际应用组合\n');
fprintf('     高端伺服：LQR/Kalman 观测器 + 前馈 + 抗饱和\n');
fprintf('     通用驱动：级联 PI (简单够用)\n');
fprintf('     无传感器 FOC：观测器估计转子位置 (t14 预告)\n\n');

fprintf('  5. 与实际机电系统的连接\n');
fprintf('     IMU (传感器) → Kalman (估计姿态)\n');
fprintf('     → 控制器 (算指令) → 电机 (执行动作)\n');
fprintf('     传感器 + 控制器 + 执行器 = 完整机电系统！\n\n');

fprintf('【动手实验】\n');
fprintf('  1. 增大 J (惯量变大)，观察响应变慢\n');
fprintf('  2. 改 PI 参数，看超调和振荡\n');
fprintf('  3. 把 LQR 的 Q 矩阵调大 → K 变大 → 响应更快\n');
fprintf('  4. 去掉前馈 FF，看稳态误差出现\n\n');

fprintf('  下一课预告：t14 — PMSM + FOC 矢量控制\n');
fprintf('  无刷电机 (BLDC/PMSM) 是机器人和新能源的核心\n');
fprintf('  FOC = Clarke/Park 变换 + 电流环 + 速度环\n');
fprintf('  核心还是 PI + 状态空间！\n');

