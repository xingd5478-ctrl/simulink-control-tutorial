%% ============================================================
% 教程 20：模型预测控制 (MPC) — 带约束的最优控制
%
% 【MPC 是什么？】
%   在每个采样时刻，求解一个有限时域优化问题：
%     "未来 N 步内，控制序列怎么走，使得：
%       1. 输出跟踪目标
%       2. 控制量尽量小
%       3. 满足所有约束（输入/输出/状态）"
%   执行第一步，下一时刻重新优化 → 滚动时域控制
%
% 【MPC vs LQR】
%   LQR: 无限时域、无约束、解析解
%   MPC: 有限时域、有约束、数值求解 (QP)
%
% 【应用场景】
%   化工过程控制、自动驾驶、无人机、机器人
%   → 任何有约束的多变量系统
%
% 【本课内容】
%   1. 手工实现简单 MPC（QP 求解）
%   2. 约束处理：输入饱和、速率限制
%   3. 预测时域和控制时域的调参
%   4. MPC Toolbox 快速原型
% ============================================================

clear; close all;

fprintf('============================================\n');
fprintf('  教程 20：模型预测控制 (MPC)\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：被控对象建模 =====

% 二阶系统（沿用 t10 的质量-弹簧-阻尼）
m = 1.0;  c = 0.5;  k = 4.0;

A = [   0,     1  ;
      -k/m,  -c/m ];

B = [  0  ;
      1/m ];

C = [ 1, 0 ];
D = 0;

sys_c = ss(A, B, C, D);

% 离散化
Ts = 0.1;  % 100ms 采样
sys_d = c2d(sys_c, Ts, 'zoh');
A_d = sys_d.A;
B_d = sys_d.B;
C_d = sys_d.C;

nx = size(A_d, 1);  % 状态维数
nu = size(B_d, 2);  % 输入维数

fprintf('【被控对象】质量-弹簧-阻尼 (Ts=%.0fms)\n', Ts*1000);
fprintf('  状态: [位移, 速度]\n');
fprintf('  输入: 力 (N)\n');
fprintf('  约束: |u| ≤ 10 N, |Δu| ≤ 2 N/step\n\n');

%% ===== 第 2 步：MPC 参数设置 =====

Np = 15;   % 预测时域 (prediction horizon)
Nc = 5;    % 控制时域 (control horizon)

% 代价函数：J = Σ(x'Qx + u'Ru) + x_N'Px_N
Q_mpc = diag([10, 0.1]);   % 状态权重
R_mpc = 0.01;               % 控制权重

% 约束
u_max = 10;   u_min = -10;   % 输入饱和
du_max = 2;                  % 速率限制

fprintf('【MPC 参数】\n');
fprintf('  预测时域 Np = %d\n', Np);
fprintf('  控制时域 Nc = %d\n', Nc);
fprintf('  Nc < Np → 后面 Np-Nc 步控制量保持不变\n\n');

%% ===== 第 3 步：手工实现无约束 MPC =====

% 构造预测矩阵
% X = F*x_k + Phi*U
% 其中 U = [u_k; u_{k+1}; ...; u_{k+Nc-1}]

F = zeros(nx*Np, nx);
Phi = zeros(nx*Np, nu*Nc);

A_pow = eye(nx);
for i = 1:Np
    A_pow = A_d * A_pow;  % 避免循环中的临时变量
end

% 重新正确构造
for i = 1:Np
    F((i-1)*nx+1:i*nx, :) = A_d^i;
    for j = 1:min(i, Nc)
        Phi((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = A_d^(i-j) * B_d;
    end
end

% 构造代价函数矩阵 H 和 f
Q_bar = kron(eye(Np), Q_mpc);
R_bar = kron(eye(Nc), R_mpc);

H = Phi' * Q_bar * Phi + R_bar;
H = (H + H') / 2;  % 确保对称

fprintf('【手工 MPC 求解】\n');
fprintf('  H 矩阵维数: %d x %d\n', size(H));

% 无约束解：U* = -H^{-1} * Phi' * Q_bar * F * x
x0 = [0.5; 0];  % 初始状态：位移 0.5m
N_steps = 100;

x_hist = zeros(nx, N_steps);
u_hist = zeros(nu, N_steps);
x = x0;

for k = 1:N_steps
    % 无约束 QP 解析解
    f = 2 * Phi' * Q_bar * F * x;
    U_opt = -H \ f;

    u = U_opt(1);  % 只执行第一步
    u_hist(k) = u;

    % 系统更新
    x = A_d * x + B_d * u;
    x_hist(:, k) = x;
end

t_vec = (0:N_steps-1) * Ts;

%% ===== 第 4 步：带约束 MPC =====

% 约束形式：Aineq * U ≤ bineq
% 输入饱和：u_min ≤ u_i ≤ u_max
% 速率限制：|u_{i+1} - u_i| ≤ du_max

Aineq = [];
bineq = [];

% 输入上下界
for i = 1:Nc
    Aineq = [Aineq;  zeros(1, i-1),  1, zeros(1, Nc-i)];
    bineq = [bineq;  u_max];
    Aineq = [Aineq;  zeros(1, i-1), -1, zeros(1, Nc-i)];
    bineq = [bineq; -u_min];
end

% 速率限制
for i = 1:Nc-1
    Aineq = [Aineq; zeros(1, i-1), -1, 1, zeros(1, Nc-i-1)];
    bineq = [bineq; du_max];
    Aineq = [Aineq; zeros(1, i-1), 1, -1, zeros(1, Nc-i-1)];
    bineq = [bineq; du_max];
end

% 带约束 MPC 仿真
x = x0;
x_c_hist = zeros(nx, N_steps);
u_c_hist = zeros(nu, N_steps);

for k = 1:N_steps
    f = 2 * Phi' * Q_bar * F * x;

    % 用 quadprog 求解带约束 QP
    options = optimoptions('quadprog', 'Display', 'off', ...
        'Algorithm', 'interior-point-convex');
    [U_opt, ~, exitflag] = quadprog(H, f, Aineq, bineq, ...
        [], [], [], [], [], options);

    if exitflag < 0
        warning('QP 求解失败 @ step %d, 使用备选方案', k);
        U_opt = -H \ f;  % 回退到无约束解
    end

    u = U_opt(1);
    u_c_hist(k) = u;
    x = A_d * x + B_d * u;
    x_c_hist(:, k) = x;
end

%% ===== 第 5 步：LQR 对比 =====

Q_lqr = diag([10, 0.1]);
R_lqr = 0.01;
K_lqr = dlqr(A_d, B_d, Q_lqr, R_lqr);

x = x0;
x_lqr_hist = zeros(nx, N_steps);
u_lqr_hist = zeros(nu, N_steps);

for k = 1:N_steps
    u = -K_lqr * x;
    u_lqr_hist(k) = max(min(u, u_max), u_min);  % 饱和
    x = A_d * x + B_d * u_lqr_hist(k);
    x_lqr_hist(:, k) = x;
end

%% ===== 第 6 步：绘图对比 =====

figure('Name', 't20: MPC vs LQR', ...
    'Position', [50, 50, 1000, 700]);

% 位移对比
subplot(2,2,1); hold on;
stairs(t_vec, x_hist(1,:), 'b', 'LineWidth', 1);
stairs(t_vec, x_c_hist(1,:), 'r', 'LineWidth', 1.5);
stairs(t_vec, x_lqr_hist(1,:), 'Color', [0 0.6 0], 'LineWidth', 1);
yline(0, 'k--');
legend('MPC 无约束', 'MPC 带约束', 'LQR', 'Location', 'best');
title('位移 x1 对比'); xlabel('时间 (s)'); ylabel('位移 (m)'); grid on;

% 控制力对比
subplot(2,2,2); hold on;
stairs(t_vec, u_hist, 'b', 'LineWidth', 1);
stairs(t_vec, u_c_hist, 'r', 'LineWidth', 1.5);
stairs(t_vec, u_lqr_hist, 'Color', [0 0.6 0], 'LineWidth', 1);
yline(u_max, 'k--'); yline(u_min, 'k--');
legend('无约束', '带约束', 'LQR+饱和', 'Location', 'best');
title('控制力 u 对比'); xlabel('时间 (s)'); ylabel('力 (N)'); grid on;

% 控制力变化率
subplot(2,2,3); hold on;
du_mpc = [diff(u_hist), 0];
du_c = [diff(u_c_hist), 0];
du_lqr = [diff(u_lqr_hist), 0];
stairs(t_vec, du_mpc, 'b', 'LineWidth', 1);
stairs(t_vec, du_c, 'r', 'LineWidth', 1.5);
stairs(t_vec, du_lqr, 'Color', [0 0.6 0], 'LineWidth', 1);
yline(du_max, 'k--'); yline(-du_max, 'k--');
legend('无约束', '带约束', 'LQR', 'Location', 'best');
title('控制力变化率 Δu'); xlabel('时间 (s)'); ylabel('Δu (N/step)'); grid on;

% 预测时域影响
subplot(2,2,4); hold on;
Np_list = [5, 15, 30];
colors = lines(length(Np_list));
for ip = 1:length(Np_list)
    Np_i = Np_list(ip);
    % 快速重建预测矩阵
    F_i = zeros(nx*Np_i, nx);
    Phi_i = zeros(nx*Np_i, nu*Nc);
    for i = 1:Np_i
        F_i((i-1)*nx+1:i*nx, :) = A_d^i;
        for j = 1:min(i, Nc)
            Phi_i((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = A_d^(i-j) * B_d;
        end
    end
    Q_bar_i = kron(eye(Np_i), Q_mpc);
    R_bar_i = kron(eye(Nc), R_mpc);
    H_i = Phi_i' * Q_bar_i * Phi_i + R_bar_i;
    H_i = (H_i + H_i') / 2;

    x_i = x0;
    x_hi = zeros(nx, N_steps);
    for k = 1:N_steps
        f_i = 2 * Phi_i' * Q_bar_i * F_i * x_i;
        u_i = -(H_i \ f_i);
        u_i = u_i(1);
        x_i = A_d * x_i + B_d * u_i;
        x_hi(:, k) = x_i;
    end
    stairs(t_vec, x_hi(1,:), 'Color', colors(ip,:), 'LineWidth', 1.5);
end
legend('Np=5', 'Np=15 (默认)', 'Np=30', 'Location', 'best');
title('预测时域 Np 对性能的影响'); xlabel('时间 (s)'); ylabel('位移 (m)'); grid on;

%% ===== 第 7 步：MPC Toolbox 演示 =====

fprintf('\n【MPC Toolbox 快速演示】\n');
try
    mpc_obj = mpc(sys_c, Ts, Np, Nc);
    mpc_obj.Weights.ManipulatedVariables = R_mpc;
    mpc_obj.Weights.OutputVariables = Q_mpc(1);
    mpc_obj.MV.Min = u_min;
    mpc_obj.MV.Max = u_max;
    fprintf('  MPC 对象创建成功\n');
    fprintf('  采样时间 = %.2f s\n', mpc_obj.Ts);
    fprintf('  预测时域 = %d, 控制时域 = %d\n', mpc_obj.PredictionHorizon, ...
        mpc_obj.ControlHorizon);
catch ME
    fprintf('  MPC Toolbox demo skipped: %s\n', ME.message);
end

fprintf('\n========================================\n');
fprintf('  教程 20 完成！\n');
fprintf('========================================\n\n');

fprintf('【MPC 调参指南】\n');
fprintf('  预测时域 Np：覆盖系统主要动态 (Np×Ts ≥ 5×τ)\n');
fprintf('  控制时域 Nc：通常取 Np/3 ~ Np/5\n');
fprintf('  输出权重：大 → 跟踪更紧 → 控制更激进\n');
fprintf('  输入权重：大 → 控制更平滑 → 跟踪变慢\n');
fprintf('  约束越紧 → 优化越难 → QP 可能无解\n\n');

fprintf('【MPC 的代价】\n');
fprintf('  每次迭代求解一个 QP → 计算量大\n');
fprintf('  QP 维数 = Nc × nu\n');
fprintf('  对于快速系统 (Ts<1ms)，MPC 可能来不及算\n');
fprintf('  → 工业上用显式 MPC：离线求解 → 查表在线\n');
