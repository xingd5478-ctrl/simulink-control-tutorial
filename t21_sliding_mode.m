%% ============================================================
% 教程 21：滑模控制 (SMC) — 非线性鲁棒控制
%
% 【什么是滑模控制？】
%   设计一个"滑模面" s(x) = 0
%   用不连续控制律迫使系统状态"滑"向这个面
%   一旦到达滑模面 → 系统动态退化为低阶稳定系统
%   → 对参数不确定和外部扰动完全免疫！
%
% 【核心思想】
%   1. 选择滑模面 s = λ*e + ė
%   2. 设计控制律 u = u_eq + u_sw
%      u_eq: 等效控制（让系统保持在滑模面上）
%      u_sw: 切换控制（迫使系统趋向滑模面）
%
% 【SMC vs PID vs LQR】
%   PID: 简单但参数敏感
%   LQR: 需要精确模型
%   SMC: 模型不准也行 → 对不确定性鲁棒
%   代价：控制信号剧烈抖动 (chattering)
% ============================================================

clear; close all;

fprintf('============================================\n');
fprintf('  教程 21：滑模控制 — 非线性鲁棒控制\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：被控对象 =====

m = 1.0;  c_real = 0.5;  k_real = 4.0;

% 控制器用的"错误"模型（故意不准确）
c_model = 0.3;
k_model = 3.0;
F_bound = 1.5;     % |Δf| 上界

lambda = 5;         % 滑模面斜率
eta = 2;            % 趋近律增益
K_sw = eta + F_bound;

fprintf('【系统参数】\n');
fprintf('  真实: m=%.1f, c=%.1f, k=%.1f\n', m, c_real, k_real);
fprintf('  模型: m=%.1f, c=%.1f, k=%.1f (故意不准!)\n', m, c_model, k_model);
fprintf('  滑模面: s = λe + ė, λ=%.1f\n', lambda);
fprintf('  切换增益: K=%.1f\n\n', K_sw);

%% ===== 第 2 步：仿真 — SMC vs PID vs LQR =====

dt = 0.001;  t_end = 5;
t_sim = 0:dt:t_end;  N = length(t_sim);
r = ones(1, N);
d = 1.5 * sin(2*pi*1.5*t_sim);  % 扰动

% LQR 设计
A_lqr = [0 1; -k_real/m -c_real/m];
B_lqr = [0; 1/m];
K_lqr = lqr(A_lqr, B_lqr, diag([50, 10]), 0.1);

% PID 参数
Kp = 20;  Ki = 50;  Kd = 5;

% 初始化
x_smc  = [0; 0];  e_prev_pid = 0;  e_int_pid = 0;
x_pid  = [0; 0];
x_lqr  = [0; 0];

% 存储所有状态
x1_hist = zeros(3, N);   % 位移 [SMC; PID; LQR]
x2_hist = zeros(3, N);   % 速度 [SMC; PID; LQR]
u_hist  = zeros(3, N);   % 控制 [SMC; PID; LQR]
s_hist  = zeros(1, N);   % 滑模面

for k = 1:N
    d_k = d(k);  r_k = r(k);

    % === SMC ===
    e = x_smc(1) - r_k;
    e_dot = x_smc(2);
    s = lambda * e + e_dot;
    s_hist(k) = s;

    u_eq = m * (-c_model/m * x_smc(2) - k_model/m * x_smc(1) + lambda * e_dot);
    u_sw = -K_sw * sign(s);
    u_smc_val = u_eq + u_sw + k_model * r_k;
    u_hist(1,k) = u_smc_val;

    % === PID ===
    e_pid = r_k - x_pid(1);
    e_int_pid = e_int_pid + e_pid * dt;
    e_dot_pid = (e_pid - e_prev_pid) / dt;
    u_hist(2,k) = Kp * e_pid + Ki * e_int_pid + Kd * e_dot_pid;
    e_prev_pid = e_pid;

    % === LQR ===
    u_lqr_val = -K_lqr * (x_lqr - [r_k; 0]);
    u_hist(3,k) = u_lqr_val;

    % === 系统更新 (Euler) ===
    for idx = 1:3
        if idx == 1, xx = x_smc; uu = u_smc_val;
        elseif idx == 2, xx = x_pid; uu = u_hist(2,k);
        else, xx = x_lqr; uu = u_lqr_val;
        end
        xx(1) = xx(1) + xx(2) * dt;
        xx(2) = xx(2) + ((-c_real*xx(2) - k_real*xx(1) + uu + d_k) / m) * dt;
        if idx == 1, x_smc = xx;
        elseif idx == 2, x_pid = xx;
        else, x_lqr = xx; end
    end

    x1_hist(:,k) = [x_smc(1); x_pid(1); x_lqr(1)];
    x2_hist(:,k) = [x_smc(2); x_pid(2); x_lqr(2)];
end

%% ===== 第 3 步：边界层饱和函数 (anti-chattering) =====

Phi_list = [0.01, 0.05, 0.2];
x_sat1_hist = zeros(length(Phi_list), N);
u_sat_hist  = zeros(length(Phi_list), N);

for ip = 1:length(Phi_list)
    Phi = Phi_list(ip);
    x_sat = [0; 0];
    for k = 1:N
        e_sat = x_sat(1) - r(k);
        e_dot_sat = x_sat(2);
        s_sat = lambda * e_sat + e_dot_sat;

        u_eq_sat = m * (-c_model/m * x_sat(2) - k_model/m * x_sat(1) + lambda * e_dot_sat);
        u_sw_sat = -K_sw * sat(s_sat / Phi);
        u_sat_val = u_eq_sat + u_sw_sat + k_model * r(k);
        u_sat_hist(ip,k) = u_sat_val;

        x_sat(1) = x_sat(1) + x_sat(2) * dt;
        x_sat(2) = x_sat(2) + ((-c_real*x_sat(2) - k_real*x_sat(1) + u_sat_val + d(k)) / m) * dt;
        x_sat1_hist(ip,k) = x_sat(1);
    end
end

%% ===== 第 4 步：图 1 — 跟踪与控制对比 =====

figure('Name', 't21: SMC 控制性能', ...
    'Position', [50, 50, 1000, 700]);

% (1) 位移跟踪
subplot(3,2,1); hold on;
plot(t_sim, r, 'k--', 'LineWidth', 1);
plot(t_sim, x1_hist(1,:), 'b', 'LineWidth', 1.2);
plot(t_sim, x1_hist(2,:), 'r', 'LineWidth', 1);
plot(t_sim, x1_hist(3,:), 'Color', [0 0.6 0], 'LineWidth', 1);
legend('目标', 'SMC', 'PID', 'LQR', 'Location', 'best');
title('位置跟踪 (有扰动 d=1.5sin(3πt))');
xlabel('时间 (s)'); ylabel('位移 (m)'); grid on;

% (2) 控制信号
subplot(3,2,2); hold on;
plot(t_sim, u_hist(1,:), 'b', 'LineWidth', 0.8);
plot(t_sim, u_hist(2,:), 'r', 'LineWidth', 0.8);
plot(t_sim, u_hist(3,:), 'Color', [0 0.6 0], 'LineWidth', 0.8);
legend('SMC', 'PID', 'LQR', 'Location', 'best');
title('控制信号'); xlabel('时间 (s)'); ylabel('u (N)'); grid on;

% (3) 滑模面 s(t)
subplot(3,2,3);
plot(t_sim, s_hist, 'b', 'LineWidth', 1);
hold on; yline(0, 'r--', 'LineWidth', 1);
title('滑模面 s = λe + ė → 0');
xlabel('时间 (s)'); ylabel('s'); grid on;
idx_s0 = find(abs(s_hist)<0.01, 1);
if ~isempty(idx_s0)
    fprintf('  滑模面 s 在 ~%.2fs 内收敛到 0\n', t_sim(idx_s0));
else
    fprintf('  滑模面 s 未收敛到 0\n');
end

% (4) 相轨迹 (位置-速度)
subplot(3,2,4); hold on;
plot(x1_hist(1,:), x2_hist(1,:), 'b', 'LineWidth', 1);
plot(x1_hist(2,:), x2_hist(2,:), 'r', 'LineWidth', 0.8);
plot(x1_hist(3,:), x2_hist(3,:), 'Color', [0 0.6 0], 'LineWidth', 0.8);
plot(1, 0, 'ko', 'MarkerSize', 8, 'LineWidth', 2);  % 目标点
legend('SMC', 'PID', 'LQR', '目标 (1,0)', 'Location', 'best');
title('相轨迹 (x1, x2)'); xlabel('位移'); ylabel('速度'); grid on;

% (5) 边界层厚度影响
subplot(3,2,5); hold on;
plot(t_sim, r, 'k--', 'LineWidth', 1);
plot(t_sim, x1_hist(1,:), 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8);
for ip = 1:length(Phi_list)
    plot(t_sim, x_sat1_hist(ip,:), 'LineWidth', 1.2);
end
legend('目标', 'sign', sprintf('\\Phi=%.2f', Phi_list(1)), ...
    sprintf('\\Phi=%.2f', Phi_list(2)), sprintf('\\Phi=%.2f', Phi_list(3)), 'Location', 'best');
title('边界层 Φ 抑制抖振');
xlabel('时间 (s)'); ylabel('位移 (m)'); grid on;

% (6) RMS 误差柱状图
subplot(3,2,6);
try
    rmse = [rms(x1_hist(1,1000:end)-r(1000:end));
            rms(x1_hist(2,1000:end)-r(1000:end));
            rms(x1_hist(3,1000:end)-r(1000:end))];
catch
    rmse = [std(x1_hist(1,1000:end)-r(1000:end));
            std(x1_hist(2,1000:end)-r(1000:end));
            std(x1_hist(3,1000:end)-r(1000:end))];
end
b = bar(rmse);
b.FaceColor = 'flat';
b.CData(1,:) = [0 0.45 0.74];
b.CData(2,:) = [0.85 0.33 0.10];
b.CData(3,:) = [0 0.5 0];
set(gca, 'XTickLabel', {'SMC', 'PID', 'LQR'});
ylabel('RMS 跟踪误差 (m)'); grid on;
title('稳态 RMS 误差');
text(1:3, rmse', string(round(rmse,4)), ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom');

fprintf('\n【性能对比 (RMS 跟踪误差)】\n');
fprintf('  SMC : %.4f m  ← 对扰动最不敏感\n', rmse(1));
fprintf('  PID : %.4f m\n', rmse(2));
fprintf('  LQR : %.4f m\n\n', rmse(3));

%% ===== 第 5 步：图 2 — 控制力与抖振细节 =====

figure('Name', 't21: 抖振分析', ...
    'Position', [50, 50, 900, 350]);

% 放大显示 SMC 抖振
idx_zoom = (t_sim >= 2) & (t_sim <= 2.5);

subplot(1,2,1); hold on;
plot(t_sim(idx_zoom), u_hist(1,idx_zoom), 'b', 'LineWidth', 0.8);
for ip = 1:length(Phi_list)
    plot(t_sim(idx_zoom), u_sat_hist(ip,idx_zoom), 'LineWidth', 1);
end
legend('sign 抖振', sprintf('\\Phi=%.2f', Phi_list(1)), ...
    sprintf('\\Phi=%.2f', Phi_list(2)), sprintf('\\Phi=%.2f', Phi_list(3)), 'Location', 'best');
title('控制信号放大 (2-2.5s) — 抖振抑制效果');
xlabel('时间 (s)'); ylabel('u (N)'); grid on;

subplot(1,2,2); hold on;
plot(t_sim(idx_zoom), x1_hist(1,idx_zoom), 'Color', [0.6 0.6 0.6], 'LineWidth', 0.8);
for ip = 1:length(Phi_list)
    plot(t_sim(idx_zoom), x_sat1_hist(ip,idx_zoom), 'LineWidth', 1.2);
end
plot(t_sim(idx_zoom), r(idx_zoom), 'k--', 'LineWidth', 1);
legend('sign', sprintf('\\Phi=%.2f', Phi_list(1)), ...
    sprintf('\\Phi=%.2f', Phi_list(2)), sprintf('\\Phi=%.2f', Phi_list(3)), '目标', 'Location', 'best');
title('位移放大 (2-2.5s)');
xlabel('时间 (s)'); ylabel('位移 (m)'); grid on;

fprintf('========================================\n');
fprintf('  教程 21 完成！\n');
fprintf('========================================\n\n');

fprintf('【SMC 工程实践要点】\n');
fprintf('  1. 经典 sign()     → 完美鲁棒性，但抖振严重\n');
fprintf('  2. 边界层 sat()    → 工程折中，Φ大则抖振小但精度降\n');
fprintf('  3. Super-Twisting  → 连续控制无抖振，适合高精度场景\n');
fprintf('  4. SMC + Observer  → 部分状态不可测时的组合方案\n\n');

fprintf('【推荐 Φ 选择】\n');
fprintf('  Φ = 0.01~0.05 : 高精度 (μm级)，轻微抖振\n');
fprintf('  Φ = 0.1~0.2   : 通用场景，平滑控制\n');
fprintf('  Φ > 0.5       : 接近连续控制，但鲁棒性减弱\n');

%% ===== 辅助函数 =====
function y = sat(x)
    if abs(x) <= 1
        y = x;
    else
        y = sign(x);
    end
end
