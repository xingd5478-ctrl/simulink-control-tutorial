%% ============================================================
% 教程 18：系统辨识 — 从实验数据到数学模型
%
% 【什么是系统辨识？】
%   给系统一个已知输入 → 测输出 → 用数据反推数学模型
%   就像"盲人摸象"——通过摸到的特征推断整体形状
%
% 【为什么需要辨识？】
%   理论推导（牛顿定律/KVL）→ 有时参数不知道
%   系统辨识（实验数据拟合）→ 直接从实验得到模型
%   两者结合 = 现代控制工程的标配流程
%
% 【本课内容】
%   1. 阶跃响应辨识（一阶+延迟系统）
%   2. 最小二乘法参数估计
%   3. 模型验证与残差分析
%   4. 使用 System Identification Toolbox
% ============================================================

clear; close all;

fprintf('============================================\n');
fprintf('  教程 18：系统辨识 — 从数据到模型\n');
fprintf('============================================\n\n');

%% ===== Simulink 模型：未知系统阶跃响应实验 =====

mdl = 'tutorial18_sysid';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl, 'Model');
open_system(mdl);

add_block('simulink/Sources/Step', [mdl '/Step Excite'], ...
    'Position', [50, 80, 100, 120]);
set_param([mdl '/Step Excite'], 'Time', '0.5', 'After', '2');

add_block('simulink/Continuous/Transfer Fcn', [mdl '/Unknown Plant'], ...
    'Position', [200, 80, 290, 120]);
set_param([mdl '/Unknown Plant'], 'Numerator', '[2]', 'Denominator', '[3 1]');

add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [400, 80, 450, 120]);

add_line(mdl, 'Step Excite/1', 'Unknown Plant/1');
add_line(mdl, 'Unknown Plant/1', 'Scope/1');

ph = get_param([mdl '/Unknown Plant'], 'PortHandles');
set_param(ph.Outport(1), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'y_plant');

fprintf('  [Simulink] tutorial18_sysid.slx 已创建\n\n');

%% ===== 第 1 步：生成"未知"系统的实验数据 =====

% 假设我们不知道系统参数，只能做实验测数据
% 真实系统：G(s) = 2/(3s+1) * e^{-1.5s}（带延迟的一阶系统）
% 我们只知道输入 u(t) 和输出 y(t)

K_true = 2.0;
tau_true = 3.0;
td_true = 1.5;     % 延迟 (dead time)

% 先构造无延迟的一阶系统
sys_true = tf(K_true, [tau_true, 1]);

fprintf('【真实系统（我们假装不知道）】\n');
fprintf('  G(s) = %.1f/(%.1fs+1) * e^{-%.1fs}\n', K_true, tau_true, td_true);
fprintf('  增益 K=%.1f, 时间常数 τ=%.1f, 延迟 θ=%.1f\n\n', K_true, tau_true, td_true);

% 生成阶跃响应数据（带噪声模拟真实传感器）
dt = 0.05;
t_data = (0:dt:30)';
u = ones(size(t_data));
u(t_data < 1) = 0;  % 1秒时阶跃

% 仿真真实系统输出
[y_true, ~] = lsim(sys_true, u, t_data);

% 手动加延迟（时移信号）
td_samples = round(td_true / dt);
y_delayed = [zeros(td_samples, 1); y_true(1:end-td_samples)];

% 加测量噪声
rng(42);  % 固定随机种子，结果可复现
noise_std = 0.05;
y_meas = y_delayed + noise_std * randn(size(y_delayed));

fprintf('【实验数据】\n');
fprintf('  输入：1V 阶跃信号 @ t=1s\n');
fprintf('  输出：带 %.2f 标准差噪声的位移测量值\n', noise_std);
fprintf('  共 %d 个采样点，dt=%.2fs\n\n', length(t_data), dt);

%% ===== 第 2 步：阶跃响应法辨识 =====

% 方法：从阶跃响应曲线直接读特征值
% - 稳态值 → K
% - 63.2% 稳态值对应的时间 → τ + θ
% - 拐点切线 → θ

y_ss = mean(y_meas(end-50:end));  % 取最后一段平均作为稳态值
K_est = y_ss / 1;  % 输入幅值为1

% 找稳态值的 63.2% 对应时间
y_632 = 0.632 * y_ss;
idx_632 = find(y_meas >= y_632, 1, 'first');
t_632 = t_data(idx_632);

% 找稳态值的 28.3% 对应时间  (用于两点法)
y_283 = 0.283 * y_ss;
idx_283 = find(y_meas >= y_283, 1, 'first');
t_283 = t_data(idx_283);

tau_est = 1.5 * (t_632 - t_283);
theta_est = t_632 - tau_est;

fprintf('【阶跃响应法辨识结果】\n');
fprintf('  估计增益  K_est  = %.2f (真实值 %.1f)\n', K_est, K_true);
fprintf('  估计时间常数 τ_est = %.2f (真实值 %.1f)\n', tau_est, tau_true);
fprintf('  估计延迟 θ_est = %.2f (真实值 %.1f)\n', theta_est, td_true);
fprintf('  → 简单但有噪声敏感性\n\n');

%% ===== 第 3 步：最小二乘法辨识 =====

% 用 ARX 模型：y[k] = a1*y[k-1] + b0*u[k-d] + b1*u[k-d-1]
% 写成矩阵形式：Y = Φ*θ → θ_ls = (Φ'Φ)^(-1)Φ'Y

d = round(theta_est/dt);  % 延迟步数
N = length(y_meas);

% 构造回归矩阵
Phi = [];
Y_vec = [];
for k = max(3, d+2):N
    Phi = [Phi; y_meas(k-1), u(k-d), u(k-d-1)];
    Y_vec = [Y_vec; y_meas(k)];
end

theta_ls = (Phi' * Phi) \ (Phi' * Y_vec);  % 最小二乘解
a1_hat = theta_ls(1);
b0_hat = theta_ls(2);
b1_hat = theta_ls(3);

% 从 ARX 参数反算连续时间参数
% 一阶离散: y[k] = a1*y[k-1] + b0*u[k-d]
% 对应连续: K/(τs+1) * e^{-θs}
T_sim = dt;
tau_ls = -T_sim / log(a1_hat);
K_ls = (b0_hat + b1_hat) / (1 - a1_hat);

fprintf('【最小二乘法辨识结果】\n');
fprintf('  a1 = %.4f → τ_ls = %.2f s\n', a1_hat, tau_ls);
fprintf('  K_ls = %.2f\n', K_ls);
fprintf('  → 比阶跃法更精确，尤其在有噪声时\n\n');

%% ===== 第 4 步：模型验证 — 对比预测输出 =====

% 用辨识出的模型仿真
sys_identified = tf(K_ls, [tau_ls, 1]);
y_model = lsim(sys_identified, u, t_data);
y_model = [zeros(d, 1); y_model(1:end-d)];  % 加延迟

figure('Name', 't18: 系统辨识结果', ...
    'Position', [50, 50, 1000, 700]);

% 图1：实测 vs 模型预测
subplot(2,2,1); hold on;
plot(t_data, y_meas, 'b.', 'MarkerSize', 3);
plot(t_data, y_true, 'g-', 'LineWidth', 1.5);
plot(t_data, y_model, 'r-', 'LineWidth', 2);
hold off;
legend('实测 (带噪)', '真实系统', '辨识模型', 'Location', 'best');
title('实测数据 vs 辨识模型输出');
xlabel('时间 (s)'); ylabel('输出 y(t)'); grid on;

% 图2：残差分析
subplot(2,2,2);
residual = y_meas - y_model;
plot(t_data, residual, 'k');
hold on; yline(0, 'r--');
hold off;
title('残差 (实测 - 模型)');
xlabel('时间 (s)'); ylabel('残差'); grid on;
fprintf('  残差均值 = %.4f, 残差方差 = %.4f\n', mean(residual), var(residual));

% 图3：不同噪声水平下的辨识精度对比
subplot(2,2,3);
noise_levels = [0.01, 0.05, 0.1, 0.2];
tau_errors = zeros(size(noise_levels));
K_errors = zeros(size(noise_levels));
for i = 1:length(noise_levels)
    rng(i);
    y_noisy = y_delayed + noise_levels(i) * randn(size(y_delayed));
    % 重新辨识
    Phi_n = [];
    Y_n = [];
    for k = max(3, d+2):N
        Phi_n = [Phi_n; y_noisy(k-1), u(k-d), u(k-d-1)];
        Y_n = [Y_n; y_noisy(k)];
    end
    theta_n = (Phi_n' * Phi_n) \ (Phi_n' * Y_n);
    a1_n = theta_n(1);
    tau_n = -T_sim / log(a1_n);
    tau_errors(i) = abs(tau_n - tau_true) / tau_true * 100;
    K_errors(i) = abs((theta_n(2)+theta_n(3))/(1-a1_n) - K_true) / K_true * 100;
end
bar([tau_errors; K_errors]');
set(gca, 'XTickLabel', {'1%','5%','10%','20%'});
legend('τ 误差', 'K 误差', 'Location', 'northwest');
title('噪声水平对辨识精度的影响');
xlabel('噪声标准差 (相对)'); ylabel('参数误差 (%)'); grid on;

% 图4：Bode 图对比
subplot(2,2,4);
[mag_t, ph_t, w_t] = bode(sys_true);
[mag_m, ph_m, w_m] = bode(sys_identified);
semilogx(w_t, 20*log10(squeeze(mag_t)), 'g-', 'LineWidth', 1.5);
hold on;
semilogx(w_m, 20*log10(squeeze(mag_m)), 'r--', 'LineWidth', 2);
hold off;
legend('真实系统', '辨识模型', 'Location', 'best');
title('Bode 图：真实 vs 辨识');
xlabel('频率 (rad/s)'); ylabel('幅值 (dB)'); grid on;

%% ===== 第 5 步：System Identification Toolbox 演示 =====

fprintf('\n【System Identification Toolbox 快速演示】\n');

% 创建 iddata 对象
data = iddata(y_meas, u, dt);
data.TimeUnit = 'seconds';

% 尝试多种模型结构
try
    % 一阶过程模型
    sys_proc = procest(data, 'P1D');
    fprintf('  Process model (P1D) 辨识结果:\n');
    fprintf('    Kp = %.3f, Tp1 = %.3f, Td = %.3f\n', ...
        sys_proc.Kp, sys_proc.Tp1, sys_proc.Td);

    % 传递函数模型
    sys_tf = tfest(data, 1, 0);  % 一阶无零点
    [num_tf, den_tf] = tfdata(tf(sys_tf));
    fprintf('  TF model (1阶): K=%.3f, τ=%.3f\n', ...
        dcgain(tf(sys_tf)), -1/den_tf{1}(2));
catch ME
    fprintf('  Toolbox demo skipped: %s\n', ME.message);
end

fprintf('\n========================================\n');
fprintf('  教程 18 完成！\n');
fprintf('========================================\n\n');

fprintf('【辨识工程实践 checklist】\n');
fprintf('  1. 输入信号要有足够激励（阶跃、PRBS、扫频）\n');
fprintf('  2. 采集足够长的数据（至少 5×τ）\n');
fprintf('  3. 数据预处理：去趋势、去野值、滤波\n');
fprintf('  4. 试多种模型结构，用 AIC/BIC 选最优\n');
fprintf('  5. 用独立数据集做交叉验证（不要用训练数据验证）\n');
fprintf('  6. 辨识 → 控制器设计 → 再辨识 → 迭代优化\n');

save_system(mdl, fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']));
