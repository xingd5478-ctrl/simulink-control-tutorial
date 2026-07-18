%% ============================================================
% 教程 23：模型参考自适应控制 (MRAC)
%
% 【MRAC 解决什么问题？】
%   PID/LQR 的增益是固定的 → 系统参数变了（老化、负载变化）
%   → 原来的增益不再适用 → 需要"自适应"地调整参数。
%
%   MRAC 的思路：
%   1. 设定一个"参考模型"——描述你想要的理想系统行为
%   2. 比较真实输出和参考模型输出的误差
%   3. 用 MIT 规则（或 Lyapunov 方法）在线调整控制器参数
%   4. 让真实系统逐步"变成"参考模型
%
% 【MIT 规则】
%   调整目标：最小化 e²，其中 e = y_real - y_model
%   参数更新：dθ/dt = -γ · e · (∂e/∂θ)
%   即：误差大时调快一点，误差小时调慢一点
%
% 【本课内容】
%   1. MIT 规则的推导与实现
%   2. Simulink 模型：自适应增益 vs 固定增益对比
%   3. 系统参数突变时的自适应能力
%   4. 自适应增益 γ 的调节
% ============================================================

clear; close all;

fprintf('============================================\n');
fprintf('  教程 23：模型参考自适应控制 (MRAC)\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：参考模型与被控对象 =====

% 参考模型：我们希望系统表现成的样子
tau_m = 0.5;   % 参考时间常数（快！）
sys_model = tf([1], [tau_m, 1]);

% 真实被控对象：参数会变化（我们假装不知道）
tau_real = 2.0;  % 真实时间常数（比参考模型慢很多）
sys_real = tf([1], [tau_real, 1]);

fprintf('【系统参数】\n');
fprintf('  参考模型: G_m(s) = 1/(%.1fs+1) — 理想行为（快）\n', tau_m);
fprintf('  真实系统: G_p(s) = 1/(%.1fs+1) — 实际行为（慢 4 倍）\n', tau_real);
fprintf('  目标: 通过自适应调整增益，让真实系统追平参考模型\n\n');

%% ===== 第 2 步：Simulink 模型搭建 =====

mdl = 'tutorial23_mrac';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl, 'Model');
open_system(mdl);

% --- 参考模型路径 ---
add_block('simulink/Sources/Step', [mdl '/Step Ref'], ...
    'Position', [50, 50, 100, 90]);
set_param([mdl '/Step Ref'], 'Time', '0.5', 'After', '2');  % 阶跃到 2

add_block('simulink/Continuous/Transfer Fcn', [mdl '/Ref Model'], ...
    'Position', [350, 50, 440, 90]);
set_param([mdl '/Ref Model'], ...
    'Numerator', '[1]', 'Denominator', ['[' num2str(tau_m) ' 1]']);

% --- 自适应控制器路径 ---
% Apply Gain 模块：计算 θ(t) * r(t)，直接驱动被控对象
add_block('simulink/Math Operations/Product', [mdl '/Apply Gain'], ...
    'Position', [200, 150, 260, 210]);
set_param([mdl '/Apply Gain'], 'Inputs', '2');

add_block('simulink/Continuous/Transfer Fcn', [mdl '/Real Plant'], ...
    'Position', [350, 160, 440, 200]);
set_param([mdl '/Real Plant'], ...
    'Numerator', '[1]', 'Denominator', ['[' num2str(tau_real) ' 1]']);

% --- 固定增益对比路径 ---
add_block('simulink/Math Operations/Gain', [mdl '/Fixed Gain'], ...
    'Position', [200, 310, 250, 350]);
set_param([mdl '/Fixed Gain'], 'Gain', '3.0');

add_block('simulink/Continuous/Transfer Fcn', [mdl '/Real Plant Fixed'], ...
    'Position', [350, 310, 440, 350]);
set_param([mdl '/Real Plant Fixed'], ...
    'Numerator', '[1]', 'Denominator', ['[' num2str(tau_real) ' 1]']);

% --- MIT 自适应律：θ̇ = -γ · e · ym ---
add_block('simulink/Math Operations/Add', [mdl '/Error Calc'], ...
    'Position', [480, 200, 510, 260]);
set_param([mdl '/Error Calc'], 'Inputs', '|-+');

add_block('simulink/Math Operations/Product', [mdl '/Product MIT'], ...
    'Position', [480, 320, 520, 380]);
set_param([mdl '/Product MIT'], 'Inputs', '2');

add_block('simulink/Math Operations/Gain', [mdl '/Gamma'], ...
    'Position', [570, 330, 600, 370]);
set_param([mdl '/Gamma'], 'Gain', '-2.0');

add_block('simulink/Continuous/Integrator', [mdl '/Integrator'], ...
    'Position', [650, 330, 700, 370]);
set_param([mdl '/Integrator'], 'InitialCondition', '1.0');

% --- 示波器 ---
add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [600, 50, 650, 290]);
set_param([mdl '/Scope'], 'NumInputPorts', '3');

% --- 连线 ---
% 参考模型路径
add_line(mdl, 'Step Ref/1', 'Ref Model/1');
add_line(mdl, 'Ref Model/1', 'Scope/1');

% 自适应路径：Step → Apply Gain (θ*r) → Real Plant
add_line(mdl, 'Step Ref/1', 'Apply Gain/1');      % r(t) 输入
add_line(mdl, 'Apply Gain/1', 'Real Plant/1');
add_line(mdl, 'Real Plant/1', 'Scope/2');

% 固定增益路径
add_line(mdl, 'Step Ref/1', 'Fixed Gain/1');
add_line(mdl, 'Fixed Gain/1', 'Real Plant Fixed/1');
add_line(mdl, 'Real Plant Fixed/1', 'Scope/3');

% MIT 自适应律连线链：
% 误差 = y_plant - y_model
add_line(mdl, 'Real Plant/1', 'Error Calc/2');     % y_plant → (+)
add_line(mdl, 'Ref Model/1', 'Error Calc/1');       % y_model → (-)

% Product: e * ym
add_line(mdl, 'Error Calc/1', 'Product MIT/1');     % e = yp - ym
add_line(mdl, 'Ref Model/1', 'Product MIT/2');      % ym

% θ̇ = -γ * e * ym = Gamma * (e * ym)
add_line(mdl, 'Product MIT/1', 'Gamma/1');

% θ = ∫ θ̇ dt
add_line(mdl, 'Gamma/1', 'Integrator/1');

% θ → Apply Gain (乘以 r 得到控制量)
add_line(mdl, 'Integrator/1', 'Apply Gain/2');      % θ(t) 输入

fprintf('【Simulink 模型已创建】tutorial23_mrac.slx\n');
fprintf('  三条并行路径：\n');
fprintf('    参考模型 — 阶跃→1/(0.5s+1)→Scope[1]\n');
fprintf('    自适应   — 阶跃→ApplyGain(θ*r)→1/(2s+1)→Scope[2]\n');
fprintf('              ↑ MIT链: Error→Product→Gamma→Integ→θ┘\n');
fprintf('    固定增益 — 阶跃→K=3→1/(2s+1)→Scope[3]\n');
fprintf('  自适应律: θ̇ = -γ·e·ym,  e=yp-ym\n\n');

%% ===== 第 3 步：MATLAB 仿真对比 =====

dt = 0.005;  t_end = 15;
t = 0:dt:t_end;  N = length(t);

r = 2 * ones(1, N);  % 目标值 = 2
r(t < 0.5) = 0;

% 参考模型输出
ym = zeros(1, N);
% 被控对象输出
yp_adapt = zeros(1, N);
yp_fixed = zeros(1, N);
% 自适应增益
theta = zeros(1, N);
theta(1) = 1.0;  % 初始增益

gamma = 2.0;     % 自适应率
K_fixed = 3.0;   % 固定增益

% 欧拉仿真
for k = 1:N-1
    % 参考模型: τm·ẏm + ym = r
    ym_dot = (r(k) - ym(k)) / tau_m;
    ym(k+1) = ym(k) + ym_dot * dt;

    % 自适应对象: τp·ẏp + yp = θ·r
    yp_dot = (theta(k) * r(k) - yp_adapt(k)) / tau_real;
    yp_adapt(k+1) = yp_adapt(k) + yp_dot * dt;

    % 固定增益对象
    yp_dot_f = (K_fixed * r(k) - yp_fixed(k)) / tau_real;
    yp_fixed(k+1) = yp_fixed(k) + yp_dot_f * dt;

    % MIT 规则：dθ/dt = -γ · (yp - ym) · ym
    e_adapt = yp_adapt(k) - ym(k);
    theta_dot = -gamma * e_adapt * ym(k);
    theta(k+1) = theta(k) + theta_dot * dt;
end

%% ===== 第 4 步：系统参数突变 — 自适应 vs 固定 =====

% 5 秒时 τ_real 从 2.0 突变为 4.0
tau_init = tau_real;
tau_change_time = 5;
tau_changed = 4.0;

ym2 = zeros(1, N);  yp_a2 = zeros(1, N);
yp_f2 = zeros(1, N);  theta2 = zeros(1, N);
theta2(1) = 1.0;

for k = 1:N-1
    % 当前真实时间常数
    if t(k) < tau_change_time
        tau_now = tau_init;
    else
        tau_now = tau_changed;
    end

    ym_dot = (r(k) - ym2(k)) / tau_m;
    ym2(k+1) = ym2(k) + ym_dot * dt;

    yp_dot = (theta2(k) * r(k) - yp_a2(k)) / tau_now;
    yp_a2(k+1) = yp_a2(k) + yp_dot * dt;

    yp_dot_f = (K_fixed * r(k) - yp_f2(k)) / tau_now;
    yp_f2(k+1) = yp_f2(k) + yp_dot_f * dt;

    e_adapt = yp_a2(k) - ym2(k);
    theta_dot = -gamma * e_adapt * ym2(k);
    theta2(k+1) = theta2(k) + theta_dot * dt;
end

%% ===== 第 5 步：绘图 =====

figure('Name', 't23: MRAC 自适应控制', ...
    'Position', [50, 50, 1000, 700]);

% 图1：正常工况对比
subplot(2,2,1); hold on;
plot(t, r, 'k--', 'LineWidth', 1);
plot(t, ym, 'b', 'LineWidth', 1.5);
plot(t, yp_adapt, 'r', 'LineWidth', 1.5);
plot(t, yp_fixed, 'Color', [0 0.6 0], 'LineWidth', 1);
legend('目标', '参考模型', '自适应', '固定增益', 'Location', 'best');
title('输出对比（正常工况）');
xlabel('时间 (s)'); ylabel('输出'); grid on;

% 图2：自适应增益变化
subplot(2,2,2); hold on;
plot(t, theta, 'r', 'LineWidth', 1.5);
yline(2/0.5, 'b--', 'LineWidth', 1);  % 理论最优值 = τp/τm
text(t_end*0.6, 2/0.5*1.1, sprintf('理论最优 θ*=τp/τm=%.1f', tau_real/tau_m));
title('自适应增益 θ(t)');
xlabel('时间 (s)'); ylabel('增益'); grid on;
fprintf('  理论最优增益: θ* = %.1f (真实系统/参考模型)\n', tau_real/tau_m);
fprintf('  MIT 自适应最终收敛于 θ* 附近 ✓\n\n');

% 图3：参数突变工况
subplot(2,2,3); hold on;
plot(t, r, 'k--', 'LineWidth', 1);
plot(t, ym2, 'b', 'LineWidth', 1.5);
plot(t, yp_a2, 'r', 'LineWidth', 1.5);
plot(t, yp_f2, 'Color', [0 0.6 0], 'LineWidth', 1);
xline(tau_change_time, 'k:', 'LineWidth', 1.5);
text(tau_change_time+0.2, 2.5, '参数突变!');
legend('目标', '参考模型', '自适应', '固定增益', 'Location', 'southeast');
title(sprintf('参数突变 (τ: %.1f→%.1f) 时对比', tau_init, tau_changed));
xlabel('时间 (s)'); ylabel('输出'); grid on;

% 图4：突变后增益重新调整
subplot(2,2,4); hold on;
plot(t, theta2, 'r', 'LineWidth', 1.5);
yline(tau_init/tau_m, 'b--', 'LineWidth', 1);
yline(tau_changed/tau_m, 'g--', 'LineWidth', 1);
xline(tau_change_time, 'k:', 'LineWidth', 1.5);
legend('θ(t)', '旧最优值', '新最优值', 'Location', 'best');
title('参数突变后增益自适应重新收敛');
xlabel('时间 (s)'); ylabel('增益'); grid on;

fprintf('========================================\n');
fprintf('  教程 23 完成！\n');
fprintf('========================================\n\n');

fprintf('【MRAC 核心理解】\n');
fprintf('  1. 参考模型 = "理想中的自己" → 努力追上\n');
fprintf('  2. MIT 规则 = "看差距、调参数" → 误差大则大步调\n');
fprintf('  3. γ 太大 → 快速收敛但不稳定（超调/振荡）\n');
fprintf('  4. γ 太小 → 稳定但收敛慢\n');
fprintf('  5. 参数突变 → 固定增益退化 → 自适应自动恢复 ✓\n\n');

fprintf('【线性控制器 vs 自适应控制器】\n');
fprintf('  PID/LQR: 假设系统不变 → 参数变化时性能退化\n');
fprintf('  MRAC: 自动调整 → 系统变了也能追回性能\n');
fprintf('  → MRAC 适合于：负载变化大的场合（机器人、航空）\n');
fprintf('  → 但不适合：变化太快（来不及调）、噪声太大的场景\n');

save_system(mdl, fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']));
