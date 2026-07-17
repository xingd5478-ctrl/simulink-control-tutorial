%% ============================================================
% 教程 19：鲁棒控制 — H∞ 与 μ 综合入门
%
% 【为什么需要鲁棒控制？】
%   LQR 假设模型精确 → 但实际总有不确定性
%   H∞ 控制：设计时主动考虑"最坏情况"
%   目标是保证系统在所有可能的不确定性下都稳定
%
% 【本课核心概念】
%   1. 不确定性建模（参数不确定）
%   2. 混合灵敏度 S/KS/T 整形
%   3. H∞ 控制器设计 (hinfsyn)
%   4. μ 分析 — 鲁棒稳定裕度
%   5. H∞ vs LQR 对比
%
% 【前置知识】t09 状态空间, t16 频域分析
% ============================================================

clear; close all;

fprintf('============================================\n');
fprintf('  教程 19：鲁棒控制 H∞ / μ 综合\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：名义系统 + 不确定性建模 =====

m_nom = 1.0;  c_nom = 0.5;  k_nom = 10.0;

G_nom = ss([0 1; -k_nom/m_nom -c_nom/m_nom], ...
           [0; 1/m_nom], [1 0], 0);

% 参数不确定 → 用 uss (uncertain state-space)
m_unc = ureal('m', 1.0, 'Percentage', 20);
c_unc = ureal('c', 0.5, 'Percentage', 20);
k_unc = ureal('k', 10.0, 'Percentage', 20);

A_unc = [0 1; -k_unc/m_unc -c_unc/m_unc];
B_unc = [0; 1/m_unc];
G_unc = uss(A_unc, B_unc, [1 0], 0);

fprintf('【名义系统 + 不确定性】\n');
fprintf('  质量 m=1.0±20%% kg, 阻尼 c=0.5±20%%, 刚度 k=10.0±20%%\n\n');

%% ===== 第 2 步：不确定系统开环响应 =====

figure('Name', 't19: 不确定系统', ...
    'Position', [50, 50, 900, 400]);

subplot(1,2,1); hold on;
for i = 1:20
    Gi = usample(G_unc, 1);
    step(feedback(Gi, 1), 0:0.01:5);
end
hold off; grid on;
title('20 组随机采样的闭环阶跃响应');
xlabel('时间 (s)'); ylabel('位移 (m)');
fprintf('  → 20 组随机采样的阶跃响应差异显著\n');

subplot(1,2,2); hold on;
for i = 1:20
    Gi = usample(G_unc, 1);
    [mag, ~, w] = bode(Gi, {0.1, 100});
    loglog(w, squeeze(mag), 'Color', [0.6 0.6 0.6]);
end
hold off; grid on;
set(gca, 'XScale', 'log', 'YScale', 'log');
title('20 组随机采样的幅频特性');
xlabel('频率 (rad/s)'); ylabel('幅值');
fprintf('  → 频域特性同样分散，需要鲁棒控制器\n\n');

%% ===== 第 3 步：加权函数设计（混合灵敏度）=====

s = tf('s');
W1 = (s/1.5 + 1) / (s/0.015 + 1);    % 低频扰动抑制
W2 = 0.01 * (s+1) / (0.01*s+1);       % 控制能量限制
W3 = (s/5 + 1) / (s/50 + 1);          % 高频噪声衰减

fprintf('【加权函数 — 频域整形目标】\n');
fprintf('  W1 大 → 低频 S 小 → 扰动抑制好\n');
fprintf('  W2 大 → 控制信号不太激进\n');
fprintf('  W3 大 → 高频 T 小 → 噪声抑制好\n\n');

figure('Name', 't19: 加权函数', ...
    'Position', [50, 50, 600, 400]);
sigma(1/W1, 'b', 1/W2, 'g', 1/W3, 'r', {0.001, 1000});
legend('1/W1 (S约束)', '1/W2 (KS约束)', '1/W3 (T约束)', 'Location', 'best');
title('加权函数倒数：频域整形目标');
grid on;

%% ===== 第 4 步：H∞ 综合 =====

% 手工构建广义对象 P: [r; u] -> [z1=W1*e; z2=W2*u; z3=W3*y; e=r-y]
P11 = [W1; 0; 0; 1];
P12 = [-W1*G_nom; W2; W3*G_nom; -G_nom];
P = ss([P11, P12]);

fprintf('【H∞ 综合】\n');

K_valid = false;
try
    [K_hinf, CL, gamma] = hinfsyn(P, 1, 1);
    fprintf('  H∞ 范数 γ = %.3f\n', gamma);
    if gamma < 1
        fprintf('  γ < 1 → 设计满足所有频域约束\n\n');
    else
        fprintf('  γ ≥ 1 → 需放松权重函数约束\n\n');
    end
    K_valid = true;
catch ME
    fprintf('  hinfsyn 失败: %s\n', ME.message);
    fprintf('  → 尝试放宽 W1 的穿越频率\n\n');
end

%% ===== 第 5 步：闭环分析与对比 =====

figure('Name', 't19: H∞ vs LQR 对比', ...
    'Position', [50, 50, 900, 600]);

% 左上：H∞ 不确定系统闭环响应
subplot(2,2,1); hold on;
if K_valid
    for i = 1:20
        Gi = usample(G_unc, 1);
        step(feedback(Gi*K_hinf, 1), 0:0.01:5);
    end
    title('H∞ 控制：不确定系统闭环响应');
else
    text(0.5, 0.5, 'H∞ 求解未成功', 'HorizontalAlignment', 'center');
    title('H∞ 控制 (未收敛)');
end
hold off; grid on;
xlabel('时间 (s)'); ylabel('位移 (m)');

% 右上：LQR 不确定系统闭环响应
subplot(2,2,2); hold on;
Q = diag([100, 1]);
R = 1;
K_lqr = lqr(G_nom.A, G_nom.B, Q, R);

for i = 1:20
    Gi = usample(G_unc, 1);
    A_cl = Gi.A - Gi.B * K_lqr;
    step(ss(A_cl, Gi.B, Gi.C, 0), 0:0.01:5);
end
hold off; grid on;
title('LQR 控制：不确定系统闭环响应');
xlabel('时间 (s)'); ylabel('位移 (m)');

% 左下：名义系统阶跃对比
subplot(2,2,3); hold on;
step(feedback(G_nom*K_lqr, 1), 0:0.01:5);
if K_valid
    step(feedback(G_nom*K_hinf, 1), 0:0.01:5);
    legend('LQR', 'H∞', 'Location', 'best');
else
    legend('LQR', 'Location', 'best');
end
hold off; grid on;
title('名义模型闭环阶跃响应');
xlabel('时间 (s)'); ylabel('位移 (m)');

% 右下：灵敏度函数
subplot(2,2,4);
if K_valid
    L_hinf = G_nom * K_hinf;
    S_hinf = feedback(1, L_hinf);
    sigma(S_hinf, 'b', 1/W1, 'r--', {0.001, 1000});
    legend('S (H∞)', '1/W1 (约束)', 'Location', 'best');
    title('灵敏度函数 S = |1/(1+GK)|');
else
    text(0.5, 0.5, '无 H∞ 控制器', 'HorizontalAlignment', 'center');
    title('灵敏度函数');
end
grid on;

%% ===== 第 6 步：鲁棒稳定性验证 =====

figure('Name', 't19: 鲁棒稳定性分析', ...
    'Position', [50, 50, 700, 500]);

% 上图：Bode 幅频对比
subplot(2,1,1);
Gi_min = usubs(G_unc, 'm', 0.8, 'c', 0.4, 'k', 8.0);
Gi_max = usubs(G_unc, 'm', 1.2, 'c', 0.6, 'k', 12.0);
[mag_nom, ~, w_nom] = bode(G_nom, {0.1, 100});
[mag_min, ~, w_min] = bode(Gi_min, {0.1, 100});
[mag_max, ~, w_max] = bode(Gi_max, {0.1, 100});
loglog(w_nom, squeeze(mag_nom), 'b', 'LineWidth', 1.5); hold on;
loglog(w_min, squeeze(mag_min), 'r--', 'LineWidth', 1.2);
loglog(w_max, squeeze(mag_max), 'g--', 'LineWidth', 1.2);
hold off;
legend('名义', 'min 参数', 'max 参数', 'Location', 'southwest');
title('参数波动下的幅频特性对比');
xlabel('频率 (rad/s)'); ylabel('幅值'); grid on;

% 下图：鲁棒稳定裕度
subplot(2,1,2);
try
    [stabmarg, ~] = robstab(G_unc);
    bar(1, stabmarg.LowerBound, 'FaceColor', [0.2 0.6 0.8]);
    hold on; yline(1, 'r--', 'LineWidth', 1.5);
    text(1, stabmarg.LowerBound * 0.5, sprintf('%.2f×', stabmarg.LowerBound), ...
        'HorizontalAlignment', 'center', 'FontSize', 14, 'FontWeight', 'bold');
    hold off;
    set(gca, 'XTickLabel', {'鲁棒稳定裕度'});
    ylabel('裕度倍数'); grid on;
    title(sprintf('参数可波动 %.0f%% 仍保持稳定', stabmarg.LowerBound*100));
    fprintf('  鲁棒稳定裕度 = %.2f (参数可波动 %.0f%%)\n', ...
        stabmarg.LowerBound, stabmarg.LowerBound*100);
catch
    bar(1, 0, 'FaceColor', [0.8 0.8 0.8]);
    text(1, 0.5, 'robstab 不可用', 'HorizontalAlignment', 'center', 'FontSize', 12);
    set(gca, 'XTickLabel', {'鲁棒稳定裕度'});
    ylim([0, 2]); grid on;
    title('鲁棒稳定性 (robstab 不可用)');
end

fprintf('\n========================================\n');
fprintf('  教程 19 完成！\n');
fprintf('========================================\n\n');

fprintf('【H∞ vs LQR 对比总结】\n');
fprintf('  LQR: 最优性能 (名义模型) → 参数变化时性能退化\n');
fprintf('  H∞: 保证最坏情况性能 → 对所有不确定性都稳定\n');
fprintf('  取舍: H∞ 名义性能略低于 LQR，但鲁棒性强得多\n\n');

fprintf('【设计流程】\n');
fprintf('  1. 建立名义模型 + 不确定性边界\n');
fprintf('  2. 选择加权函数 W1/W2/W3（频域整形）\n');
fprintf('  3. hinfsyn 求解 H∞ 控制器\n');
fprintf('  4. μ 分析验证鲁棒稳定裕度 > 1\n');
fprintf('  5. 控制器降阶 → 非线性验证 → 部署\n');
