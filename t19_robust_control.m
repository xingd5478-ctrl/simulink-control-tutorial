%% ============================================================
% 教程 19：鲁棒控制 — H∞ 与 μ 综合入门
%
% 【为什么需要鲁棒控制？】
%   LQR 假设模型精确 → 但实际总有不确定性
%   H∞ 控制：设计时主动考虑"最坏情况"
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

m_nom = 1.0;  c_nom = 1.5;  k_nom = 10.0;

% 名义模型：质量-弹簧-阻尼  G(s) = 1/(m·s² + c·s + k)
G_nom = ss([0 1; -k_nom/m_nom -c_nom/m_nom], ...
           [0; 1/m_nom], [1 0], 0);

% 参数不确定性：±20% (需要 Robust Control Toolbox)
unc_valid = true;
try
    m_unc = ureal('m', 1.0, 'Percentage', 20);
    c_unc = ureal('c', 1.5, 'Percentage', 20);
    k_unc = ureal('k', 10.0, 'Percentage', 20);
catch ME
    unc_valid = false;
    fprintf('  ureal 不可用: %s\n', ME.message);
    fprintf('  → 需要 Robust Control Toolbox，将使用名义模型继续\n\n');
end

if unc_valid
    A_unc = [0 1; -k_unc/m_unc -c_unc/m_unc];
    B_unc = [0; 1/m_unc];
    G_unc = uss(A_unc, B_unc, [1 0], 0);
else
    % 回退：用名义模型+随机扰动模拟不确定性
    G_unc = G_nom;  % placeholder
end

% DC 增益补偿（Type-0 系统，稳态值 ≠ 1）
G0 = dcgain(G_nom);
Kr = 1 / (G0 / (1 + G0));    % 使闭环阶跃响应稳态值 = 1

fprintf('【名义系统 + 不确定性】\n');
fprintf('  名义模型: G(s)=1/(s²+%.1fs+%.0f), ωn=%.2f rad/s, ζ=%.2f\n', ...
    c_nom, k_nom, sqrt(k_nom/m_nom), c_nom/(2*sqrt(k_nom*m_nom)));
fprintf('  参数不确定: m=1.0±20%%, c=1.5±20%%, k=10.0±20%%\n');
fprintf('  参考增益 Kr=%.1f (DC补偿，使稳态=1)\n', Kr);
fprintf('  → 高频轻阻尼共振峰导致响应高度分散\n');
fprintf('  → LQR 单一名义模型无法保证所有情况下的性能\n\n');

%% ===== 图 1：不确定系统的分散表现 =====

fig1 = figure('Name', 't19_fig1_不确定系统', ...
    'Position', [100, 100, 1100, 420]);
t_vec = 0:0.01:5;

% --- 左：闭环阶跃响应包络 ---
subplot(1,2,1);
N_sample = 50;
y_all = zeros(N_sample, length(t_vec));
if unc_valid
    for i = 1:N_sample
        if unc_valid, Gi = usample(G_unc, 1); else, Gi = perturb_nominal(G_nom, 0.2); end
        [y_i, ~] = step(feedback(Kr*Gi, 1), t_vec);
        y_all(i, :) = y_i';
    end
else
    % 无 Robust Control Toolbox：用随机扰动模拟
    for i = 1:N_sample
        Gi = perturb_nominal(G_nom, 0.2);
        [y_i, ~] = step(feedback(Kr*Gi, 1), t_vec);
        y_all(i, :) = y_i';
    end
end

% 包络 + 名义
y_min  = min(y_all, [], 1);
y_max  = max(y_all, [], 1);
y_med  = median(y_all, 1);
[y_nom, ~] = step(feedback(Kr*G_nom, 1), t_vec);

h_fill = fill([t_vec, fliplr(t_vec)], [y_min, fliplr(y_max)], ...
    [0.75 0.82 0.95], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
hold on;
h_med = plot(t_vec, y_med, 'b', 'LineWidth', 1.5);
h_nom = plot(t_vec, y_nom, 'k--', 'LineWidth', 1.5);
hold off; grid on;

legend([h_fill, h_med, h_nom], {'包络 (min–max)', '中位数', '名义模型'}, ...
    'Location', 'southeast', 'FontSize', 8);
title(sprintf('闭环阶跃响应（%d 组随机采样）', N_sample), 'FontWeight', 'bold');
xlabel('时间 (s)'); ylabel('位移 (m)');
ylim([0, 1.4]);

% --- 右：幅频特性包络 ---
subplot(1,2,2);
w_bode = logspace(-1, 2, 200);
mag_all = zeros(N_sample, length(w_bode));
if unc_valid
    for i = 1:N_sample
        if unc_valid, Gi = usample(G_unc, 1); else, Gi = perturb_nominal(G_nom, 0.2); end
        [m, ~] = bode(feedback(Kr*Gi, 1), w_bode);
        mag_all(i, :) = squeeze(m)';
    end
else
    for i = 1:N_sample
        Gi = perturb_nominal(G_nom, 0.2);
        [m, ~] = bode(feedback(Kr*Gi, 1), w_bode);
        mag_all(i, :) = squeeze(m)';
    end
end

mag_min = min(mag_all, [], 1);
mag_max = max(mag_all, [], 1);
mag_med = median(mag_all, 1);
[mag_nom, ~] = bode(feedback(Kr*G_nom, 1), w_bode);

h_fill2 = fill([w_bode, fliplr(w_bode)], [mag_min, fliplr(mag_max)], ...
    [0.75 0.82 0.95], 'EdgeColor', 'none', 'FaceAlpha', 0.7);
hold on;
set(gca, 'XScale', 'log', 'YScale', 'log');
h_med2 = loglog(w_bode, mag_med, 'b', 'LineWidth', 1.5);
h_nom2 = loglog(w_bode, squeeze(mag_nom), 'k--', 'LineWidth', 1.5);
hold off; grid on;

legend([h_fill2, h_med2, h_nom2], {'包络 (min–max)', '中位数', '名义模型'}, ...
    'Location', 'southwest', 'FontSize', 8);
title('闭环幅频特性', 'FontWeight', 'bold');
xlabel('频率 (rad/s)'); ylabel('幅值');

%% ===== 第 2 步：加权函数 + H∞ 综合 =====

s = tf('s');

% W1: 低频大 → |S| 小 → 扰动抑制好
%     高频小 → 允许 |S|≈1（无约束）
%     交叉频率约 1 rad/s（低于共振频率 ωn=3.16，留足裕度）
W1 = (s/2.5 + 1) / (s + 0.1);   % DC×10, crossover≈1 rad/s

% W2: 限制控制信号能量（常值即可）
W2 = tf(0.2);

% W3: 低频小 → 允许 |T|≈1
%     高频大 → |T| 小 → 噪声/高频未建模动态抑制
W3 = (s + 8) / (s/20 + 1);      % DC≈8, HF→20

fprintf('【加权函数设计】\n');
fprintf('  W1 = (s/2.5+1)/(s+0.1)  → |S|≤-20dB@DC, 带宽≈1rad/s\n');
fprintf('  W2 = 0.2                → 控制量上限\n');
fprintf('  W3 = (s+8)/(s/20+1)     → |T|≤-26dB@HF, 噪声抑制\n');
fprintf('  (交叉频率低于 ωn=3.16，避开共振峰)\n\n');

% 构建广义对象 P
%   z = [W1; 0; 0] * r  +  [-W1*G; W2; W3*G] * u
%   e = [1]        * r  +  [-G]            * u

P11 = [W1; 0; 0];
P12 = [-W1*G_nom; W2; W3*G_nom];
P21 = 1;
P22 = -G_nom;
P = ss([P11, P12; P21, P22]);

fprintf('【H∞ 综合】\n');
K_valid = false;
try
    [K_hinf, ~, gamma] = hinfsyn(P, 1, 1);
    fprintf('  H∞ 范数 γ = %.3f\n', gamma);
    if gamma < 1
        fprintf('  → γ < 1，设计满足所有频域约束 ✓\n');
    else
        fprintf('  → γ ≥ 1，性能约束未完全满足，可放松权重\n');
    end
    K_valid = true;
catch ME
    fprintf('  hinfsyn 失败: %s\n', ME.message);
    fprintf('  → 尝试放松 W1/W2/W3 权重\n');
end
fprintf('\n');

% LQR 基准
Q = diag([100, 1]);  R = 1;
K_lqr = lqr(G_nom.A, G_nom.B, Q, R);

%% ===== 图 2：H∞ vs LQR 全面对比 =====

fig2 = figure('Name', 't19_fig2_H∞vsLQR', ...
    'Position', [100, 100, 1100, 750]);

% --- (1) H∞ 控制：不确定系统包络 ---
subplot(2,2,1);

y_hinf_all = zeros(N_sample, length(t_vec));
hinf_stable = 0;
if K_valid
    for i = 1:N_sample
        if unc_valid, Gi = usample(G_unc, 1); else, Gi = perturb_nominal(G_nom, 0.2); end
        [y_i, ~] = step(feedback(Kr*Gi*K_hinf, 1), t_vec);
        y_hinf_all(i, :) = y_i';
        hinf_stable = hinf_stable + 1;
    end
end

if hinf_stable > 0
    y_h_min = min(y_hinf_all(1:hinf_stable, :), [], 1);
    y_h_max = max(y_hinf_all(1:hinf_stable, :), [], 1);
    y_h_med = median(y_hinf_all(1:hinf_stable, :), 1);

    fill([t_vec, fliplr(t_vec)], [y_h_min, fliplr(y_h_max)], ...
        [0.85 0.75 0.75], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
    hold on;
    plot(t_vec, y_h_med, 'r', 'LineWidth', 1.5);
    hold off; grid on;
    title(sprintf('H∞ 控制 — %d 组不确定系统', hinf_stable), 'FontWeight', 'bold');
else
    text(0.5, 0.5, sprintf('H∞ 求解未收敛\n请调整权重函数'), ...
        'HorizontalAlignment', 'center', 'FontSize', 11);
    title('H∞ 控制 (未收敛)', 'FontWeight', 'bold');
end
xlabel('时间 (s)'); ylabel('位移 (m)');
ylim([0, 1.6]);

% --- (2) LQR 控制：不确定系统包络 ---
subplot(2,2,2);

y_lqr_all = zeros(N_sample, length(t_vec));
for i = 1:N_sample
    if unc_valid, Gi = usample(G_unc, 1); else, Gi = perturb_nominal(G_nom, 0.2); end
    A_cl = Gi.A - Gi.B * K_lqr;
    [y_i, ~] = step(ss(A_cl, Gi.B, Gi.C, 0) * Kr, t_vec);
    y_lqr_all(i, :) = y_i';
end

y_l_min = min(y_lqr_all, [], 1);
y_l_max = max(y_lqr_all, [], 1);
y_l_med = median(y_lqr_all, 1);

fill([t_vec, fliplr(t_vec)], [y_l_min, fliplr(y_l_max)], ...
    [0.75 0.82 0.95], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
hold on;
plot(t_vec, y_l_med, 'b', 'LineWidth', 1.5);
hold off; grid on;
title(sprintf('LQR 控制 — %d 组不确定系统', N_sample), 'FontWeight', 'bold');
xlabel('时间 (s)'); ylabel('位移 (m)');
ylim([0, 1.6]);

% --- (3) 名义模型对比 ---
subplot(2,2,3); hold on;

A_cl_lqr = G_nom.A - G_nom.B * K_lqr;
[y_nl, ~] = step(ss(A_cl_lqr, G_nom.B, G_nom.C, 0) * Kr, t_vec);
plot(t_vec, y_nl, 'b', 'LineWidth', 1.5);
if K_valid
    [y_nh, ~] = step(feedback(Kr*G_nom*K_hinf, 1), t_vec);
    plot(t_vec, y_nh, 'r', 'LineWidth', 1.5);
end

lgd3 = {'LQR'};
if K_valid, lgd3{end+1} = 'H∞'; end
legend(lgd3, 'Location', 'best', 'FontSize', 9);
hold off; grid on;
title('名义模型响应', 'FontWeight', 'bold');
xlabel('时间 (s)'); ylabel('位移 (m)');

% --- (4) 灵敏度 |S| 对比 ---
subplot(2,2,4); hold on;

w_sv = logspace(-3, 3, 300);
lgd4 = {};

if K_valid
    S_hinf = feedback(1, G_nom * K_hinf);
    [mag_S, ~] = bode(S_hinf, w_sv);
    loglog(w_sv, squeeze(mag_S), 'r', 'LineWidth', 1.5);
    lgd4{end+1} = '|S| H∞';
end

[mag_iW1, ~] = bode(1/W1, w_sv);
loglog(w_sv, squeeze(mag_iW1), 'k--', 'LineWidth', 1.2);
lgd4{end+1} = '1/W_1 (上界约束)';

% 标记约束违反区域（若 γ>1）
if K_valid
    [magSt, ~] = bode(S_hinf, w_sv);
    [magWt, ~] = bode(1/W1, w_sv);
    viol = squeeze(magSt)' > squeeze(magWt)';
    if any(viol)
        viol_w = w_sv(viol);
        viol_lo = min(viol_w);
        viol_hi = max(viol_w);
        yl = ylim;
        fill([viol_lo viol_hi viol_hi viol_lo], [yl(1) yl(1) yl(2) yl(2)], ...
            [1 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.2);
        text(sqrt(viol_lo*viol_hi), yl(2)*0.5, ...
            {sprintf('|S| 超约束区'), sprintf('(γ=%.1f>1)', gamma)}, ...
            'HorizontalAlignment', 'center', 'FontSize', 8, 'Color', [0.8 0.2 0.2]);
    end
end

hold off; grid on;
legend(lgd4, 'Location', 'southwest', 'FontSize', 9);
title('灵敏度 |S| = |1/(1+GK)|', 'FontWeight', 'bold');
xlabel('频率 (rad/s)'); ylabel('幅值');

%% ===== 图 3：加权函数 + 鲁棒稳定裕度 =====

fig3 = figure('Name', 't19_fig3_权函数与稳定性', ...
    'Position', [100, 100, 1100, 420]);

% --- 左：三个加权函数倒数 ---
subplot(1,2,1); hold on;

w_wt = logspace(-3, 3, 300);
[mag1, ~] = bode(1/W1, w_wt);
[mag2, ~] = bode(1/W2, w_wt);
[mag3, ~] = bode(1/W3, w_wt);

loglog(w_wt, squeeze(mag1), 'b', 'LineWidth', 1.5);
loglog(w_wt, squeeze(mag2), 'Color', [0 0.6 0], 'LineWidth', 1.5);
loglog(w_wt, squeeze(mag3), 'r', 'LineWidth', 1.5);

% 频段标注（数据坐标）
xl = xlim;
loglog([1e-3 0.3], [80 80], 'b--', 'LineWidth', 0.8);
text(0.01, 100, {'低频段 → |S| 小', '(扰动抑制)'}, 'FontSize', 8, 'Color', 'b');
loglog([8 1e3], [3e-2 3e-2], 'r--', 'LineWidth', 0.8);
text(50, 0.07, {'高频段 → |T| 小', '(噪声抑制)'}, 'FontSize', 8, 'Color', 'r');

hold off;
legend('1/W_1 (S 上界)', '1/W_2 (KS 上界)', '1/W_3 (T 上界)', ...
    'Location', 'southwest', 'FontSize', 8);
title('加权函数倒数 — 频域整形目标', 'FontWeight', 'bold');
xlabel('频率 (rad/s)'); ylabel('幅值'); grid on;

% --- 右：鲁棒稳定裕度 ---
subplot(1,2,2); hold on;

try
    [stabmarg, ~] = robstab(G_unc);
    sm = stabmarg.LowerBound;

    % 彩色柱状图，颜色随裕度变化
    if sm >= 2
        bar_color = [0.2 0.7 0.3];   % 绿色：良好
    elseif sm >= 1
        bar_color = [0.95 0.6 0.1];  % 橙色：可接受
    else
        bar_color = [0.85 0.2 0.2];  % 红色：不足
    end

    bar(1, sm, 'FaceColor', bar_color, 'BarWidth', 0.45, ...
        'EdgeColor', 'none');
    yline(1, 'r--', 'LineWidth', 1.5);
    yline(2, ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);

    text(1, sm + 0.3, sprintf('%.1f×', sm), ...
        'HorizontalAlignment', 'center', 'FontSize', 18, 'FontWeight', 'bold', ...
        'Color', bar_color);

    xlim([0.3, 1.7]);
    ylim([0, max(3, sm + 1)]);
    set(gca, 'XTick', 1, 'XTickLabel', {'鲁棒稳定裕度'});
    ylabel('裕度倍数'); grid on;

    if sm >= 2
        status_str = sprintf('优秀：参数可波动 ±%.0f%% 仍保证稳定', (sm-1)*100);
    elseif sm >= 1
        status_str = sprintf('可接受：参数可波动 ±%.0f%% 仍保证稳定', (sm-1)*100);
    else
        status_str = '不足：部分采样点可能不稳定';
    end
    title(status_str, 'FontWeight', 'bold');
    fprintf('  鲁棒稳定裕度 = %.2f×  (%s)\n', sm, status_str);
catch
    text(0.5, 0.5, 'robstab 不可用\n(需要 Robust Control Toolbox)', ...
        'HorizontalAlignment', 'center', 'FontSize', 12);
    xlim([0, 1]); ylim([0, 1]);
    title('鲁棒稳定性 (不可用)', 'FontWeight', 'bold');
end
hold off;

%% ===== 保存图像到 docs/images/ =====

outDir = fullfile(fileparts(mfilename('fullpath')), 'docs', 'images');
if ~exist(outDir, 'dir'), mkdir(outDir); end

figs = flipud(findall(0, 'Type', 'figure'));
fig_names = {'t19_robust_control_fig1', 't19_robust_control_fig2', 't19_robust_control_fig3'};
for k = 1:min(numel(figs), numel(fig_names))
    file = fullfile(outDir, [fig_names{k}, '.png']);
    try
        exportgraphics(figs(k), file, 'Resolution', 150);
    catch
        saveas(figs(k), file);
    end
end

fprintf('\n========================================\n');
fprintf('  教程 19 完成！图像已保存到 docs/images/\n');
fprintf('========================================\n\n');

fprintf('【H∞ vs LQR 对比总结】\n');
fprintf('  LQR: 名义模型最优 → 参数变化时性能退化严重\n');
fprintf('  H∞: 主动考虑最坏情况 → 在不确定集内全局鲁棒\n\n');

fprintf('【设计流程】\n');
fprintf('  1. 建立名义模型 + 不确定性边界 (ureal/uss)\n');
fprintf('  2. 根据性能需求选择 W1/W2/W3 加权函数\n');
fprintf('  3. 构建广义对象 P → hinfsyn 求解\n');
fprintf('  4. γ < 1 → 满足约束；否则放松权重\n');
fprintf('  5. μ 分析 (robstab) 验证鲁棒稳定裕度\n');
fprintf('  6. 降阶 → 非线性仿真验证 → 硬件部署\n');

%% ===== 辅助函数 =====

function Gs = perturb_nominal(G_nom, pct)
% 随机扰动名义模型参数（无 Robust Control Toolbox 时的回退方案）
    pert = 1 + pct * (2*rand - 1);
    Gs = G_nom * pert;
end
