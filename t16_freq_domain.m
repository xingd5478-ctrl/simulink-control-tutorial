%% ============================================================
% 教程 16：频域分析 — Bode图、Nyquist图与稳定裕度
%
% 【为什么需要频域分析？】
%   时域（阶跃响应）看的是"快不快、振不振"
%   频域看的是"系统对每个频率的正弦输入会怎么响应"
%
%   频域工具是控制工程师的"听诊器"：
%     - Bode图 → 看系统对不同频率的放大/衰减
%     - Nyquist图 → 判断闭环是否稳定
%     - 增益裕度(GM)和相位裕度(PM) → 量化稳定程度
%
% 【核心概念】
%   传递函数 G(s) → 令 s = jω → G(jω) = |G|∠θ
%     |G(jω)| = 幅值响应（输出幅值/输入幅值）
%     ∠G(jω)  = 相位响应（输出延迟角度）
% ============================================================

clear; close all;

fprintf('============================================\n');
fprintf('  教程 16：频域分析 — Bode / Nyquist / Nichols\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：构建示例系统 =====

% 质量-弹簧-阻尼系统（二阶）
m = 1.0;  c = 0.3;  k = 10.0;

sys = tf([1], [m, c, k]);  % G(s) = 1/(s² + 0.3s + 10)

fprintf('【示例系统】质量-弹簧-阻尼\n');
fprintf('  G(s) = 1 / (s² + %.1fs + %.0f)\n', c, k);
fprintf('  自然频率 ωn = %.2f rad/s (%.2f Hz)\n', sqrt(k/m), sqrt(k/m)/(2*pi));
fprintf('  阻尼比 ζ = %.3f\n\n', c/(2*sqrt(m*k)));

%% ===== 第 2 步：Bode 图 — 最常用的频域工具 =====

figure('Name', 't16: Bode图', 'Position', [50, 50, 900, 600]);

% Bode 图是两幅图：幅频 + 相频
[mag, phase, w] = bode(sys);
mag = squeeze(mag);
phase = squeeze(phase);

subplot(2,1,1);
loglog(w, mag, 'b', 'LineWidth', 1.5); grid on;
ylabel('幅值 |G(jω)|'); xlabel('频率 (rad/s)');
title('Bode 图：幅频特性');

% 标注自然频率
[~, idx] = min(abs(w - sqrt(k/m)));
hold on; plot(w(idx), mag(idx), 'ro', 'MarkerSize', 8, 'LineWidth', 2);
text(w(idx)*1.3, mag(idx), sprintf('ω_n=%.1f', sqrt(k/m)), 'FontSize', 10);
hold off;

subplot(2,1,2);
semilogx(w, phase, 'b', 'LineWidth', 1.5); grid on;
ylabel('相位 (度)'); xlabel('频率 (rad/s)');
title('Bode 图：相频特性');

fprintf('【Bode 图解读】\n');
fprintf('  幅频：低频 → 幅值 ≈ 1/k = %.3f（稳态增益）\n', 1/k);
fprintf('        高频 → 幅值以 -40dB/dec 衰减（二阶系统特性）\n');
fprintf('  相频：低频 → 相位 ≈ 0°（同相）\n');
fprintf('        高频 → 相位 → -180°（反相，二阶极限）\n');
fprintf('  ω_n 处有谐振峰（阻尼比 ζ < 0.707 时出现）\n\n');

%% ===== 第 3 步：Nyquist 图 — 判断闭环稳定性 =====

figure('Name', 't16: Nyquist图', 'Position', [50, 50, 600, 500]);

% 创建开环传递函数（带增益，使得 Nyquist 图包裹 (-1,0) 点）
K = 15;
sysOL = K * sys;  % 开环传递函数

nyquist(sysOL); grid on;
title(sprintf('Nyquist 图 (K = %.0f)', K));

fprintf('【Nyquist 判据】\n');
fprintf('  开环 G(s) 稳定（极点全在 LHP）\n');
fprintf('  → 如果 Nyquist 曲线逆时针包围 (-1,0) 点 = 0 次 → 闭环稳定\n');
fprintf('  → 如果包围 (-1,0) 点 N 次 → 闭环有 N 个不稳定极点\n\n');

%% ===== 第 4 步：稳定裕度 — 量化"有多稳定" =====

figure('Name', 't16: 稳定裕度', 'Position', [50, 50, 800, 500]);

margin(sysOL);  % 自动标注 GM 和 PM
title('增益裕度 (GM) 和 相位裕度 (PM)');

[GM, PM, wcg, wcp] = margin(sysOL);

fprintf('【稳定裕度】\n');
fprintf('  增益裕度 GM = %.2f dB (%.2f 倍)\n', 20*log10(GM), GM);
fprintf('    → 增益可以再放大 %.1f 倍才失稳\n', GM);
fprintf('  相位裕度 PM = %.1f°\n', PM);
fprintf('    → 相位最多还能滞后 %.0f° 才失稳\n', PM);
fprintf('  穿越频率 ω_cp = %.2f rad/s（对应闭环带宽）\n', wcp);
fprintf('\n  工程经验值：\n');
fprintf('    GM > 6dB, PM > 30° → 可行但偏小\n');
fprintf('    GM > 10dB, PM > 45° → 良好的鲁棒性\n');
fprintf('    PM > 60° → 无超调的保守设计\n\n');

%% ===== 第 5 步：频域与时域的关系 =====

figure('Name', 't16: 频域-时域对比', 'Position', [50, 50, 900, 400]);

K_list = [1, 5, 15, 30];
t = 0:0.01:10;

subplot(1,2,1); hold on;
colors = lines(length(K_list));
for i = 1:length(K_list)
    sysCL = feedback(K_list(i)*sys, 1);
    [y, t] = step(sysCL, t);
    plot(t, y, 'Color', colors(i,:), 'LineWidth', 1.5);
end
hold off; grid on;
legend('K=1', 'K=5', 'K=15', 'K=30', 'Location', 'best');
title('时域：不同增益下的阶跃响应');
xlabel('时间 (s)'); ylabel('输出');

subplot(1,2,2); hold on;
for i = 1:length(K_list)
    [GM_i, PM_i] = margin(K_list(i)*sys);
end
% 手动画出幅频对比
for i = 1:length(K_list)
    [mag_i, ~, w_i] = bode(K_list(i)*sys);
    mag_i = squeeze(mag_i);
    loglog(w_i, mag_i, 'Color', colors(i,:), 'LineWidth', 1.5);
end
grid on;
legend('K=1', 'K=5', 'K=15', 'K=30', 'Location', 'best');
title('频域：不同增益下的幅频特性');
xlabel('频率 (rad/s)'); ylabel('幅值');
set(gca, 'XScale', 'log', 'YScale', 'log');
hold off;

fprintf('========================================\n');
fprintf('  教程 16 完成！\n');
fprintf('========================================\n\n');

fprintf('【关键结论】\n');
fprintf('  1. Bode图 = 控制工程师的"心电图"\n');
fprintf('  2. GM/PM 越大 → 系统越稳定，但响应越慢\n');
fprintf('  3. 穿越频率 ≈ 闭环带宽 ≈ 响应速度\n');
fprintf('  4. 增益 K 越大 → 带宽越大 → 响应越快 → GM/PM 越小\n');
fprintf('  5. 频域设计的目标：在带宽和稳定裕度之间取得平衡\n');
fprintf('\n试着改 K 值，观察 Bode 图和阶跃响应的变化！\n');
