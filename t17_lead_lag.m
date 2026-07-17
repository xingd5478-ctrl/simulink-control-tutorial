%% ============================================================
% 教程 17：超前-滞后校正器 — 根轨迹与频域设计
%
% 【校正器是什么？】
%   系统响应不够好 → 加一个校正器 C(s) 串联进去
%   超前校正 (Lead)    → 改善动态响应（加相位裕度、加速）
%   滞后校正 (Lag)     → 改善稳态精度（增低频增益）
%   超前-滞后 (Lead-Lag) → 两者兼顾
%
% 【传递函数形式】
%   Lead:  C(s) = K * (αTs + 1) / (Ts + 1),   α > 1
%   Lag:   C(s) = K * (Ts + 1) / (βTs + 1),   β > 1
%
% 【设计思路】
%   1. 看根轨迹 → 确定需要把极点往哪移
%   2. 看 Bode 图 → 确定需要补多少相位
%   3. 计算校正器参数
%   4. 验证效果
% ============================================================

clear; close all;

fprintf('============================================\n');
fprintf('  教程 17：超前-滞后校正器设计\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：原始系统 — 看看哪不好 =====

% 三阶系统：慢 + 超调大
sys = tf([5], [1, 5, 7, 3]);  % G(s) = 5/(s³+5s²+7s+3)

fprintf('【原始系统】G(s) = 5 / (s³ + 5s² + 7s + 3)\n\n');

% 原始阶跃响应
figure('Name', 't17: 校正器设计对比', ...
    'Position', [50, 50, 1000, 700]);

subplot(2,3,1);
step(feedback(sys, 1), 0:0.01:20);
title('原始闭环阶跃响应'); grid on;
S1 = stepinfo(feedback(sys, 1));
fprintf('  原始性能：\n');
fprintf('    上升时间 = %.2f s\n', S1.RiseTime);
fprintf('    调节时间 = %.2f s\n', S1.SettlingTime);
fprintf('    超调量   = %.1f%%\n', S1.Overshoot);
fprintf('    稳态误差 = %.1f%%\n\n', abs(1-S1.SettlingMax)*100);

%% ===== 第 2 步：根轨迹 — 看极点在哪 =====

subplot(2,3,2);
rlocus(sys); grid on;
title('原始系统根轨迹');
fprintf('【根轨迹分析】\n');
fprintf('  开环极点: ');
p = pole(sys);
for i = 1:length(p)
    fprintf('%.2f ', p(i));
end
fprintf('\n  → 所有极点都在负实轴上 → 无振荡，但慢\n\n');

%% ===== 第 3 步：Bode 图 — 看频域表现 =====

subplot(2,3,3);
margin(sys); grid on;
[GM, PM] = margin(sys);
fprintf('【频域分析】\n');
fprintf('  PM = %.1f° — 相位裕度不错\n', PM);
fprintf('  → 但低频增益低 → 稳态误差大\n');
fprintf('  → 首先需要一个 Lag 校正器提升低频增益\n\n');

%% ===== 第 4 步：设计滞后校正器 (Lag) =====

% Lag 补偿器：在低频提供额外增益，不影响高频相位
% C_lag(s) = (Ts + 1) / (βTs + 1), β > 1

beta = 10;    % 低频增益提升倍数
T_lag = 5;    % 转折频率在低频
C_lag = tf([T_lag, 1], [beta*T_lag, 1]);

subplot(2,3,4);
step(feedback(C_lag*sys, 1), 0:0.01:20);
title('Lag 校正后阶跃响应'); grid on;
S2 = stepinfo(feedback(C_lag*sys, 1));
fprintf('【Lag 校正后】\n');
fprintf('  稳态误差大幅减小\n');
fprintf('  超调量 = %.1f%%\n', S2.Overshoot);
fprintf('  但响应速度仍慢（上升时间 = %.2f s）\n\n', S2.RiseTime);

%% ===== 第 5 步：设计超前校正器 (Lead) =====

% Lead 补偿器：在高频提供相位超前，加速响应
% C_lead(s) = (αTs + 1) / (Ts + 1), α > 1

alpha = 5;      % 最大相位超前 ≈ arcsin((α-1)/(α+1))
T_lead = 0.5;
C_lead = tf([alpha*T_lead, 1], [T_lead, 1]);

subplot(2,3,5);
step(feedback(C_lead*sys, 1), 0:0.01:20);
title('Lead 校正后阶跃响应'); grid on;
S3 = stepinfo(feedback(C_lead*sys, 1));
fprintf('【Lead 校正后】\n');
fprintf('  上升时间缩短 = %.2f s\n', S3.RiseTime);
fprintf('  超调量 = %.1f%%\n\n', S3.Overshoot);

%% ===== 第 6 步：超前-滞后 (Lead-Lag) =====

C_ll = C_lead * C_lag;

subplot(2,3,6);
step(feedback(C_ll*sys, 1), 0:0.01:20);
title('Lead-Lag 校正后阶跃响应'); grid on;
S4 = stepinfo(feedback(C_ll*sys, 1));
fprintf('【Lead-Lag 校正后】\n');
fprintf('  上升时间 = %.2f s\n', S4.RiseTime);
fprintf('  调节时间 = %.2f s\n', S4.SettlingTime);
fprintf('  超调量   = %.1f%%\n', S4.Overshoot);
fprintf('  → 既有速度，又有精度！\n\n');

%% ===== 第 7 步：校正器频域对比 =====

figure('Name', 't17: 校正器频域特性', ...
    'Position', [50, 50, 900, 400]);

subplot(1,2,1); hold on;
bode(C_lag, 'b'); bode(C_lead, 'r'); bode(C_ll, 'm');
legend('Lag', 'Lead', 'Lead-Lag', 'Location', 'southwest');
title('各校正器 Bode 图'); grid on;

subplot(1,2,2); hold on;
step(feedback(sys, 1), 0:0.01:20);
step(feedback(C_lag*sys, 1), 0:0.01:20);
step(feedback(C_lead*sys, 1), 0:0.01:20);
step(feedback(C_ll*sys, 1), 0:0.01:20);
legend('原始', 'Lag', 'Lead', 'Lead-Lag', 'Location', 'best');
title('各方案阶跃响应对比'); grid on;

fprintf('========================================\n');
fprintf('  教程 17 完成！\n');
fprintf('========================================\n\n');

fprintf('【校正器设计口诀】\n');
fprintf('  Lag (滞后): 加低频增益 → 减稳态误差\n');
fprintf('  Lead (超前): 加高频相位 → 加响应速度\n');
fprintf('  Lead+PI ≈ Lead-Lag → 工程最常用组合\n');
fprintf('\n');
fprintf('【设计流程】\n');
fprintf('  1. 画出原始系统的 Bode 图 + 根轨迹\n');
fprintf('  2. 确定目标：要速度还是精度？\n');
fprintf('  3. 选校正器类型，计算参数\n');
fprintf('  4. 验证：阶跃响应 + Bode 图 + 根轨迹\n');
fprintf('  5. 迭代调整直到满意\n');
