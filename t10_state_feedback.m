%% ============================================================
% 教程 10：状态反馈控制 — 极点配置 & LQR 最优控制
%
% 【理论知识】
%   t09 我们学会了"描述系统"（状态空间建模）
%   t10 我们学习"控制系统"（设计状态反馈）
%
%   控制目标：让质量-弹簧-阻尼系统按照我们想要的方式运动
%
%  ┌─────────────────────────────────────────────────────────┐
%  │ 一、开环 vs 闭环                                        │
%  ├─────────────────────────────────────────────────────────┤
%  │                                                         │
%  │  开环（t09）：  u = F(t)       输入直接加到系统         │
%  │  闭环（t10）：  u = -K·x + r   根据状态计算控制力       │
%  │                                                         │
%  │  闭环方程：  ẋ = Ax + B(-Kx + r) = (A-BK)x + Br         │
%  │                                                         │
%  │  关键洞察：通过选择 K，我们可以任意改变 (A-BK) 的特征值 │
%  │           → 也就是改变系统的动态特性！                  │
%  └─────────────────────────────────────────────────────────┘
%
%  ┌─────────────────────────────────────────────────────────┐
%  │ 二、极点配置 (Pole Placement)                           │
%  ├─────────────────────────────────────────────────────────┤
%  │                                                         │
%  │  开环极点：eig(A) = -0.25 ± j1.984                      │
%  │    → ωn=2, ζ=0.125 → 振荡剧烈，衰减很慢                 │
%  │                                                         │
%  │  我们希望：更快的响应 + 合适的阻尼                       │
%  │  目标极点：ζ=0.707, ωn=4 → s = -2.83 ± j2.83            │
%  │                                                         │
%  │  MATLAB 一行搞定： K = place(A, B, [p1, p2])            │
%  └─────────────────────────────────────────────────────────┘
%
%  ┌─────────────────────────────────────────────────────────┐
%  │ 三、LQR 最优控制 (Linear Quadratic Regulator)           │
%  ├─────────────────────────────────────────────────────────┤
%  │                                                         │
%  │  极点配置：我们知道"要什么"，手动指定极点               │
%  │  LQR：我们定义"什么重要"，让算法自动算最优 K            │
%  │                                                         │
%  │  代价函数：                                             │
%  │    J = ∫₀^∞ (x^T Q x + u^T R u) dt                     │
%  │                                                         │
%  │  Q：状态偏差的惩罚权重（"位置不准有多严重？"）           │
%  │  R：控制能量的惩罚权重（"用力太猛有多浪费？"）           │
%  │                                                         │
%  │  求解：K = R⁻¹ B^T P，其中 P 是 Riccati 方程的解        │
%  │  MATLAB 一行搞定： K = lqr(A, B, Q, R)                  │
%  └─────────────────────────────────────────────────────────┘
%
%  ┌─────────────────────────────────────────────────────────┐
%  │ 四、极点配置 vs LQR — 什么时候用哪个？                  │
%  ├─────────────────────────────────────────────────────────┤
%  │                                                         │
%  │  极点配置：                                             │
%  │    + 直观（"我要 2 秒内稳定"）                          │
%  │    - 多变量时很难手动选所有极点                         │
%  │    - 不考虑控制能量的代价                               │
%  │                                                         │
%  │  LQR：                                                  │
%  │    + 自动折中性能 vs 能量                               │
%  │    + 适合 MIMO，自动处理耦合                             │
%  │    - Q/R 的选择需要经验                                 │
%  │    - 黑箱感较强                                         │
%  └─────────────────────────────────────────────────────────┘
%
% 【本课目标】
%   1. 计算开环极点，理解系统固有特性
%   2. 用极点配置设计 K，观察闭环响应
%   3. 用 LQR 设计 K，对比不同 Q/R 的效果
%   4. 理解 Q/R 的工程含义
% ============================================================

clear; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

%% ===== 系统参数（沿用 t09 的质量-弹簧-阻尼）=====

m = 1.0;    % 质量
c = 0.5;    % 阻尼
k = 4.0;    % 弹簧刚度

A = [   0,     1  ;
      -k/m,  -c/m ];

B = [  0  ;
      1/m ];

C = [ 1, 0 ];    % 测量位移
D = 0;

fprintf('============================================\n');
fprintf('  教程 10：状态反馈控制\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：分析开环系统 =====

openLoopPoles = eig(A);
zeta_open = -real(openLoopPoles(1)) / abs(openLoopPoles(1));
wn_open = abs(openLoopPoles(1));

fprintf('【开环分析】\n');
fprintf('  状态矩阵 A 的特征值：\n');
p1 = openLoopPoles(1); p2 = openLoopPoles(2);
fprintf('    s₁ = %.2f %s j%.2f\n', real(p1), signStr(imag(p1)), abs(imag(p1)));
fprintf('    s₂ = %.2f %s j%.2f\n', real(p2), signStr(imag(p2)), abs(imag(p2)));
fprintf('  → 阻尼比 ζ = %.3f  (%.1f%% 超调)\n', zeta_open, 100*exp(-pi*zeta_open/sqrt(1-zeta_open^2)));
fprintf('  → 固有频率 ωn = %.2f rad/s\n', wn_open);
fprintf('  → 稳定时间 ts ≈ %.1f s  (2%% 准则)\n\n', 4/(zeta_open*wn_open));

fprintf('  问题：振荡太剧烈，稳定太慢！\n');
fprintf('  解决方案：引入状态反馈 u = -Kx，改变闭环极点。\n\n');

%% ===== 第 2 步：极点配置设计 =====

% 目标极点：ζ=0.707 (最优阻尼), ωn=4 (比开环快一倍)
zeta_desired = 0.707;
wn_desired = 4.0;

p_desired = roots([1, 2*zeta_desired*wn_desired, wn_desired^2]);

fprintf('【极点配置设计】\n');
fprintf('  目标阻尼 ζ=%.3f, 目标频率 ωn=%.1f\n', zeta_desired, wn_desired);
fprintf('  目标极点：s₁ = %.2f + j%.2f\n', real(p_desired(1)), imag(p_desired(1)));
fprintf('            s₂ = %.2f - j%.2f\n\n', real(p_desired(2)), imag(p_desired(2)));

% 计算反馈增益
K_place = place(A, B, p_desired);

fprintf('  设计的反馈增益：K = [ %.4f,  %.4f ]\n', K_place(1), K_place(2));
fprintf('  物理含义：\n');
fprintf('    u = -%.4f × x₁ - %.4f × x₂\n', K_place(1), K_place(2));
fprintf('    u = -%.4f × (位移) - %.4f × (速度)\n\n', K_place(1), K_place(2));

% 验证闭环极点
Acl_place = A - B*K_place;
closedLoopPoles_place = eig(Acl_place);
fprintf('  验证闭环极点（应等于目标）：\n');
fprintf('    s₁ = %.2f + j%.2f\n', real(closedLoopPoles_place(1)), imag(closedLoopPoles_place(1)));
fprintf('    s₂ = %.2f - j%.2f\n', real(closedLoopPoles_place(2)), imag(closedLoopPoles_place(2)));

% 计算闭环稳定时间
zeta_cl = -real(closedLoopPoles_place(1)) / abs(closedLoopPoles_place(1));
wn_cl = abs(closedLoopPoles_place(1));
ts_cl = 4/(zeta_cl * wn_cl);
overshoot_cl = 100*exp(-pi*zeta_cl/sqrt(1-zeta_cl^2));
fprintf('  → 稳定时间 ts ≈ %.1f s, 超调 ≈ %.1f%%\n\n', ts_cl, overshoot_cl);

%% ===== 第 3 步：LQR 最优控制设计 =====

fprintf('【LQR 最优控制设计】\n\n');

% LQR 设计 1：强调位置精度（Q 大，R 小）
Q1 = diag([100, 1]);   % 位置误差很重要，速度次要
R1 = 1;                 % 能量成本适中
[K_lqr1, S1, e1] = lqr(A, B, Q1, R1);

fprintf('  LQR-1: Q=[100,0;0,1], R=1 (位置精度优先)\n');
fprintf('    K = [ %.4f,  %.4f ]\n', K_lqr1(1), K_lqr1(2));
fprintf('    闭环极点: %.2f±j%.2f\n\n', real(e1(1)), abs(imag(e1(1))));

% LQR 设计 2：平衡性能和能量
Q2 = diag([10, 1]);
R2 = 1;
[K_lqr2, S2, e2] = lqr(A, B, Q2, R2);

fprintf('  LQR-2: Q=[10,0;0,1], R=1 (均衡)\n');
fprintf('    K = [ %.4f,  %.4f ]\n', K_lqr2(1), K_lqr2(2));
fprintf('    闭环极点: %.2f±j%.2f\n\n', real(e2(1)), abs(imag(e2(1))));

% LQR 设计 3：节省能量（Q 小，R 大）
Q3 = diag([1, 1]);
R3 = 10;
[K_lqr3, S3, e3] = lqr(A, B, Q3, R3);

fprintf('  LQR-3: Q=[1,0;0,1], R=10 (节能优先)\n');
fprintf('    K = [ %.4f,  %.4f ]\n', K_lqr3(1), K_lqr3(2));
fprintf('    闭环极点: %.2f±j%.2f\n\n', real(e3(1)), abs(imag(e3(1))));

fprintf('  观察：K 的值越大 → 控制越"猛" → 响应越快但能量消耗大\n');
fprintf('        K 的值越小 → 控制越"柔" → 响应慢但省能量\n\n');

%% ===== 第 4 步：搭建 Simulink 模型 =====

mdl = 'tutorial10_state_feedback';
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

% --- 公共信号源：阶跃参考输入 r(t) ---
% 我们希望位移追踪 1m（即 r=1）
add_block('simulink/Sources/Step', [mdl '/Setpoint r=1'], ...
    'Position', [50, 60, 130, 100]);
set_param([mdl '/Setpoint r=1'], ...
    'Time', '0.5', ...
    'Before', '0', ...
    'After', '1');

% 布局：从上到下 4 行，分别为：
%   行 1：开环（无反馈）
%   行 2：极点配置闭环
%   行 3：LQR-1 (位置优先)
%   行 4：LQR-3 (节能优先)
%
% 每行结构：
%   r → (+) → [Plant SS] → y
%         ↑       │
%         └─[K]─←─┘  (状态反馈)

rowY = [180, 270, 360, 450];  % 每行的 Y 坐标
rowTags = {'OL', 'PP', 'LQ1', 'LQ3'};  % 短标签用于模块命名
K_mats = {[0, 0], K_place, K_lqr1, K_lqr3};

for iRow = 1:4
    y0 = rowY(iRow);
    tag = rowTags{iRow};  % 平铺命名用

    % --- 误差计算（r - y）---
    add_block('simulink/Math Operations/Add', [mdl '/' tag '_Error'], ...
        'Position', [150, y0, 180, y0+30]);
    set_param([mdl '/' tag '_Error'], 'Inputs', '|+-');

    % --- 状态反馈增益 K ---
    actualK = K_mats{iRow};

    % Demux: 把状态向量 [x1; x2] 拆成两个标量
    add_block('simulink/Signal Routing/Demux', [mdl '/' tag '_Demux'], ...
        'Position', [350, y0-15, 370, y0+55]);
    set_param([mdl '/' tag '_Demux'], 'Outputs', '2');

    % Gain K(1) for state x1
    add_block('simulink/Math Operations/Gain', [mdl '/' tag '_K1'], ...
        'Position', [400, y0-10, 440, y0+15]);
    set_param([mdl '/' tag '_K1'], 'Gain', num2str(actualK(1)));

    % Gain K(2) for state x2
    add_block('simulink/Math Operations/Gain', [mdl '/' tag '_K2'], ...
        'Position', [400, y0+30, 440, y0+55]);
    set_param([mdl '/' tag '_K2'], 'Gain', num2str(actualK(2)));

    % Sum K1*x1 + K2*x2
    add_block('simulink/Math Operations/Add', [mdl '/' tag '_SumK'], ...
        'Position', [470, y0-5, 500, y0+45]);
    set_param([mdl '/' tag '_SumK'], 'Inputs', '|++', 'IconShape', 'round');

    % --- 被控对象（State-Space 模块）---
    add_block('simulink/Continuous/State-Space', [mdl '/' tag '_Plant'], ...
        'Position', [250, y0-5, 310, y0+45]);
    set_param([mdl '/' tag '_Plant'], ...
        'A', mat2str(A), ...
        'B', mat2str(B), ...
        'C', 'eye(2)', ...     % 输出完整状态 [x1; x2] 供反馈使用
        'D', 'zeros(2,1)', ...
        'X0', '[0; 0]');

    % --- To Workspace ---
    add_block('simulink/Sinks/To Workspace', [mdl '/' tag '_y'], ...
        'Position', [540, y0-5, 600, y0+25]);
    set_param([mdl '/' tag '_y'], 'VariableName', ['y_' tag]);

    add_block('simulink/Sinks/To Workspace', [mdl '/' tag '_u'], ...
        'Position', [540, y0+35, 600, y0+65]);
    set_param([mdl '/' tag '_u'], 'VariableName', ['u_' tag]);

    % --- 连线 ---
    % 参考输入 r → Error (+)
    add_line(mdl, 'Setpoint r=1/1', [tag '_Error/1']);

    % Error → Plant
    add_line(mdl, [tag '_Error/1'], [tag '_Plant/1']);

    % Plant → Demux
    add_line(mdl, [tag '_Plant/1'], [tag '_Demux/1']);

    % Demux → K1 (output port 1 = x1)
    add_line(mdl, [tag '_Demux/1'], [tag '_K1/1']);
    % Demux → K2 (output port 2 = x2)
    add_line(mdl, [tag '_Demux/2'], [tag '_K2/1']);

    % K1, K2 → SumK
    add_line(mdl, [tag '_K1/1'], [tag '_SumK/1']);
    add_line(mdl, [tag '_K2/1'], [tag '_SumK/2']);

    % SumK → Error (-)  (反馈 u = Kx 减回去)
    add_line(mdl, [tag '_SumK/1'], [tag '_Error/2']);

    % Plant → ToWS (y 取自 x1 = Demux 端口 1；u 取自 SumK)
    add_line(mdl, [tag '_Demux/1'], [tag '_y/1']);
    add_line(mdl, [tag '_SumK/1'], [tag '_u/1']);
end

Simulink.BlockDiagram.arrangeSystem(mdl);

fprintf('  [OK] Simulink 模型搭建完成\n');

%% ===== 第 5 步：运行仿真 =====

fprintf('\n=== 运行仿真 ===\n');
set_param(mdl, 'StopTime', '8');
simOut = sim(mdl);

%% ===== 第 6 步：绘图分析 =====

figure('Name', 't10: 状态反馈控制 — 极点配置 vs LQR', ...
    'Position', [50, 50, 1100, 800]);
t = simOut.tout;

% --- 子图 1：位移响应对比 ---
subplot(3, 2, [1 2]);
colors = {'k', 'b', 'r', [0 0.6 0]};
labels = {'开环 (无控制)', ...
    sprintf('极点配置 (ts≈%.0fs)', ts_cl), ...
    'LQR-精准 (Q大R小)', ...
    'LQR-节能 (Q小R大)'};
lineStyles = {'--', '-', '-', '--'};

legHandles = [];
legNames = {};
for iRow = 1:4
    varName = sprintf('y_%s', rowTags{iRow});
    ydata = getSimData(simOut, varName, t);
    h = plot(t, ydata, 'Color', colors{iRow}, ...
        'LineStyle', lineStyles{iRow}, 'LineWidth', 2);
    hold on;
    legHandles = [legHandles, h];
    legNames = [legNames, labels{iRow}];
end
% 参考线 r=1
plot(t, ones(size(t)), ':', 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
legHandles = [legHandles, h];
legNames{end+1} = '参考输入 r=1';

hold off;
legend(legHandles, legNames, 'Location', 'southeast');
title('位移响应对比 — 状态反馈如何改变系统动态');
xlabel('时间 (s)'); ylabel('位移 y (m)'); grid on;

% --- 子图 2：控制量 u(t) 对比 ---
subplot(3, 2, [3 4]);
legHandles2 = [];
legNames2 = {};
for iRow = 1:4
    varName = sprintf('u_%s', rowTags{iRow});
    udata = getSimData(simOut, varName, t);
    h = plot(t, udata, 'Color', colors{iRow}, ...
        'LineStyle', lineStyles{iRow}, 'LineWidth', 2);
    hold on;
    legHandles2 = [legHandles2, h];
    legNames2 = [legNames2, [labels{iRow} ' 控制力 u']];
end
hold off;
legend(legHandles2, legNames2, 'Location', 'southeast');
title('控制输入 u(t) — "用力有多猛？"');
xlabel('时间 (s)'); ylabel('控制力 u (N)'); grid on;

% --- 子图 3：开环 vs 闭环极点分布 ---
subplot(3, 2, 5);
% 绘制 s 平面
plot(real(openLoopPoles), imag(openLoopPoles), 'kx', ...
    'MarkerSize', 12, 'LineWidth', 2); hold on;
plot(real(p_desired), imag(p_desired), 'b*', ...
    'MarkerSize', 12, 'LineWidth', 2);
plot(real(e1), imag(e1), 'rs', 'MarkerSize', 8, 'LineWidth', 2);
plot(real(e3), imag(e3), 'gd', 'MarkerSize', 8, 'LineWidth', 2);

% 画等 ζ 线和等 ωn 线
theta = linspace(pi/2, pi, 100);
for zz = [0.125, 0.707]
    r = 6;
    x_zeta = -r*zz*cos(linspace(-pi/2, pi/2, 50));
    y_zeta = r*sqrt(1-zz^2)*cos(linspace(-pi/2, pi/2, 50));
    plot(x_zeta, y_zeta, ':', 'Color', [0.7 0.7 0.7]);
end
xline(0, 'k'); yline(0, 'k');
axis([-6, 1, -5, 5]);
hold off;
legend('开环极点 (ζ=0.125)', ...
    '极点配置目标 (ζ=0.707)', ...
    'LQR-1 闭环极点', ...
    'LQR-3 闭环极点', ...
    'Location', 'northeast');
title('s 平面极点分布 — 越往左越快，越靠近实轴越不振荡');
xlabel('实部 Re(s)'); ylabel('虚部 Im(s)'); grid on;
axis equal;

% --- 子图 4：K 增益对比柱状图 ---
subplot(3, 2, 6);
K_all = [0, 0; K_place; K_lqr1; K_lqr3];
b = bar(K_all);
b(1).FaceColor = [0.3 0.6 1.0];  % K1: 位置反馈
b(2).FaceColor = [1.0 0.4 0.4];  % K2: 速度反馈
set(gca, 'XTickLabel', labels);
legend('K₁ (位置反馈)', 'K₂ (速度反馈)', 'Location', 'northwest');
title('反馈增益 K 对比 — K 越大 = 控制越"硬"');
ylabel('增益值'); grid on;

sgtitle('教程 10：状态反馈控制 — 从开环到 LQR 最优控制');

%% ===== 第 7 步：理论总结 =====

fprintf('\n========================================\n');
fprintf('  教程 10 完成！\n');
fprintf('========================================\n\n');

fprintf('【理论总结 — 三种设计方法的递进关系】\n\n');

fprintf('  1. 开环 (u = r)\n');
fprintf('     极点：s = %.2f ± j%.2f\n', real(openLoopPoles(1)), imag(openLoopPoles(1)));
fprintf('     ζ = %.3f → 剧烈振荡，5s+ 才稳定\n\n', zeta_open);

fprintf('  2. 极点配置 (u = r - Kx)\n');
fprintf('     K = [%.1f, %.1f]\n', K_place(1), K_place(2));
fprintf('     "我想让系统以 ζ=0.707 阻尼、ωn=4 频率运动"\n');
fprintf('     → 直接指定极点 → 反算 K\n');
fprintf('     稳定时间：~%.1fs\n\n', ts_cl);

fprintf('  3. LQR 最优控制 (u = r - Kx)\n');
fprintf('     "平衡性能 vs 能耗，让数学自动找最优 K"\n');
fprintf('     K_lqr1 = [%.1f, %.1f]  (Q大 → 追求性能)\n', K_lqr1(1), K_lqr1(2));
fprintf('     K_lqr3 = [%.1f, %.1f]  (R大 → 节省能量)\n\n', K_lqr3(1), K_lqr3(2));

fprintf('  Q/R 调节口诀：\n');
fprintf('    Q↑ → 性能好，但控制量↑（费电/费劲）\n');
fprintf('    R↑ → 省能量，但响应慢\n');
fprintf('    Q/R 比例决定性能 vs 节能的平衡点\n\n');

fprintf('  极点配置 vs LQR 的选择：\n');
fprintf('    - 你能清楚说出"我要 2s 稳定、无超调" → 极点配置\n');
fprintf('    - 你说"位置要准，但别太耗电" → LQR\n');
fprintf('    - MIMO 系统（>2个状态）→ 几乎都用 LQR\n');
fprintf('    - 实践中 LQR 更常用，Q/R 可通过经验积累\n\n');

fprintf('【动手实验】\n\n');

fprintf('  1. 修改目标极点，观察 K 的变化\n');
fprintf('     >> p_fast = [-6, -8];  K_fast = place(A,B,p_fast)\n');
fprintf('     K 会变得很大！因为要让系统更快，需要更大的控制力\n\n');

fprintf('  2. 调节 Q 矩阵，体验"性能 vs 节能"\n');
fprintf('     >> Q = diag([1000, 1]);  R = 1;\n');
fprintf('     >> K = lqr(A,B,Q,R)\n');
fprintf('     K1 会非常大 → 位置非常准，但控制力也很大\n\n');

fprintf('  3. 不稳定系统的控制（验证能控性）\n');
fprintf('     >> A_unstable = [0 1; 4 0.5];  %% 弹簧负刚度\n');
fprintf('     >> eig(A_unstable)  %% 有一个极点在右半平面！\n');
fprintf('     >> Co = ctrb(A_unstable, B);\n');
fprintf('     >> rank(Co)  %% rank=2 → 能控 → 可以镇定\n');
fprintf('     >> K = place(A_unstable, B, [-2, -3])  %% 镇定成功\n\n');

fprintf('  4. 在你的 Simulink 模型中\n');
fprintf('     双击 PolePlace 行的 K1/K2 增益模块\n');
fprintf('     → 改小试试（如 K1=5, K2=1）：响应变慢\n');
fprintf('     → 改大试试（如 K1=20, K2=5）：响应变快，但控制力剧烈\n\n');

fprintf('  下一课预告：t11 — 状态观测器 (Luenberger Observer)\n');
fprintf('  现实中不一定能测量所有状态！\n');
fprintf('  比如你能测位移，但测不了速度 → 用观测器"估计"速度\n');
fprintf('  Kalman 滤波就是观测器的随机版本！\n');

% ============================================================
% 辅助函数
% ============================================================
function s = signStr(x)
    if x >= 0, s = '+'; else, s = '-'; end
end
