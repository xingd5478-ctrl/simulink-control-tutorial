%% ============================================================
% 教程 11：状态观测器 (Luenberger Observer)
%
% 【问题引出】
%   t10 的状态反馈 u = -Kx 假设我们知道所有状态 [x1; x2]
%   现实中：
%     ✓ 位移 x₁：好测，一个编码器/电位计即可
%     ✗ 速度 x₂：需要转速传感器，增加成本、体积、故障点
%
%   怎么办？用能测到的量（位移 y）去估计测不到的量（速度 x₂）
%   → 这就是 "状态观测器"
%
% ┌─────────────────────────────────────────────────────────┐
% │ 一、Luenberger 观测器的结构                              │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │  核心思想：                                              │
% │    1. 用 A,B 建一个"虚拟模型"，跑同样的输入 u            │
% │    2. 比较模型的输出 ŷ 和真实输出 y → 得到误差           │
% │    3. 用误差 × 增益 L 来"修正"模型的估计                 │
% │                                                         │
% │  观测器方程：                                            │
% │    dẋ̂/dt = A x̂ + B u  +  L (y - ŷ)                     │
% │    ŷ     = C x̂                                          │
% │                                                         │
% │     ┌── 模型预测 ──┐  ┌── 修正项 ──┐                    │
% │                                                         │
% │  真实系统：  ẋ = Ax + Bu,    y = Cx                      │
% │  观测器：    ẋ̂ = Ax̂ + Bu + L(y - ŷ),   ŷ = Cx̂           │
% └─────────────────────────────────────────────────────────┘
%
% ┌─────────────────────────────────────────────────────────┐
% │ 二、误差动态 — 观测器好不好的数学判断                     │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │  定义估计误差： e = x - x̂                               │
% │                                                         │
% │  ė = ẋ - ẋ̂                                              │
% │    = (Ax+Bu) - (Ax̂+Bu + L(Cx-Cx̂))                      │
% │    = A(x-x̂) - LC(x-x̂)                                   │
% │    = (A - LC) e                                         │
% │                                                         │
% │  关键：误差以 ė = (A-LC)e 动态衰减                       │
% │  只要 (A-LC) 的特征值全在左半平面，误差就会 → 0！        │
% │                                                         │
% │  这与状态反馈的 (A-BK) 形成"对偶"：                      │
% │    反馈：place(A, B, p)         → K                     │
% │    观测器：place(A', C', p)'    → L                     │
% │           (转置后问题变成和反馈一样的形式)                │
% └─────────────────────────────────────────────────────────┘
%
% ┌─────────────────────────────────────────────────────────┐
% │ 三、分离原理 (Separation Principle)                      │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │  实际控制器 = 状态反馈 K + 观测器 L 一起用：             │
% │                                                         │
% │  u = -K x̂                                               │
% │  (用估计状态代替真实状态做反馈！)                         │
% │                                                         │
% │  分离原理说：K 和 L 可以独立设计，互不影响！              │
% │  - K 决定控制性能（t10）                                 │
% │  - L 决定估计收敛速度（t11）                             │
% │  - 完整闭环的极点 = eig(A-BK) ∪ eig(A-LC)               │
% └─────────────────────────────────────────────────────────┘
%
% 【本课目标】
%   1. 理解为什么需要观测器
%   2. 手搭 Luenberger 观测器，对照误差动态方程
%   3. 对比真实状态 vs 估计状态，观察收敛过程
%   4. 理解分离原理
% ============================================================

clear; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

%% ===== 系统参数（沿用 t09/t10）=====

m = 1.0;  c = 0.5;  k = 4.0;

A = [   0,     1  ;
      -k/m,  -c/m ];

B = [  0  ;
      1/m ];

C = [ 1, 0 ];    % 只测量位移（模拟现实约束）
D = 0;

fprintf('============================================\n');
fprintf('  教程 11：状态观测器 (Luenberger Observer)\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：分析可观测性 =====

% 可观测性矩阵：Ob = [C; CA; CA²; ...]
Ob = obsv(A, C);
rank_Ob = rank(Ob);

fprintf('【可观测性分析】\n');
fprintf('  可观测性矩阵 Ob = [C; CA]：\n');
fprintf('    rank(Ob) = %d  (n=%d)\n\n', rank_Ob, length(A));

if rank_Ob == length(A)
    fprintf('  ✓ 系统完全可观测！可以从输出 y 重建全部状态。\n\n');
else
    fprintf('  ✗ 系统不可观测！无法从 y 重建状态，需要增加传感器。\n\n');
    return;
end

fprintf('  物理含义：只测位移 y，能否推断出速度 ẏ？\n');
fprintf('    可以！因为位移是速度的积分，位移的变化率就是速度。\n');
fprintf('    这就是可观测性的物理本质。\n\n');

%% ===== 第 2 步：设计观测器增益 L =====

% 观测器极点：应该比控制器极点"更快"
% 经验法则：观测器极点 ≈ 2~5 倍控制器极点实部
% 这样估计误差在控制起作用之前就已经收敛了

% 设观测器极点：-6, -8（比 t10 的极点配置 -2.83±j2.83 快 2-3 倍）
p_obs = [-6, -8];

% 对偶法求 L：L = place(A', C', p_obs)'
L = place(A', C', p_obs)';

fprintf('【观测器设计】\n');
fprintf('  目标观测器极点：s₁ = %.1f, s₂ = %.1f\n', p_obs(1), p_obs(2));
fprintf('  观测器增益 L = [ %.4f ]\n', L(1));
fprintf('                  [ %.4f ]\n\n', L(2));

fprintf('  L 的物理含义：\n');
fprintf('    L₁ = %.1f：位移误差 (y-ŷ) 对 x̂₁ 估计的修正力度\n', L(1));
fprintf('    L₂ = %.1f：位移误差 (y-ŷ) 对 x̂₂ 估计的修正力度\n\n', L(2));

% 验证观测器误差动态
A_obs_err = A - L*C;
obsErrPoles = eig(A_obs_err);
fprintf('  验证 (A-LC) 特征值（应等于目标）：\n');
fprintf('    s₁ = %.1f, s₂ = %.1f\n\n', obsErrPoles(1), obsErrPoles(2));

fprintf('  误差收敛时间 ts ≈ %.2f s (2%% 准则)\n', 4/min(abs(real(obsErrPoles))));

%% ===== 第 3 步：同时设计控制器 K（分离原理）=====

% 控制器极点（和 t10 一样）
zeta_d = 0.707;  wn_d = 4.0;
p_ctrl = roots([1, 2*zeta_d*wn_d, wn_d^2]);
K = place(A, B, p_ctrl);

fprintf('\n【分离原理：同时设计 K 和 L】\n');
fprintf('  控制器 K = [ %.2f,  %.2f ]  (极点: %.1f±j%.1f)\n', ...
    K(1), K(2), abs(real(p_ctrl(1))), abs(imag(p_ctrl(1))));
fprintf('  观测器 L = [ %.2f,  %.2f ]  (极点: %.1f, %.1f)\n', ...
    L(1), L(2), p_obs(1), p_obs(2));
fprintf('  完整闭环极点 = eig(A-BK) ∪ eig(A-LC)\n');
fprintf('  两者可以独立设计，互不干扰！\n\n');

%% ===== 第 4 步：搭建 Simulink 模型 =====

mdl = 'tutorial11_observer';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), save_system(mdl, fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx'])); close_system(mdl, 0); end
new_system(mdl, 'Model');
open_system(mdl);

% ┌──────────────────────── 模型结构 ─────────────────────────┐
% │                                                           │
% │  第一行：真实系统（Plant）— 状态 [x1; x2] 的真值           │
% │  第二行：Luenberger 观测器 — 估计状态 [x̂1; x̂2]             │
% │                                                           │
% │  布局：                                                   │
% │                                                           │
% │  r ─→ (+) ─→ [Plant SS] ─→ x_true ─→ Scope (对比)        │
% │         ↑         │                                      │
% │         └─ K ── x̂  ← 用估计状态做反馈！                   │
% │                    │                                      │
% │  x_true ──── y ─── (+) ─→ L ─→ (+) ─→ [∫] ─→ x̂          │
% │                     ↑                      │             │
% │                     └──── Cx̂ ←─────────────┘             │
% │                            ↑                              │
% │                            u                              │
% └───────────────────────────────────────────────────────────┘

% --- 信号源 ---
add_block('simulink/Sources/Step', [mdl '/Setpoint r'], ...
    'Position', [50, 100, 130, 140]);
set_param([mdl '/Setpoint r'], 'Time', '0.5', 'Before', '0', 'After', '1');

% ===== 行 1：真实 Plant =====
% Plant 输出完整状态 [x1; x2] (C=eye(2) 供反馈用)
add_block('simulink/Continuous/State-Space', [mdl '/Plant_real'], ...
    'Position', [240, 80, 310, 130]);
set_param([mdl '/Plant_real'], ...
    'A', mat2str(A), 'B', mat2str(B), ...
    'C', 'eye(2)', 'D', 'zeros(2,1)', ...
    'X0', '[0.3; 0]');   % ★ 初态位移 0.3m — 观测器不知道！

% 误差加法器 r - K*x̂
add_block('simulink/Math Operations/Add', [mdl '/Error_r_minus_Kx'], ...
    'Position', [140, 85, 170, 115]);
set_param([mdl '/Error_r_minus_Kx'], 'Inputs', '|+-');

% Demux 拆分真实状态（用于对比显示）
add_block('simulink/Signal Routing/Demux', [mdl '/Demux_real'], ...
    'Position', [360, 75, 380, 135]);
set_param([mdl '/Demux_real'], 'Outputs', '2');

% ===== 行 2：观测器 =====
% 观测器方程：ẋ̂ = (A-LC)x̂ + Bu + Ly = (A-LC)x̂ + [B, L]·[u; y]
% 输入：[u; y]（2×1） 输出：x̂ = [x̂₁; x̂₂]（2×1）

A_obs = A - L*C;         % 观测器系统矩阵
B_obs = [B, L];          % 输入矩阵：[B, L] 使 [u; y] → Bu + Ly
C_obs = eye(2);          % 输出完整估计状态
D_obs = zeros(2, 2);

% 用 Mux 将 u 和 y 合并成一个 2×1 输入向量
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_u_y'], ...
    'Position', [240, 200, 260, 260]);
set_param([mdl '/Mux_u_y'], 'Inputs', '2', 'DisplayOption', 'bar');

% 观测器 State-Space 模块
add_block('simulink/Continuous/State-Space', [mdl '/Observer'], ...
    'Position', [300, 200, 370, 260]);
set_param([mdl '/Observer'], ...
    'A', mat2str(A_obs), ...
    'B', mat2str(B_obs), ...
    'C', mat2str(C_obs), ...
    'D', mat2str(D_obs), ...
    'X0', '[0; 0]');   % ★ 观测器从 0 开始，不知道真实位移是 0.3

fprintf('  [OK] 观测器搭建完成（State-Space 模块实现 ė = (A-LC)x̂ + Bu + Ly）\n');

% ===== 顶层连线 =====

% 真实 Plant 需要的输入：u = r - K*x̂
% 先从观测器取 x̂ → Demux → K

% Demux for estimated state
add_block('simulink/Signal Routing/Demux', [mdl '/Demux_xhat'], ...
    'Position', [500, 200, 520, 260]);
set_param([mdl '/Demux_xhat'], 'Outputs', '2');

% Gains for K feedback
add_block('simulink/Math Operations/Gain', [mdl '/K1'], ...
    'Position', [550, 200, 590, 225]);
set_param([mdl '/K1'], 'Gain', num2str(K(1)));

add_block('simulink/Math Operations/Gain', [mdl '/K2'], ...
    'Position', [550, 240, 590, 265]);
set_param([mdl '/K2'], 'Gain', num2str(K(2)));

% Sum K1*x̂₁ + K2*x̂₂
add_block('simulink/Math Operations/Add', [mdl '/Sum_Kx'], ...
    'Position', [630, 215, 660, 255]);
set_param([mdl '/Sum_Kx'], 'Inputs', '|++', 'IconShape', 'round');

% --- 顶层连线 ---

% r → Error
add_line(mdl, 'Setpoint r/1', 'Error_r_minus_Kx/1');

% Error → Plant
add_line(mdl, 'Error_r_minus_Kx/1', 'Plant_real/1');

% Plant → Demux_real
add_line(mdl, 'Plant_real/1', 'Demux_real/1');

% Plant u also → Mux_u_y (port 1)
add_line(mdl, 'Error_r_minus_Kx/1', 'Mux_u_y/1');

% Plant output y (x1 from Demux_real port 1) → Mux_u_y (port 2)
add_line(mdl, 'Demux_real/1', 'Mux_u_y/2');

% Mux → Observer
add_line(mdl, 'Mux_u_y/1', 'Observer/1');

% Observer output → Demux_xhat
add_line(mdl, 'Observer/1', 'Demux_xhat/1');

% Demux_xhat → K1, K2
add_line(mdl, 'Demux_xhat/1', 'K1/1');
add_line(mdl, 'Demux_xhat/2', 'K2/1');

% K1, K2 → Sum_Kx
add_line(mdl, 'K1/1', 'Sum_Kx/1');
add_line(mdl, 'K2/1', 'Sum_Kx/2');

% Sum_Kx → Error (-)
add_line(mdl, 'Sum_Kx/1', 'Error_r_minus_Kx/2');

% --- Scope 对比真实 vs 估计 ---
add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [800, 60, 860, 130]);
set_param([mdl '/Scope'], 'NumInputPorts', '4');

% x₁ truth vs estimate
add_line(mdl, 'Demux_real/1', 'Scope/1');   % x₁ true
add_line(mdl, 'Demux_xhat/1', 'Scope/2');   % x̂₁ estimate

% x₂ truth vs estimate
add_line(mdl, 'Demux_real/2', 'Scope/3');   % x₂ true
add_line(mdl, 'Demux_xhat/2', 'Scope/4');   % x̂₂ estimate

% --- To Workspace ---
add_block('simulink/Sinks/To Workspace', [mdl '/ws_x1_true'], ...
    'Position', [800, 200, 860, 230]);
set_param([mdl '/ws_x1_true'], 'VariableName', 'x1_true');

add_block('simulink/Sinks/To Workspace', [mdl '/ws_x1_hat'], ...
    'Position', [800, 250, 860, 280]);
set_param([mdl '/ws_x1_hat'], 'VariableName', 'x1_hat');

add_block('simulink/Sinks/To Workspace', [mdl '/ws_x2_true'], ...
    'Position', [800, 300, 860, 330]);
set_param([mdl '/ws_x2_true'], 'VariableName', 'x2_true');

add_block('simulink/Sinks/To Workspace', [mdl '/ws_x2_hat'], ...
    'Position', [800, 350, 860, 380]);
set_param([mdl '/ws_x2_hat'], 'VariableName', 'x2_hat');

add_line(mdl, 'Demux_real/1', 'ws_x1_true/1');
add_line(mdl, 'Demux_xhat/1', 'ws_x1_hat/1');
add_line(mdl, 'Demux_real/2', 'ws_x2_true/1');
add_line(mdl, 'Demux_xhat/2', 'ws_x2_hat/1');

Simulink.BlockDiagram.arrangeSystem(mdl);

fprintf('  [OK] 顶层连线完成\n');

%% ===== 第 5 步：运行仿真 =====

fprintf('\n=== 运行仿真 ===\n');
set_param(mdl, 'StopTime', '5');
simOut = sim(mdl);

%% ===== 第 6 步：绘图分析 =====

figure('Name', 't11: Luenberger 状态观测器', 'Position', [50, 50, 1000, 700]);
t = simOut.tout;

% --- 子图 1：位移 x₁ 真实 vs 估计 ---
subplot(3, 1, 1);
x1t = getSimData(simOut, 'x1_true', t);
x1h = getSimData(simOut, 'x1_hat', t);
plot(t, x1t, 'b', 'LineWidth', 2); hold on;
plot(t, x1h, 'r--', 'LineWidth', 2); hold off;
legend('真实 x₁ (Plant 初态=0.3)', '估计 x̂₁ (观测器初态=0)', ...
    'Location', 'southeast');
title('位移估计 — 观测器从 0 开始，快速追上真实值');
xlabel('时间 (s)'); ylabel('位移 (m)'); grid on;

% --- 子图 2：速度 x₂ 真实 vs 估计 ---
subplot(3, 1, 2);
x2t = getSimData(simOut, 'x2_true', t);
x2h = getSimData(simOut, 'x2_hat', t);
plot(t, x2t, 'b', 'LineWidth', 2); hold on;
plot(t, x2h, 'r--', 'LineWidth', 2); hold off;
legend('真实 x₂ (不可直接测量)', '估计 x̂₂ (从位移推断!)', ...
    'Location', 'southeast');
title('速度估计 — 没有速度传感器，但从位移推断出了速度！');
xlabel('时间 (s)'); ylabel('速度 (m/s)'); grid on;

% --- 子图 3：估计误差 e₁ = x₁ - x̂₁, e₂ = x₂ - x̂₂ ---
subplot(3, 1, 3);
e1 = x1t - x1h;
e2 = x2t - x2h;
plot(t, e1, 'b', 'LineWidth', 1.5); hold on;
plot(t, e2, 'r', 'LineWidth', 1.5); hold off;
legend('e₁ = x₁ - x̂₁', 'e₂ = x₂ - x̂₂', 'Location', 'northeast');
title('估计误差 — 按 ė = (A-LC)e 指数衰减到 0');
xlabel('时间 (s)'); ylabel('估计误差'); grid on;

% 打印误差收敛情况
fprintf('  最终估计误差: e₁(5s) = %.4f, e₂(5s) = %.4f\n', e1(end), e2(end));

sgtitle('教程 11：Luenberger 状态观测器 — 从可测输出重建不可测状态');

%% ===== 第 7 步：理论总结 =====

fprintf('\n========================================\n');
fprintf('  教程 11 完成！\n');
fprintf('========================================\n\n');

fprintf('【理论总结】\n\n');

fprintf('  1. 观测器 = 系统模型 + 修正项\n');
fprintf('     ẋ̂ = Ax̂ + Bu + L(y - ŷ)\n');
fprintf('     "模型预测" + "用测量误差修正"\n\n');

fprintf('  2. 误差动态：ė = (A - LC)e\n');
fprintf('     只要 eig(A-LC) < 0，误差就 → 0\n');
fprintf('     你的观测器极点 = [%.0f, %.0f] → 误差约 %.2fs 收敛\n\n', ...
    p_obs(1), p_obs(2), 4/min(abs(real(obsErrPoles))));

fprintf('  3. 对偶性 (Duality)\n');
fprintf('     状态反馈:  K = place(A, B, p)\n');
fprintf('     观测器:    L = place(A'', C'', p)''\n');
fprintf('     (A-BK) 和 (A-LC) 的极点设计是"转置对偶"的\n\n');

fprintf('  4. 分离原理\n');
fprintf('     K 和 L 可以完全独立设计\n');
fprintf('     完整系统的特征值 = eig(A-BK) ∪ eig(A-LC)\n');
fprintf('     这是现代控制理论最优美的结论之一\n\n');

fprintf('  5. 观测器增益 L 的影响\n');
fprintf('     L 大 → 估计收敛快 → 但对测量噪声敏感\n');
fprintf('     L 小 → 估计收敛慢 → 但平滑/抗噪\n');
fprintf('     这和 Kalman 滤波的思想一脉相承！（t12 预告）\n\n');

fprintf('【动手实验】\n\n');
fprintf('  1. 改变初始条件差距，看收敛过程\n');
fprintf('     真实 Plant 初态: [0.3; 0]\n');
fprintf('     观测器初态:     [0; 0]\n');
fprintf('     → 差距越大，收敛过程越明显\n\n');

fprintf('  2. 改观测器极点，看收敛速度\n');
fprintf('     快极点 [-20, -25]：误差瞬间消失，但对噪声敏感\n');
fprintf('     慢极点 [-2, -3]：收敛慢，但平滑\n\n');

fprintf('  3. 双击 Observer 子系统\n');
fprintf('     → 对照 ė = (A-LC)x̂ + Bu + Ly 方程\n');
fprintf('     → 找到 L1, L2 增益和反馈路径\n\n');

fprintf('  下一课预告：t12 — Kalman 滤波\n');
fprintf('  Luenberger 观测器假设无噪声 → L 靠极点配置\n');
fprintf('  Kalman 滤波考虑过程噪声+测量噪声 → L 靠统计优化\n');
fprintf('  但本质上，Kalman = 带噪声模型的 Luenberger 观测器！\n');

% ============================================================
% 辅助函数
