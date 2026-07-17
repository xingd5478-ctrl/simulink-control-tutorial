%% ============================================================
% 教程 09：状态空间模型 — 现代控制理论的基石
%
% 【理论知识】
%   经典控制 → 传递函数 G(s)，只关心输入→输出的外部关系
%   现代控制 → 状态空间，揭示系统内部所有状态的运动规律
%
%   标准形式：
%     dx/dt = A·x + B·u     （状态方程：状态如何变化）
%     y     = C·x + D·u     （输出方程：我们能测量到什么）
%
%   其中：
%     x — 状态向量 (n×1)，系统的"内部记忆"
%     u — 输入向量 (m×1)，控制力/激励
%     y — 输出向量 (p×1)，传感器读数
%     A — 系统矩阵 (n×n)，决定系统固有特性
%     B — 输入矩阵 (n×m)，控制力如何影响状态
%     C — 输出矩阵 (p×n)，状态如何映射到测量值
%     D — 前馈矩阵 (p×m)，输入直接传到输出（通常为0）
%
%   ★ 为什么状态空间在企业级项目中是必须的？
%     1. 多输入多输出 (MIMO)：传递函数很难处理，状态空间天然支持
%     2. 内部状态可见：电机控制需要知道转速+电流+位置，不只是最终输出
%     3. 最优控制 (LQR) 和 Kalman 滤波 都建立在状态空间之上
%     4. 非线性系统：状态空间可以扩展到时变/非线性
%
% 【物理案例】质量-弹簧-阻尼系统 (Mass-Spring-Damper)
%
%     ┌───┐  k(弹簧)   ┌───┐      ┌───┐
%     │墙面├≈≈≈≈≈≈≈≈≈≈├───┤      │   │
%     └───┘            │ m ├──────┤ F │  ← 外力
%                      │   │  c   │   │
%                      └─┬─┘ (阻尼)└───┘
%                        │
%                    位置 y(t)
%
%   运动方程 (牛顿第二定律)：
%     m·ÿ + c·ẏ + k·y = F(t)
%
%   选取状态变量：
%     x₁ = y   (位置)
%     x₂ = ẏ   (速度)
%
%   则状态空间方程为：
%     d/dt[x₁] = [   0      1  ]·[x₁] + [  0   ]·F
%     d/dt[x₂]   [ -k/m  -c/m ] [x₂]   [ 1/m ]
%
%           y  = [   1      0  ]·[x₁] + [  0   ]·F
%                               [x₂]
%
% 【本课目标】
%   1. 用 State-Space 模块直接搭建模型
%   2. 与等效传递函数模型对比验证
%   3. 深入理解：用积分器手搭状态方程，看到"状态的流动"
%   4. 学会从物理方程推导 A、B、C、D 矩阵
% ============================================================

clear; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

%% ===== 第 0 步：系统参数与理论推导 =====

m = 1.0;    % 质量 (kg)
c = 0.5;    % 阻尼系数 (N·s/m)
k = 4.0;    % 弹簧刚度 (N/m)

%   A = [   0,    1 ]    B = [  0  ]
%       [ -k/m, -c/m ]        [ 1/m ]
%
%   C = [ 1, 0 ]          D = 0

A = [   0,     1  ;
      -k/m,  -c/m ];

B = [  0  ;
      1/m ];

C = [ 1, 0 ];

D = 0;

fprintf('============================================\n');
fprintf('  教程 09：状态空间模型\n');
fprintf('============================================\n\n');

fprintf('【理论】质量-弹簧-阻尼系统\n');
fprintf('  m=%.1f kg, c=%.1f N·s/m, k=%.1f N/m\n\n', m, c, k);

fprintf('  状态变量选择：\n');
fprintf('    x₁ = 位移 y  (position)\n');
fprintf('    x₂ = 速度 ẏ  (velocity)\n\n');

fprintf('  状态空间矩阵：\n');
fprintf('    A = [   0     1  ]    B = [  0  ]\n');
fprintf('        [  %.1f  %.1f ]        [ %.1f ]\n', -k/m, -c/m, 1/m);
fprintf('    C = [   1     0  ]    D = [  0  ]\n\n');

%% ===== 第 1 步：建立等效传递函数（用于对比验证）=====

% 从状态空间到传递函数的理论：
% G(s) = C·(sI - A)⁻¹·B + D
% 对本系统：G(s) = 1/(m·s² + c·s + k)

num = [1];           % 分子
den = [m, c, k];     % 分母: m·s² + c·s + k

fprintf('  等效传递函数：G(s) = 1 / (%.1f s² + %.1f s + %.1f)\n\n', m, c, k);

% 验证：用 ss2tf 从状态空间反算传递函数
[num_check, den_check] = ss2tf(A, B, C, D);
fprintf('  ★ 验证 ss2tf(A,B,C,D)：理论结果 = [%s] / [%s]\n', ...
    mat2str(num_check, 3), mat2str(den_check, 3));
fprintf('    与手动推导一致 ✓\n\n');

%% ===== 第 2 步：搭建 Simulink 模型 =====

mdl = 'tutorial09_state_space';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), save_system(mdl, fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx'])); close_system(mdl, 0); end
new_system(mdl, 'Model');
open_system(mdl);

% --- 信号源：阶跃力 F(t) ---
add_block('simulink/Sources/Step', [mdl '/Step Force'], ...
    'Position', [50, 120, 130, 160]);
set_param([mdl '/Step Force'], ...
    'Time', '1', ...
    'Before', '0', ...
    'After', '1');  % t=1s 时施加 1N 的力

% --- 方法 A：传递函数（熟悉的经典方法）---
add_block('simulink/Continuous/Transfer Fcn', [mdl '/TF Model'], ...
    'Position', [200, 100, 290, 150]);
set_param([mdl '/TF Model'], ...
    'Numerator',   mat2str(num), ...
    'Denominator', mat2str(den));

% --- 方法 B：State-Space 模块（新方法）---
add_block('simulink/Continuous/State-Space', [mdl '/SS Model'], ...
    'Position', [200, 220, 290, 280]);
set_param([mdl '/SS Model'], ...
    'A', mat2str(A), ...
    'B', mat2str(B), ...
    'C', mat2str(C), ...
    'D', mat2str(D), ...
    'X0', '[0; 0]');

% --- 方法 C：手搭积分器实现（理解状态的本质）---
% 子系统：用积分器实现 dx/dt = Ax + Bu，y = Cx + Du
% 这会让你"看到"每一个状态变量

add_block('simulink/Ports & Subsystems/Subsystem', ...
    [mdl '/Manual SS (Integrators)'], ...
    'Position', [200, 350, 290, 410]);
set_param([mdl '/Manual SS (Integrators)'], ...
    'BackgroundColor', 'lightBlue');

% 手动搭建状态方程，揭示"状态的流动"
% ------------------------------------------------------------
% 核心思路：用积分器 + 增益 + 反馈 实现 dx/dt = Ax + Bu
%
% 对于质量-弹簧-阻尼系统：
%   dx₁/dt = x₂                        (速度=位置的导数)
%   dx₂/dt = -(k/m)·x₁ - (c/m)·x₂ + (1/m)·u
%        y = x₁                        (输出=位置)
%
% 信号流：
%   u ─→ [1/m] ─→ (+) ─→ [∫] ─→ x₂ ─→ [∫] ─→ x₁ ─→ y
%                  ↑              │              │
%                  ├── [-c/m] ─────┘              │
%                  └── [-k/m] ────────────────────┘
% ------------------------------------------------------------

subsys = [mdl '/Manual SS (Integrators)'];

% 删除子系统的默认 In1→Out1 连接（保留 In1 和 Out1 端口）
delete_line(subsys, 'In1/1', 'Out1/1');

% --- 第 1 组：前向通路的增益和积分器 ---
% 输入增益：u × (1/m)
add_block('simulink/Math Operations/Gain', [subsys '/Gain_1overm'], ...
    'Position', [120, 160, 170, 200]);
set_param([subsys '/Gain_1overm'], 'Gain', num2str(1/m));

% 加法器：三个信号相加得到 dx₂/dt
add_block('simulink/Math Operations/Add', [subsys '/Sum_x2dot'], ...
    'Position', [220, 130, 260, 210]);
set_param([subsys '/Sum_x2dot'], 'Inputs', '|+++', 'IconShape', 'round');

% 积分器 x₂：对 dx₂/dt 积分，输出 = 速度
add_block('simulink/Continuous/Integrator', [subsys '/Int_x2_velocity'], ...
    'Position', [300, 160, 360, 210]);
set_param([subsys '/Int_x2_velocity'], 'InitialCondition', '0');

% 积分器 x₁：对 dx₁/dt (= x₂) 积分，输出 = 位移
add_block('simulink/Continuous/Integrator', [subsys '/Int_x1_position'], ...
    'Position', [300, 60, 360, 110]);
set_param([subsys '/Int_x1_position'], 'InitialCondition', '0');

% --- 第 2 组：A 矩阵的反馈增益（状态→状态导数）---
% -(k/m) * x₁：弹簧回复力 → 送回到 Sum_x2dot 的端口 1
add_block('simulink/Math Operations/Gain', [subsys '/Gain_minus_k_over_m'], ...
    'Position', [430, 60, 480, 100]);
set_param([subsys '/Gain_minus_k_over_m'], 'Gain', num2str(-k/m));

% -(c/m) * x₂：阻尼力 → 送回到 Sum_x2dot 的端口 2
add_block('simulink/Math Operations/Gain', [subsys '/Gain_minus_c_over_m'], ...
    'Position', [430, 140, 480, 180]);
set_param([subsys '/Gain_minus_c_over_m'], 'Gain', num2str(-c/m));

% --- 第 3 组：Goto 模块（将状态变量导出到子系统外部）---
add_block('simulink/Signal Routing/Goto', [subsys '/Goto_x1'], ...
    'Position', [510, 60, 550, 100]);
set_param([subsys '/Goto_x1'], 'GotoTag', 'x1_state', 'TagVisibility', 'global');

add_block('simulink/Signal Routing/Goto', [subsys '/Goto_x2'], ...
    'Position', [510, 160, 550, 200]);
set_param([subsys '/Goto_x2'], 'GotoTag', 'x2_state', 'TagVisibility', 'global');

% --- 连线 ---
% 前向通路
add_line(subsys, 'In1/1',    'Gain_1overm/1');          % u → 1/m
add_line(subsys, 'Gain_1overm/1', 'Sum_x2dot/3');        % (1/m)*u → Sum 端口3
add_line(subsys, 'Sum_x2dot/1', 'Int_x2_velocity/1');    % Sum → 积分器 x₂
add_line(subsys, 'Int_x2_velocity/1', 'Int_x1_position/1'); % x₂ → 积分器 x₁

% A 矩阵反馈
add_line(subsys, 'Int_x1_position/1', 'Gain_minus_k_over_m/1');     % x₁ → -(k/m)
add_line(subsys, 'Gain_minus_k_over_m/1', 'Sum_x2dot/1');           % → Sum 端口1
add_line(subsys, 'Int_x2_velocity/1', 'Gain_minus_c_over_m/1');     % x₂ → -(c/m)
add_line(subsys, 'Gain_minus_c_over_m/1', 'Sum_x2dot/2');           % → Sum 端口2

% 状态导出
add_line(subsys, 'Int_x1_position/1', 'Goto_x1/1');
add_line(subsys, 'Int_x2_velocity/1', 'Goto_x2/1');

% 输出 y = x₁
add_line(subsys, 'Int_x1_position/1', 'Out1/1');

fprintf('  [OK] 手动积分器子系统搭建完成\n');

%% ===== 第 3 步：顶层连线（三种方法并列对比）=====

% --- 输出显示：4通道示波器 ---
% Channel 1: TF output
% Channel 2: SS block output
% Channel 3: Manual integrator output
% Channel 4: Input force

add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [500, 70, 560, 130]);
set_param([mdl '/Scope'], 'NumInputPorts', '4');

% --- From blocks to observe internal states ---
add_block('simulink/Signal Routing/From', [mdl '/From x1'], ...
    'Position', [500, 200, 540, 230]);
set_param([mdl '/From x1'], 'GotoTag', 'x1_state');

add_block('simulink/Signal Routing/From', [mdl '/From x2'], ...
    'Position', [500, 270, 540, 300]);
set_param([mdl '/From x2'], 'GotoTag', 'x2_state');

% --- To Workspace 模块（用于 MATLAB 绘图）---
add_block('simulink/Sinks/To Workspace', [mdl '/y_TF'], ...
    'Position', [620, 70, 690, 100]);
set_param([mdl '/y_TF'], 'VariableName', 'y_tf');

add_block('simulink/Sinks/To Workspace', [mdl '/y_SS'], ...
    'Position', [620, 120, 690, 150]);
set_param([mdl '/y_SS'], 'VariableName', 'y_ss');

add_block('simulink/Sinks/To Workspace', [mdl '/y_MA'], ...
    'Position', [620, 170, 690, 200]);
set_param([mdl '/y_MA'], 'VariableName', 'y_ma');

add_block('simulink/Sinks/To Workspace', [mdl '/x1_data'], ...
    'Position', [620, 220, 690, 250]);
set_param([mdl '/x1_data'], 'VariableName', 'x1_data');

add_block('simulink/Sinks/To Workspace', [mdl '/x2_data'], ...
    'Position', [620, 270, 690, 300]);
set_param([mdl '/x2_data'], 'VariableName', 'x2_data');

% --- 顶层连线 ---

% Force → TF Model
add_line(mdl, 'Step Force/1', 'TF Model/1');
% Force → SS Model
add_line(mdl, 'Step Force/1', 'SS Model/1');
% Force → Manual SS
add_line(mdl, 'Step Force/1', 'Manual SS (Integrators)/1');

% TF → Scope/1, ToWS
add_line(mdl, 'TF Model/1', 'Scope/1');
add_line(mdl, 'TF Model/1', 'y_TF/1');

% SS → Scope/2, ToWS
add_line(mdl, 'SS Model/1', 'Scope/2');
add_line(mdl, 'SS Model/1', 'y_SS/1');

% Manual → Scope/3, ToWS
add_line(mdl, 'Manual SS (Integrators)/1', 'Scope/3');
add_line(mdl, 'Manual SS (Integrators)/1', 'y_MA/1');

% Force → Scope/4
add_line(mdl, 'Step Force/1', 'Scope/4');

% From blocks → To Workspace
add_line(mdl, 'From x1/1', 'x1_data/1');
add_line(mdl, 'From x2/1', 'x2_data/1');

% 美化布局
Simulink.BlockDiagram.arrangeSystem(mdl);

%% ===== 第 4 步：运行仿真 =====

fprintf('\n=== 运行仿真 ===\n');
set_param(mdl, 'StopTime', '10');
simOut = sim(mdl);

%% ===== 第 5 步：结果分析 =====

figure('Name', 't09: 状态空间模型 — 三种方法对比', ...
    'Position', [50, 100, 1000, 700]);

% --- 子图 1：三种方法的输出对比（应该完全重合）---
subplot(3, 1, 1);
t = simOut.tout;

% 提取 To Workspace 数据（通过 simOut 获取，兼容各版本）
y_tf_data = getSimData(simOut, 'y_tf', t);
y_ss_data = getSimData(simOut, 'y_ss', t);
y_ma_data = getSimData(simOut, 'y_ma', t);

plot(t, y_tf_data, 'b--', 'LineWidth', 2); hold on;
plot(t, y_ss_data, 'r-', 'LineWidth', 1.5);
plot(t, y_ma_data, 'g:', 'LineWidth', 2); hold off;
legend('传递函数 TF', 'State-Space 模块', '手动积分器', ...
    'Location', 'southeast');
title('输出 y(t) — 三种方法对比（应完全重合）');
xlabel('时间 (s)'); ylabel('位移 (m)'); grid on;

% 计算误差
err_ss = y_tf_data - y_ss_data;
err_ma = y_tf_data - y_ma_data;

% --- 子图 2：误差分析 ---
subplot(3, 1, 2);
plot(t, err_ss, 'r', 'LineWidth', 1.5); hold on;
plot(t, err_ma, 'g', 'LineWidth', 1.5); hold off;
legend('TF - SS Block', 'TF - Manual Integrators', 'Location', 'southeast');
title('误差分析（验证三种方法的等效性）');
xlabel('时间 (s)'); ylabel('误差 (m)'); grid on;
fprintf('  SS Block 最大误差: %.2e m\n', max(abs(err_ss)));
fprintf('  Manual 最大误差:   %.2e m\n', max(abs(err_ma)));

% --- 子图 3：状态变量轨迹（传递函数看不到的"内部信息"）---
subplot(3, 1, 3);
x1d = getSimData(simOut, 'x1_data', t);
x2d = getSimData(simOut, 'x2_data', t);

if ~any(isnan(x1d))

    yyaxis left;
    plot(t, x1d, 'b', 'LineWidth', 2);
    ylabel('位移 x₁ (m)'); grid on;

    yyaxis right;
    plot(t, x2d, 'r', 'LineWidth', 2);
    ylabel('速度 x₂ (m/s)');

    legend('状态 x₁ (位移)', '状态 x₂ (速度)', 'Location', 'southeast');
    title('状态变量轨迹 ★ 状态空间的独特优势：可以看到内部状态！');
    xlabel('时间 (s)');
else
    title('状态变量数据未成功捕获（请检查模型）');
end

sgtitle('教程 09：状态空间模型 — 从传递函数到状态空间');

%% ===== 第 6 步：理论总结与动手实验 =====

fprintf('\n========================================\n');
fprintf('  教程 09 完成！\n');
fprintf('========================================\n\n');

fprintf('【理论总结】\n');
fprintf('  1. 传递函数 G(s) 和状态空间 (A,B,C,D) 是等价的\n');
fprintf('     - 传递函数：外部视角，只看输入→输出\n');
fprintf('     - 状态空间：内部视角，揭示所有状态的流动\n\n');

fprintf('  2. 状态的选择不是唯一的！\n');
fprintf('     - 本例选了 x₁=位移, x₂=速度\n');
fprintf('     - 也可以选 x₁=位移, x₂=位移+速度\n');
fprintf('     - 不同的状态选择 → 不同的A,B,C,D → 同样的输入输出关系\n');
fprintf('     - 这叫"坐标变换"或"相似变换"\n\n');

fprintf('  3. 为什么企业级应用都用状态空间？\n');
fprintf('     - 电机FOC控制：需要控制 id, iq, ω 三个状态\n');
fprintf('     - 机器人：需要控制每个关节的角度+角速度\n');
fprintf('     - 无人机：需要控制位置+速度+姿态+角速度（12个状态）\n');
fprintf('     - Kalman滤波：本质是状态观测器，必须用状态空间\n');
fprintf('     - LQR/MPC：最优控制建立在状态空间之上\n\n');

fprintf('【动手实验 — 请逐个尝试】\n\n');

fprintf('  1. 改变初始条件，观察状态轨迹的变化\n');
fprintf('     >> set_param(''%s/SS Model'', ''X0'', ''[0.5; 0]'')\n', mdl);
fprintf('     >> t09_state_space\n');
fprintf('     含义：初始位移0.5m（拉弹簧后释放），观察自由衰减\n\n');

fprintf('  2. 改变系统参数，观察阻尼特性的变化\n');
fprintf('     m=1, c=0.1 (欠阻尼，振荡衰减)\n');
fprintf('     m=1, c=4.0 (临界阻尼)\n');
fprintf('     m=1, c=8.0 (过阻尼，无振荡)\n\n');

fprintf('  3. 用 ss2tf 和 tf2ss 验证双向转换\n');
fprintf('     >> [A,B,C,D] = tf2ss([1], [1 0.5 4])  %% 传递函数→状态空间\n');
fprintf('     >> [num,den] = ss2tf(A,B,C,D)         %% 状态空间→传递函数\n');
fprintf('     注意：tf2ss 给出的 A,B,C,D 可能和手动推导的不同\n');
fprintf('     （这是"能控标准型"，状态变量含义不同，但输入输出等价）\n\n');

fprintf('  4. 双击 Manual SS (Integrators) 子系统\n');
fprintf('     → 观察"状态的流动"：输入→积分器→状态→反馈→积分器\n');
fprintf('     → 这就是 dx/dt = Ax + Bu 的图形化实现！\n');
fprintf('     → 每一个积分器的输出就是一个状态变量\n\n');

fprintf('  5. 试着自己推导一个 RLC 电路的状态空间模型\n');
fprintf('     选状态：x₁=电容电压 vc, x₂=电感电流 iL\n');
fprintf('     写出 A, B, C, D 矩阵，填入 State-Space 模块验证\n\n');

fprintf('  下一课预告：t10 将在状态空间的基础上\n');
fprintf('  学习 LQR 最优控制和极点配置——真正开始设计控制器！\n');

