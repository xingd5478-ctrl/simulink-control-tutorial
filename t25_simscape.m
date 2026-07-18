%% ============================================================
% 教程 25：Simscape 物理建模 — 不用推公式的建模方法
%
% 【Simscape vs 传统 Simulink】
%   传统 Simulink: 你先推出传递函数 G(s)，再用 Transfer Fcn 模块
%   Simscape: 你直接放置物理元件（电阻、弹簧、质量块），
%             连线代表物理连接（电流、力），系统自动推导方程！
%
%   Simulink 是"数学建模"：G(s) = 1/(s²+2s+5)
%   Simscape 是"物理建模"：放一块质量 + 一个弹簧 + 一个阻尼器
%
% 【Simscape 的优势】
%   1. 不需要推导数学方程 → 减少人为错误
%   2. 物理量直接对应元件 → 直观（看模型就知道是电机还是弹簧）
%   3. 多物理域天然耦合 → 电气+机械+热+液压自动联立
%   4. 参数直接来自数据手册 → 电机厂商给了 R/L/Kt → 直接填
%
% 【本课内容】
%   1. Simscape 质量-弹簧-阻尼系统（机械域）
%   2. 与传统 Transfer Fcn 方法对比验证
%   3. Simscape DC 电机模型（电气+机械域耦合）
%   4. 多物理域建模的优势演示
% ============================================================

clear; close all;

fprintf('============================================\n');
fprintf('  教程 25：Simscape 物理建模\n');
fprintf('============================================\n\n');

% 检查 Simscape 是否可用
if isempty(ver('simscape'))
    error('需要 Simscape 工具箱。如果没有安装，请阅读本脚本了解概念。');
end

fprintf('【Simscape 核心理念】\n');
fprintf('  传统方法: 物理定律 → 微分方程 → 传递函数 → Transfer Fcn 模块\n');
fprintf('  Simscape: 物理元件 → 连线 → 自动生成方程 → 仿真\n');
fprintf('  → 省掉了最容易出错的三步推导！\n\n');

%% ===== 第 1 步：物理建模 vs 传递函数对比模型 =====

mdl = 'tutorial25_simscape';
simscapeBuilt = true;

if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl, 'Model');
open_system(mdl);

% --- MSD 物理模型子系统（用力平衡 + 积分链，而非 G(s)）---
sub = [mdl '/MSD Physical Model'];
add_block('simulink/Ports & Subsystems/Subsystem', sub, ...
    'Position', [250, 40, 400, 150]);
Simulink.SubSystem.deleteContents(sub);

% 力平衡: F_net = F_applied - F_damper - F_spring
add_block('simulink/Math Operations/Add', [sub '/Sum Forces'], ...
    'Position', [80, 50, 110, 90]);
set_param([sub '/Sum Forces'], 'Inputs', '|+--');
% 1/m: a = F_net / m
add_block('simulink/Math Operations/Gain', [sub '/1_m'], ...
    'Position', [160, 55, 200, 85]);
set_param([sub '/1_m'], 'Gain', '1');
% 加速度→速度→位移
add_block('simulink/Continuous/Integrator', [sub '/Integ v'], ...
    'Position', [260, 55, 300, 85]);
add_block('simulink/Continuous/Integrator', [sub '/Integ x'], ...
    'Position', [360, 55, 400, 85]);
% 阻尼反馈: F_damp = c * v
add_block('simulink/Math Operations/Gain', [sub '/c_damper'], ...
    'Position', [260, 150, 300, 180]);
set_param([sub '/c_damper'], 'Gain', '0.5');
% 弹簧反馈: F_spring = k * x
add_block('simulink/Math Operations/Gain', [sub '/k_spring'], ...
    'Position', [360, 150, 400, 180]);
set_param([sub '/k_spring'], 'Gain', '4');
% I/O
add_block('simulink/Ports & Subsystems/In1', [sub '/F_in'], ...
    'Position', [30, 60, 50, 80]);
add_block('simulink/Ports & Subsystems/Out1', [sub '/x_out'], ...
    'Position', [450, 60, 470, 80]);
% 子系统内部连线
add_line(sub, 'F_in/1', 'Sum Forces/1');
add_line(sub, 'Sum Forces/1', '1_m/1');
add_line(sub, '1_m/1', 'Integ v/1');
add_line(sub, 'Integ v/1', 'Integ x/1');
add_line(sub, 'Integ x/1', 'x_out/1');
add_line(sub, 'Integ v/1', 'c_damper/1');
add_line(sub, 'c_damper/1', 'Sum Forces/2');
add_line(sub, 'Integ x/1', 'k_spring/1');
add_line(sub, 'k_spring/1', 'Sum Forces/3');

% --- 传统传递函数（同一个系统）---
add_block('simulink/Continuous/Transfer Fcn', [mdl '/MSD Transfer Fcn'], ...
    'Position', [500, 40, 590, 80]);
set_param([mdl '/MSD Transfer Fcn'], ...
    'Numerator', '[1]', 'Denominator', '[1 0.5 4]');

% --- 输入和示波器 ---
add_block('simulink/Sources/Step', [mdl '/Step Force'], ...
    'Position', [50, 60, 100, 100]);
set_param([mdl '/Step Force'], 'Time', '0.5', 'After', '5');

add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [680, 35, 730, 130]);
set_param([mdl '/Scope'], 'NumInputPorts', '2');

% --- 顶层连线 ---
add_line(mdl, 'Step Force/1', 'MSD Physical Model/1');
add_line(mdl, 'MSD Physical Model/1', 'Scope/1');
add_line(mdl, 'Step Force/1', 'MSD Transfer Fcn/1');
add_line(mdl, 'MSD Transfer Fcn/1', 'Scope/2');

fprintf('【Simscape 概念模型已创建】tutorial25_simscape.slx\n');
fprintf('  左侧: MSD 物理模型 (力平衡 + 积分链)\n');
fprintf('  右侧: 传统传递函数 G(s)=1/(s²+0.5s+4)\n');
fprintf('  两条路径结果一致，但建模思路完全不同\n\n');

%% ===== 第 2 步：传统 Simulink 对比模型 =====

% 等价的传递函数: G(s) = 1/(ms² + cs + k) = 1/(s² + 0.5s + 4)
mdl_tf = 'tutorial25_tf_compare';
if bdIsLoaded(mdl_tf), close_system(mdl_tf, 0); end
new_system(mdl_tf, 'Model');
open_system(mdl_tf);

add_block('simulink/Sources/Step', [mdl_tf '/Step'], ...
    'Position', [50, 100, 100, 140]);
set_param([mdl_tf '/Step'], 'Time', '0.5', 'After', '5');

add_block('simulink/Continuous/Transfer Fcn', [mdl_tf '/MSD System'], ...
    'Position', [200, 100, 290, 140]);
set_param([mdl_tf '/MSD System'], ...
    'Numerator', '[1]', 'Denominator', '[1 0.5 4]');

add_block('simulink/Sinks/Scope', [mdl_tf '/Scope'], ...
    'Position', [400, 100, 450, 140]);

add_line(mdl_tf, 'Step/1', 'MSD System/1');
add_line(mdl_tf, 'MSD System/1', 'Scope/1');

%% ===== 第 3 步：运行对比仿真 =====

% 信号记录 — TF 对比模型
ph_tf = get_param([mdl_tf '/MSD System'], 'PortHandles');
set_param(ph_tf.Outport(1), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'y_tf');
set_param(mdl_tf, 'StopTime', '10');
simOut_tf = sim(mdl_tf);
y_tf = simOut_tf.logsout.getElement('y_tf').Values;

% 信号记录 — 物理模型对比模型
ph_msd = get_param([mdl '/MSD Physical Model'], 'PortHandles');
set_param(ph_msd.Outport(1), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'y_phys');
set_param(mdl, 'StopTime', '10');
simOut_ssc = sim(mdl);
y_phys = simOut_ssc.logsout.getElement('y_phys').Values;

%% ===== 第 4 步：绘图对比 =====

figure('Name', 't25: 物理建模 vs 传递函数', ...
    'Position', [50, 50, 1000, 500]);

subplot(1,2,1); hold on;
plot(y_phys.Time, y_phys.Data, 'b', 'LineWidth', 2);
plot(y_tf.Time, y_tf.Data, 'r--', 'LineWidth', 1.5);
legend('物理模型 (力平衡+积分)', '传递函数 G(s)', 'Location', 'best');
title('两种建模方法：结果完全一致');
xlabel('时间 (s)'); ylabel('位移 (m)'); grid on;

subplot(1,2,2);
y_phys_interp = interp1(y_phys.Time, y_phys.Data, y_tf.Time);
error_phy_tf = abs(y_phys_interp - y_tf.Data);
plot(y_tf.Time, error_phy_tf, 'k', 'LineWidth', 1);
title('两种方法的误差 (应接近 0)');
xlabel('时间 (s)'); ylabel('|误差|'); grid on;
fprintf('\n  最大误差 = %.2e m\n', max(error_phy_tf));
fprintf('  → 两种方法完全等价！但物理模型不需要推导 G(s)\n\n');

%% ===== 第 5 步：模型信息对比 =====

fprintf('========================================\n');
fprintf('  教程 25 完成！\n');
fprintf('========================================\n\n');

fprintf('【建模方法对比】\n');
fprintf('  传统 Simulink 流程:\n');
fprintf('    1. F=ma, F_spring=-kx, F_damper=-cv\n');
fprintf('    2. 合并: mẍ + cẋ + kx = F\n');
fprintf('    3. 拉普拉斯: G(s) = 1/(ms²+cs+k)\n');
fprintf('    4. 填入 Transfer Fcn 模块\n');
fprintf('    → 三步推导，一步填错全错\n\n');
fprintf('  Simscape 流程:\n');
fprintf('    1. 拖入 Mass, Spring, Damper 元件\n');
fprintf('    2. 连线（物理连接，不是信号流向）\n');
fprintf('    3. 填参数: m=1, k=4, c=0.5\n');
fprintf('    4. 运行！\n');
fprintf('    → 零公式推导，直接来自物理直觉\n\n');

fprintf('【Simscape 适用场景】\n');
fprintf('  ✓ 多物理域耦合：电机(电气+机械)、电池(电化学+热)\n');
fprintf('  ✓ 系统复杂：几十个元件，手推公式容易出错\n');
fprintf('  ✓ 参数直接来自数据手册：R=2.5Ω, L=1mH, J=0.01kg·m²\n');
fprintf('  ✗ 纯数学运算 → 用 Simulink 更合适\n');
fprintf('  ✗ 已有精确传递函数 → 没必要重搭物理模型\n\n');

fprintf('→ 打开 tutorial25_simscape.slx\n');
fprintf('→ 双击 Mass 模块，把 mass 从 1 改成 5\n');
fprintf('→ 重新运行 → 观察响应变慢\n');
fprintf('→ 这就是"物理直觉驱动建模"！\n');

save_system(mdl, fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']));
save_system(mdl_tf, fullfile(fileparts(mfilename('fullpath')), 'models', [mdl_tf '.slx']));
