%% ============================================================
% 教程 07：子系统与模型封装 — 把模型"打包"成黑盒
% 目标：学会创建子系统、在子系统内部搭建模型、
%       理解模型层次化设计的意义
%
% 核心概念：
%   子系统 = 把一堆模块装进一个盒子
%   对外只暴露输入输出端口，双击可以进入内部
%   就像编程里的"函数"——调用者不需要知道内部实现
% ============================================================

clear; close all;

mdl = 'tutorial07_subsystem';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl, 'Model');
open_system(mdl);

fprintf('=== 教程 07：子系统与模型封装 ===\n');

%% ===== 第 1 步：创建顶层模型 =====
% 顶层放一个信号源和一个示波器，中间是一个子系统

% 正弦波输入
add_block('simulink/Sources/Sine Wave', [mdl '/Sine_Input'], ...
    'Position', [50, 100, 150, 140]);
set_param([mdl '/Sine_Input'], 'Amplitude', '5');
fprintf('  [OK] Sine_Input 添加成功\n');

% 子系统 A — Signal_Processor（信号处理流水线）
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Signal_Processor'], ...
    'Position', [280, 90, 380, 160]);
fprintf('  [OK] Signal_Processor 子系统添加成功\n');

% 示波器（两端口）
add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [520, 90, 570, 160]);
set_param([mdl '/Scope'], 'NumInputPorts', '2');
fprintf('  [OK] Scope 添加成功\n');

% 顶层连线
add_line(mdl, 'Sine_Input/1', 'Signal_Processor/1');
add_line(mdl, 'Signal_Processor/1', 'Scope/1');
add_line(mdl, 'Sine_Input/1', 'Scope/2');
fprintf('  [OK] 顶层连线完成\n');

%% ===== 第 2 步：在 Signal_Processor 内部搭模型 =====

% Gain = 3
add_block('simulink/Math Operations/Gain', [mdl '/Signal_Processor/Gain'], ...
    'Position', [200, 90, 260, 130]);
set_param([mdl '/Signal_Processor/Gain'], 'Gain', '3');

% 一阶滤波器，τ = 0.5
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Signal_Processor/Filter'], ...
    'Position', [380, 90, 460, 130]);
set_param([mdl '/Signal_Processor/Filter'], ...
    'Numerator',   '[1]', ...
    'Denominator', '[0.5 1]');

% 删除子系统默认的 In1→Out1 连线，再重新布线
delete_line([mdl '/Signal_Processor'], 'In1/1', 'Out1/1');
add_line([mdl '/Signal_Processor'], 'In1/1', 'Gain/1');
add_line([mdl '/Signal_Processor'], 'Gain/1', 'Filter/1');
add_line([mdl '/Signal_Processor'], 'Filter/1', 'Out1/1');
fprintf('  [OK] Signal_Processor 内部：Gain + Filter 搭建完成\n');

%% ===== 第 3 步：创建第二个子系统 Nested_Demo =====

add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Nested_Demo'], ...
    'Position', [280, 260, 380, 330]);
fprintf('  [OK] Nested_Demo 子系统添加成功\n');

% 顶层连线：Sine_Input → Nested_Demo
add_line(mdl, 'Sine_Input/1', 'Nested_Demo/1');

% 第二个示波器，用来看嵌套子系统的输出
add_block('simulink/Sinks/Scope', [mdl '/Scope_Nested'], ...
    'Position', [520, 260, 570, 330]);
add_line(mdl, 'Nested_Demo/1', 'Scope_Nested/1');
fprintf('  [OK] Nested_Demo 顶层连线完成\n');

%% ===== 第 4 步：在 Nested_Demo 内部搭嵌套 =====

% 第一层：Gain_Inner = 2
add_block('simulink/Math Operations/Gain', [mdl '/Nested_Demo/Gain_Inner'], ...
    'Position', [150, 90, 210, 130]);
set_param([mdl '/Nested_Demo/Gain_Inner'], 'Gain', '2');

% 第一层：再塞一个子系统 Inner_Sub
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Nested_Demo/Inner_Sub'], ...
    'Position', [330, 90, 430, 160]);

% 第一层内部连线
delete_line([mdl '/Nested_Demo'], 'In1/1', 'Out1/1');
add_line([mdl '/Nested_Demo'], 'In1/1', 'Gain_Inner/1');
add_line([mdl '/Nested_Demo'], 'Gain_Inner/1', 'Inner_Sub/1');
add_line([mdl '/Nested_Demo'], 'Inner_Sub/1', 'Out1/1');
fprintf('  [OK] Nested_Demo 内部：Gain_Inner + Inner_Sub 搭建完成\n');

%% ===== 第 5 步：在 Inner_Sub（第三层）内部搭模型 =====

add_block('simulink/Math Operations/Gain', [mdl '/Nested_Demo/Inner_Sub/Gain_Deep'], ...
    'Position', [200, 90, 260, 130]);
set_param([mdl '/Nested_Demo/Inner_Sub/Gain_Deep'], 'Gain', '4');

% 给深层模块加个显眼标签
set_param([mdl '/Nested_Demo/Inner_Sub/Gain_Deep'], ...
    'AttributesFormatString', '第3层！\nGain=4');

delete_line([mdl '/Nested_Demo/Inner_Sub'], 'In1/1', 'Out1/1');
add_line([mdl '/Nested_Demo/Inner_Sub'], 'In1/1', 'Gain_Deep/1');
add_line([mdl '/Nested_Demo/Inner_Sub'], 'Gain_Deep/1', 'Out1/1');
fprintf('  [OK] Inner_Sub 内部（第三层）：Gain_Deep=4 搭建完成\n');

%% ===== 第 6 步：开启信号记录 + 运行仿真 =====

% 记录关键信号，便于在 MATLAB 中绘图
phIn = get_param([mdl '/Sine_Input'], 'PortHandles');
set_param(phIn.Outport(1), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'SineIn');
phSP = get_param([mdl '/Signal_Processor'], 'PortHandles');
set_param(phSP.Outport(1), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'SignalProcOut');
phND = get_param([mdl '/Nested_Demo'], 'PortHandles');
set_param(phND.Outport(1), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'NestedOut');

fprintf('\n=== 运行仿真 ===\n');
simOut = sim(mdl);

%% ===== 第 7 步：绘图对比 =====

inSig  = simOut.logsout.getElement('SineIn').Values;
outSig = simOut.logsout.getElement('SignalProcOut').Values;
nestOut = simOut.logsout.getElement('NestedOut').Values;

figure;

% 上图：Signal_Processor 输入 vs 输出
subplot(2,1,1);
plot(inSig.Time, inSig.Data, 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
hold on;
plot(outSig.Time, outSig.Data, 'b', 'LineWidth', 2);
hold off;
legend('输入 (正弦 A=5)', '输出 (Gain=3 + 滤波)', 'Location', 'southeast');
title('Signal Processor：放大 3 倍 + 一阶滤波（有延迟）');
xlabel('时间 (s)'); ylabel('幅值'); grid on;

% 下图：Nested_Demo 输出
subplot(2,1,2);
plot(nestOut.Time, nestOut.Data, 'm', 'LineWidth', 2);
title('Nested Demo 输出：Gain=2 × Gain=4 = 8 倍放大（无滤波，无延迟）');
xlabel('时间 (s)'); ylabel('幅值'); grid on;

%% ===== 总结 =====
fprintf('\n========================================\n');
fprintf('  教程 07 完成！\n');
fprintf('========================================\n');
fprintf('\n现在请动手操作：\n');
fprintf('  1. 双击 Signal_Processor → 看到内部 Gain + Filter\n');
fprintf('  2. 双击 Nested_Demo → 看到 Gain_Inner + Inner_Sub\n');
fprintf('  3. 再双击 Inner_Sub → 看到第三层的 Gain_Deep\n');
fprintf('  4. 点工具栏 ← 回到顶层\n');
fprintf('\n核心理解：\n');
fprintf('  顶层：只有信号源 + 子系统 + 示波器（简洁！）\n');
fprintf('  第二层：Signal_Processor 内部（Gain + Filter）\n');
fprintf('  第三层：Nested_Demo → Inner_Sub 内部（Gain_Deep）\n');
fprintf('\n  子系统 = 编程里的函数封装\n');
fprintf('  路径用 / 表示层级，和文件夹一样\n');
fprintf('  可以无限嵌套，每一层独立、互不影响\n');
