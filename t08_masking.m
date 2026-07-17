%% ============================================================
% 教程 08：子系统封装（Mask）— 做自己的"自定义模块"
% 目标：学会给子系统加 Mask，暴露参数界面
%       做成像 Gain、Transfer Fcn 一样可配置的复用模块
%
% 核心概念：
%   Mask = 给子系统穿上"参数界面"的外衣
%   用户双击子系统 → 弹出参数对话框（不是直接进入内部）
%   内部模块用变量名引用参数 → 像函数参数一样灵活
%
% 理论联系：
%   Mask 参数 = 函数的形参
%   子系统内部 = 函数体
%   实例化 = 函数调用（传入不同实参）
%   这就是 "参数化建模" 的思想
% ============================================================

clear; close all;

mdl = 'tutorial08_masking';
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

fprintf('=== 教程 08：子系统封装（Mask）===\n');

%% ===== 第 1 步：创建一个可配置的"可编程增益滤波器"子系统 =====

% 添加子系统
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/ProgGainFilter'], ...
    'Position', [150, 90, 250, 160]);
fprintf('  [OK] ProgGainFilter 子系统添加成功\n');

% 进入子系统内部，搭建 Gain + Filter 模板
% 用变量名代替具体数值，稍后由 Mask 参数赋值
add_block('simulink/Math Operations/Gain', [mdl '/ProgGainFilter/Gain'], ...
    'Position', [200, 90, 260, 130]);
set_param([mdl '/ProgGainFilter/Gain'], 'Gain', 'myGain');  % 变量，不是数字！

add_block('simulink/Continuous/Transfer Fcn', [mdl '/ProgGainFilter/Filter'], ...
    'Position', [380, 90, 460, 130]);
set_param([mdl '/ProgGainFilter/Filter'], ...
    'Numerator',   '[1]', ...
    'Denominator', '[myTau 1]');  % myTau 也是变量！

% 删除默认连线，重新布线
delete_line([mdl '/ProgGainFilter'], 'In1/1', 'Out1/1');
add_line([mdl '/ProgGainFilter'], 'In1/1', 'Gain/1');
add_line([mdl '/ProgGainFilter'], 'Gain/1', 'Filter/1');
add_line([mdl '/ProgGainFilter'], 'Filter/1', 'Out1/1');
fprintf('  [OK] 内部模型搭建完成（使用变量 myGain, myTau）\n');

%% ===== 第 2 步：创建 Mask（封装）=====

% 获取 Mask 对象
maskObj = Simulink.Mask.create([mdl '/ProgGainFilter']);

% —— 2a. 添加参数 ——
% 参数 #1：增益值 myGain（Edit 编辑框）
maskObj.addParameter('Name', 'myGain', ...
    'Prompt', '增益 Gain 值:', ...
    'Type', 'edit', ...
    'Value', '3', ...           % 默认值
    'Evaluate', 'on');        % 允许使用 MATLAB 表达式

% 参数 #2：时间常数 myTau（Edit 编辑框）
maskObj.addParameter('Name', 'myTau', ...
    'Prompt', '滤波时间常数 τ:', ...
    'Type', 'edit', ...
    'Value', '0.5', ...
    'Evaluate', 'on');

% 参数 #3：是否启用滤波（Checkbox 复选框）
maskObj.addParameter('Name', 'useFilter', ...
    'Prompt', '启用一阶滤波', ...
    'Type', 'checkbox', ...
    'Value', 'on', ...
    'Evaluate', 'on');

fprintf('  [OK] Mask 参数添加完成（myGain, myTau, useFilter）\n');

% —— 2b. 设置显示属性 ——
% 在模块图标上显示参数值（方便一眼看清当前配置）
maskObj.set('Display', ...
    'sprintf(''增益=%.1f\\nτ=%.2f'', myGain, myTau);');
% 给模块换个标签
set_param([mdl '/ProgGainFilter'], ...
    'AttributesFormatString', '可编程\nGain+Filter');

% —— 2c. 添加说明文档 ——
maskObj.set('Description', sprintf([ ...
    '可编程增益+一阶低通滤波器。\n', ...
    '参数：myGain - 增益倍数\n', ...
    '       myTau  - 滤波时间常数 (秒)\n', ...
    '       useFilter - 是否启用一阶滤波']));

fprintf('  [OK] Mask 外观与文档配置完成\n');

%% ===== 第 3 步：添加第二个子系统（用相同模板做不同实例）=====

% 复制 ProgGainFilter 作为第二个实例
add_block([mdl '/ProgGainFilter'], [mdl '/ProgGainFilter_Instance2']);
set_param([mdl '/ProgGainFilter_Instance2'], ...
    'Position', [150, 250, 250, 320]);

% 给第二个实例设置不同的参数
set_param([mdl '/ProgGainFilter_Instance2'], 'myGain', '10', 'myTau', '2.0');
set_param([mdl '/ProgGainFilter_Instance2'], ...
    'AttributesFormatString', '实例2\nGain+Filter');

fprintf('  [OK] 第二个实例创建完成（Gain=10, τ=2.0）\n');

%% ===== 第 4 步：搭建比较测试平台 =====

% 信号源
add_block('simulink/Sources/Sine Wave', [mdl '/Sine'], ...
    'Position', [50, 100, 140, 140]);
set_param([mdl '/Sine'], 'Amplitude', '3');

% 三端口示波器
add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [450, 90, 500, 160]);
set_param([mdl '/Scope'], 'NumInputPorts', '3');

% 顶层连线：Sine 分三路，一路直接看原始信号，两路各经过一个实例
add_line(mdl, 'Sine/1', 'ProgGainFilter/1');
add_line(mdl, 'ProgGainFilter/1', 'Scope/1');

add_line(mdl, 'Sine/1', 'ProgGainFilter_Instance2/1');
add_line(mdl, 'ProgGainFilter_Instance2/1', 'Scope/2');

add_line(mdl, 'Sine/1', 'Scope/3');

% 加 To Workspace 模块捕获信号用于绘图
add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_In'], ...
    'Position', [350, 370, 420, 410]);
set_param([mdl '/ToWS_In'], 'VariableName', 'sig_in');
add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_Out1'], ...
    'Position', [350, 430, 420, 470]);
set_param([mdl '/ToWS_Out1'], 'VariableName', 'sig_out1');
add_block('simulink/Sinks/To Workspace', [mdl '/ToWS_Out2'], ...
    'Position', [350, 490, 420, 530]);
set_param([mdl '/ToWS_Out2'], 'VariableName', 'sig_out2');

% 将信号也接到 To Workspace
add_line(mdl, 'ProgGainFilter/1', 'ToWS_Out1/1');
add_line(mdl, 'ProgGainFilter_Instance2/1', 'ToWS_Out2/1');
add_line(mdl, 'Sine/1', 'ToWS_In/1');

% 美化布局
Simulink.BlockDiagram.arrangeSystem(mdl);

fprintf('  [OK] 顶层测试平台搭建完成\n');

%% ===== 第 5 步：运行仿真 =====
fprintf('\n=== 运行仿真 ===\n');
set_param(mdl, 'StopTime', '15');
simOut = sim(mdl);

%% ===== 第 6 步：绘图对比 =====

figure('Name', 't08: Mask 封装 — 参数化模块对比', 'Position', [100, 200, 800, 500]);

% 从 To Workspace 模块获取数据
inSig  = simOut.get('sig_in');
out1   = simOut.get('sig_out1');
out2   = simOut.get('sig_out2');

% 根据 To Workspace 输出的格式提取时间和数据
% 默认格式：结构体，字段为 .time 和 .signals.values
if isstruct(inSig)
    t_in  = inSig.time;
    d_in  = inSig.signals.values;
    t_o1  = out1.time;
    d_o1  = out1.signals.values;
    t_o2  = out2.time;
    d_o2  = out2.signals.values;
else  % timeseries 格式
    t_in  = inSig.Time;
    d_in  = inSig.Data;
    t_o1  = out1.Time;
    d_o1  = out1.Data;
    t_o2  = out2.Time;
    d_o2  = out2.Data;
end

plot(t_in, d_in, 'Color', [0.5 0.5 0.5], 'LineWidth', 1.5);
hold on;
plot(t_o1, d_o1, 'b', 'LineWidth', 2);
plot(t_o2, d_o2, 'r', 'LineWidth', 2);
hold off;

legend('原始正弦 (A=3)', ...
    '实例 1: Gain=3, τ=0.5', ...
    '实例 2: Gain=10, τ=2.0', ...
    'Location', 'southeast');
title('Mask 封装效果：同一模板，不同参数 → 不同输出');
xlabel('时间 (s)'); ylabel('幅值'); grid on;

%% ===== 第 7 步：操作练习指引 =====
fprintf('\n========================================\n');
fprintf('  教程 08 完成！\n');
fprintf('========================================\n');

fprintf('\n现在请动手验证你的理解：\n\n');

fprintf('  1. 双击 ProgGainFilter 模块\n');
fprintf('     → 看到了什么？不是进入内部，而是弹出参数对话框！\n');
fprintf('     → 这就是 Mask 的作用：拦截双击，显示参数界面\n\n');

fprintf('  2. 修改实例 1 的 myGain = 5, myTau = 1.0\n');
fprintf('     方法：双击模块 → 修改数值 → OK → 重新运行脚本\n');
fprintf('     观察曲线变化：增益变大 → 幅值变大；τ变大 → 延迟更大\n\n');

fprintf('  3. 要进入模块内部看实现：\n');
fprintf('     右键模块 → Mask → Look Under Mask\n');
fprintf('     你会看到 Gain 模块的值是 "myGain"（变量名），不是具体数字\n\n');

fprintf('  4. 右键 → Mask → Edit Mask\n');
fprintf('     进入 Mask 编辑器，看看参数是怎么定义的\n\n');

fprintf('  5. 尝试新增一个参数（如滤波器初始条件 IC）\n');
fprintf('     在 Mask Editor 中点击 Parameters → 添加新参数\n');
fprintf('     然后在 Filter 模块中引用它\n\n');

fprintf('========================================\n');
fprintf('  理论总结\n');
fprintf('========================================\n');

fprintf('\n  Mask 封装的三要素：\n');
fprintf('  ┌──────────────┬──────────────────────────┐\n');
fprintf('  │ Mask 参数    │ 函数形参 myGain, myTau    │\n');
fprintf('  │ 子系统内部    │ 函数体（用变量引用参数）   │\n');
fprintf('  │ 模块实例      │ 函数调用（传入不同实参）   │\n');
fprintf('  └──────────────┴──────────────────────────┘\n\n');

fprintf('  为什么要学 Mask？\n');
fprintf('  机电系统的模块都是参数化的：\n');
fprintf('  - 电机模块：额定功率、转速、电感、电阻...\n');
fprintf('  - 传感器：量程、精度、时间常数...\n');
fprintf('  - 控制器：Kp, Ki, Kd, 采样周期...\n');
fprintf('  这些参数化模块本质上都是 Masked Subsystem！\n\n');

fprintf('  下一课预告：t09 将介绍状态空间模型（现代控制理论基础）。\n');
