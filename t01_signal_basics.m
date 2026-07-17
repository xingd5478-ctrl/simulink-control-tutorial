%% ============================================================
% 教程 01：信号基础 — 正弦波、增益与示波器
% 目标：认识 Simulink 的基本构成 — 模块(Block)、信号线(Signal)、仿真运行
% ============================================================

clear; close all;

%% ---------- 第1步：创建新模型 ----------
% new_system 创建一个空白模型窗口
mdl = 'tutorial01_signals';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl, 'Model');
open_system(mdl);

%% ---------- 第2步：添加模块 ----------
% add_block 的格式：add_block('库名/模块名', '模型名/自定义名')
% Simulink 自带丰富的模块库，以 simulink 为前缀

% 正弦波信号源 (Sources 库)
add_block('simulink/Sources/Sine Wave', [mdl '/Sine Wave'], ...
    'Position', [50, 50, 100, 90]);

% 增益模块 (Math Operations 库) — 将信号放大或缩小
add_block('simulink/Math Operations/Gain', [mdl '/Gain'], ...
    'Position', [160, 50, 210, 90]);

% 示波器 (Sinks 库) — 观测信号波形
add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [320, 50, 370, 90]);

%% ---------- 第3步：连线 ----------
% add_line 连接模块的输出端口(1)到输入端口(1)
add_line(mdl, 'Sine Wave/1', 'Gain/1');
add_line(mdl, 'Gain/1', 'Scope/1');

%% ---------- 第4步：设置参数 ----------
% set_param 修改模块参数，参数名在模块属性对话框中可以看到
set_param([mdl '/Sine Wave'], 'Amplitude', '3');  % 振幅改为 3
set_param([mdl '/Gain'],      'Gain',      '2');  % 增益改为 2

% 开启 Gain 输出端口的信号记录，仿真结果会出现在 simOut.logsout 中
ph = get_param([mdl '/Gain'], 'PortHandles');
set_param(ph.Outport(1), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'gain_out');

%% ---------- 第5步：运行仿真 ----------
% 默认仿真 10 秒
simOut = sim(mdl);

% 双击模型中的 Scope 即可看到波形
% 期望结果：振幅 3 的正弦波被放大 2 倍，示波器显示振幅为 6 的正弦波

%% ---------- 第6步：在 MATLAB 中绘图验证 ----------
% 也可以将记录的信号取出，用 MATLAB 绘图
sig = simOut.logsout.getElement('gain_out').Values;

figure;
plot(sig.Time, sig.Data, 'LineWidth', 1.5);
title('增益模块输出：3 × 2 = 6 振幅的正弦波');
xlabel('时间 (s)'); ylabel('幅值'); grid on;

fprintf('教程 01 完成！打开模型 "%s" 查看模块连接。\n', mdl);
