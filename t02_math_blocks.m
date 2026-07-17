%% ============================================================
% 教程 02：数学运算模块 — 加减乘除与信号路由
% 目标：学习 Sum、Product、Mux、Demux 等常用数学模块
% ============================================================

clear; close all;

mdl = 'tutorial02_math';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl, 'Model');
open_system(mdl);

%% ---------- 创建信号源 ----------
% 两个不同频率的正弦波
add_block('simulink/Sources/Sine Wave', [mdl '/Sine1'], ...
    'Position', [50, 60, 130, 100]);
set_param([mdl '/Sine1'], 'Frequency', '1');  % 1 rad/s

add_block('simulink/Sources/Sine Wave', [mdl '/Sine2'], ...
    'Position', [50, 160, 130, 200]);
set_param([mdl '/Sine2'], 'Frequency', '3');  % 3 rad/s

% 常量模块
add_block('simulink/Sources/Constant', [mdl '/Constant'], ...
    'Position', [50, 270, 100, 300]);
set_param([mdl '/Constant'], 'Value', '5');

%% ---------- 加法器 (Sum) ----------
% Sum 模块默认有两个输入，图标显示为圆形符号列表
% 可以用 |+- 指定每个端口的符号，例如 |++ 表示两个正输入
add_block('simulink/Math Operations/Add', [mdl '/Sum'], ...
    'Position', [210, 100, 240, 130]);
set_param([mdl '/Sum'], 'Inputs', '|++');  % 两个正输入相加

%% ---------- 乘法器 (Product) ----------
add_block('simulink/Math Operations/Product', [mdl '/Product'], ...
    'Position', [330, 100, 360, 130]);

%% ---------- 信号合并 (Mux) — 将多路信号合成一路向量 ----------
add_block('simulink/Signal Routing/Mux', [mdl '/Mux'], ...
    'Position', [450, 100, 470, 200]);
set_param([mdl '/Mux'], 'Inputs', '3');

%% ---------- 信号拆分 (Demux) — 将向量拆回多路信号 ----------
add_block('simulink/Signal Routing/Demux', [mdl '/Demux'], ...
    'Position', [530, 100, 550, 200]);
set_param([mdl '/Demux'], 'Outputs', '3');

%% ---------- 示波器 ----------
add_block('simulink/Sinks/Scope', [mdl '/Scope1'], ...
    'Position', [330, 250, 380, 290]);
add_block('simulink/Sinks/Scope', [mdl '/Scope2'], ...
    'Position', [650, 100, 700, 200]);
set_param([mdl '/Scope2'], 'NumInputPorts', '3');  % Demux 的三路输出各占一个端口

%% ---------- 连线 ----------
% Sine1 和 Sine2 进加法器
add_line(mdl, 'Sine1/1', 'Sum/1');
add_line(mdl, 'Sine2/1', 'Sum/2');

% 和信号进乘法器的一个端口，常量进另一个端口
add_line(mdl, 'Sum/1', 'Product/1');
add_line(mdl, 'Constant/1', 'Product/2');

% 乘法结果不进 Mux（我们先看乘积波形）
add_line(mdl, 'Product/1', 'Scope1/1');

% 三路信号合并后拆分，演示 Mux/Demux 配对
add_line(mdl, 'Sine1/1', 'Mux/1');
add_line(mdl, 'Sine2/1', 'Mux/2');
add_line(mdl, 'Product/1', 'Mux/3');
add_line(mdl, 'Mux/1', 'Demux/1');
add_line(mdl, 'Demux/1', 'Scope2/1');
add_line(mdl, 'Demux/2', 'Scope2/2');
add_line(mdl, 'Demux/3', 'Scope2/3');

%% ---------- 开启信号记录 ----------
% 记录三路信号以便在 MATLAB 中绘图
ph1 = get_param([mdl '/Sine1'], 'PortHandles');
set_param(ph1.Outport(1), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'Sine1');
ph2 = get_param([mdl '/Sine2'], 'PortHandles');
set_param(ph2.Outport(1), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'Sine2');
phProd = get_param([mdl '/Product'], 'PortHandles');
set_param(phProd.Outport(1), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'Product');
phDemux = get_param([mdl '/Demux'], 'PortHandles');
set_param(phDemux.Outport(1), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'Demux1');
set_param(phDemux.Outport(2), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'Demux2');
set_param(phDemux.Outport(3), 'DataLogging', 'on', 'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'Demux3');

%% ---------- 运行仿真 ----------
simOut = sim(mdl);

%% ---------- MATLAB 绘图 ----------
figure('Name', 't02: 数学运算与信号路由');

% 上图：原始信号 vs 加法输出 vs 乘积
subplot(2,1,1); hold on;
plot(simOut.logsout.getElement('Sine1').Values.Time, ...
     simOut.logsout.getElement('Sine1').Values.Data, 'LineWidth', 1);
plot(simOut.logsout.getElement('Sine2').Values.Time, ...
     simOut.logsout.getElement('Sine2').Values.Data, 'LineWidth', 1);
plot(simOut.logsout.getElement('Product').Values.Time, ...
     simOut.logsout.getElement('Product').Values.Data, 'LineWidth', 2);
hold off;
legend('sin(t)', 'sin(3t)', '(sin(t)+sin(3t))\times5', 'Location', 'best');
title('信号运算：正弦叠加 × 常数增益');
xlabel('时间 (s)'); ylabel('幅值'); grid on;

% 下图：Mux→Demux 还原验证
subplot(2,1,2); hold on;
plot(simOut.logsout.getElement('Demux1').Values.Time, ...
     simOut.logsout.getElement('Demux1').Values.Data, 'LineWidth', 1);
plot(simOut.logsout.getElement('Demux2').Values.Time, ...
     simOut.logsout.getElement('Demux2').Values.Data, 'LineWidth', 1);
plot(simOut.logsout.getElement('Demux3').Values.Time, ...
     simOut.logsout.getElement('Demux3').Values.Data, 'LineWidth', 2);
hold off;
legend('Demux ch1 (sin t)', 'Demux ch2 (sin 3t)', 'Demux ch3 (乘积)', 'Location', 'best');
title('Mux→Demux 信号拆分：三路还原对比');
xlabel('时间 (s)'); ylabel('幅值'); grid on;

fprintf('教程 02 完成！\n');
fprintf('上图: 乘积波形 = (sin(t) + sin(3t)) × 5\n');
save_system(mdl, fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']));
fprintf('下图: Mux→Demux 还原的三路信号\n');
