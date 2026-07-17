%% ============================================================
% 教程 06：信号源与输出 — 仿真数据输入/输出的多种方式
% 目标：掌握常用信号源和输出模块，以及数据导入导出技巧
%
% 涵盖内容：
%  - 信号源：Sine Wave, Step, Ramp, Signal Builder, From Workspace
%  - 输出：  Scope, To Workspace, To File, Display, XY Graph
%  - 数据在 MATLAB 与 Simulink 之间交换的方法
% ============================================================

clear; close all;

%% ---------- 准备工作：在 MATLAB 中定义自定义输入信号 ----------
% From Workspace 模块需要一个 [时间, 数据] 的矩阵或 timeseries 对象
t_input = (0:0.01:10)';                    % 时间列向量
u_input = [t_input, sin(t_input) + 0.5*sin(3*t_input)];  % 自定义复合信号

% From Workspace 从 base 工作区取变量；脚本若在函数中运行(如 run_all_tutorials)
% 局部变量不可见，因此显式放入 base 工作区
assignin('base', 'u_input', u_input);

mdl = 'tutorial06_sources_sinks';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl, 'Model');
open_system(mdl);

%% ---------- 信号源区域 ----------

% (A) 斜坡信号 — 线性增长
add_block('simulink/Sources/Ramp', [mdl '/Ramp'], ...
    'Position', [30, 80, 80, 120]);
set_param([mdl '/Ramp'], 'Slope', '2');  % 斜率 2/s

% (B) 脉冲信号 — 周期性脉冲
add_block('simulink/Sources/Pulse Generator', [mdl '/Pulse'], ...
    'Position', [30, 180, 80, 220]);
set_param([mdl '/Pulse'], ...
    'Amplitude', '5', ...
    'Period',    '2', ...
    'PulseWidth', '25');  % 占空比 25%

% (C) 从工作区导入信号
add_block('simulink/Sources/From Workspace', [mdl '/From Workspace'], ...
    'Position', [30, 300, 80, 340]);
set_param([mdl '/From Workspace'], 'VariableName', 'u_input');

% (D) 信号生成器 — 可视化编辑信号
add_block('simulink/Sources/Signal Builder', [mdl '/Signal Builder'], ...
    'Position', [30, 400, 80, 460]);

%% ---------- 输出/显示区域 ----------

% (A) 标准示波器
add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [200, 80, 250, 130]);

% (B) 数字显示器 — 显示当前仿真时刻的值
add_block('simulink/Sinks/Display', [mdl '/Display'], ...
    'Position', [200, 180, 280, 220]);

% (C) 输出到工作区
add_block('simulink/Sinks/To Workspace', [mdl '/To Workspace'], ...
    'Position', [200, 290, 250, 330]);
set_param([mdl '/To Workspace'], ...
    'VariableName', 'saved_data', ...
    'SaveFormat',  'Timeseries');

% (D) 输出到文件
add_block('simulink/Sinks/To File', [mdl '/To File'], ...
    'Position', [200, 390, 250, 430]);
set_param([mdl '/To File'], 'Filename', 'tutorial06_output.mat');

% (E) 信号终结器 — 终止未连接的输出线（避免警告）
add_block('simulink/Sinks/Terminator', [mdl '/Terminator'], ...
    'Position', [400, 300, 420, 320]);

%% ---------- 路由模块 ----------
% 用 Mux 观察哪些信号可以被合并
add_block('simulink/Signal Routing/Mux', [mdl '/Mux'], ...
    'Position', [120, 80, 140, 460]);
set_param([mdl '/Mux'], 'Inputs', '4');

%% ---------- 连线 ----------
add_line(mdl, 'Ramp/1', 'Mux/1');
add_line(mdl, 'Pulse/1', 'Mux/2');
add_line(mdl, 'From Workspace/1', 'Mux/3');
add_line(mdl, 'Signal Builder/1', 'Mux/4');

add_line(mdl, 'Mux/1', 'Scope/1');

% Display 只显示第一路（Ramp）的数值
add_line(mdl, 'Ramp/1', 'Display/1');

add_line(mdl, 'Mux/1', 'To Workspace/1');
add_line(mdl, 'Mux/1', 'To File/1');
add_line(mdl, 'Signal Builder/1', 'Terminator/1');

%% ---------- 运行仿真 ----------
simOut = sim(mdl);

%% ---------- 使用 To Workspace 保存的数据 ----------
% To Workspace 记录的变量包含在 sim 的返回值中
saved_data = simOut.saved_data;
nSignals = size(saved_data.Data, 2);
figure;
for i = 1:min(3, nSignals)
    subplot(3,1,i);
    plot(saved_data.Time, saved_data.Data(:,i), 'LineWidth', 1.2);
    xlabel('时间 (s)'); ylabel(sprintf('信号 %d', i)); grid on;
end
sgtitle('To Workspace 导出的数据可在 MATLAB 中直接使用');

fprintf('教程 06 完成！\n');
fprintf('重点回顾：\n');
fprintf('  From Workspace — 将 MATLAB 数据作为输入信号\n');
fprintf('  To Workspace — 将仿真结果保存到 MATLAB 工作区\n');
fprintf('  To File — 将仿真结果保存到 .mat 文件\n');
save_system(mdl, fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']));
fprintf('  Display — 实时显示当前信号值（适合调试）\n');
