%% ============================================================
% 教程 05：PID 控制器 — 反馈控制的入门实践
% 目标：理解 P/I/D 三个环节各自的作用
%       掌握反馈闭环的搭建方法
%
% PID 公式：u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt
%   e(t) = 设定值 - 实际输出（误差）
%   Kp — 比例增益：立即响应误差，但可能有余差
%   Ki — 积分增益：消除稳态误差，但可能导致振荡
%   Kd — 微分增益：抑制超调，但对噪声敏感
%
% 被控对象：一阶惯性系统 G(s) = 2/(3s+1)
% ============================================================

clear; close all;

mdl = 'tutorial05_pid';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl, 'Model');
open_system(mdl);

%% ---------- 设定值（阶跃信号）----------
add_block('simulink/Sources/Step', [mdl '/Setpoint'], ...
    'Position', [50, 120, 110, 160]);
set_param([mdl '/Setpoint'], 'Time', '1');

%% ---------- 求和点（误差计算 = 设定值 - 反馈）----------
add_block('simulink/Math Operations/Add', [mdl '/Error'], ...
    'Position', [180, 110, 210, 140]);
set_param([mdl '/Error'], 'Inputs', '|+-');  % 端口1为正，端口2为负

%% ---------- PID 控制器 ----------
add_block('simulink/Continuous/PID Controller', [mdl '/PID'], ...
    'Position', [300, 110, 350, 150]);
set_param([mdl '/PID'], ...
    'P', '3', ...    % 比例增益
    'I', '0.5', ...  % 积分增益
    'D', '0.2');     % 微分增益

%% ---------- 被控对象 (Plant) ----------
% 一阶惯性系统 G(s) = 2/(3s+1)
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Plant'], ...
    'Position', [440, 110, 510, 150]);
set_param([mdl '/Plant'], ...
    'Numerator',   '[2]', ...
    'Denominator', '[3 1]');

%% ---------- 输出显示 ----------
add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [600, 110, 650, 150]);
set_param([mdl '/Scope'], 'NumInputPorts', '2');

%% ---------- 反馈连线 ----------
% 前向通路
add_line(mdl, 'Setpoint/1', 'Error/1');
add_line(mdl, 'Error/1', 'PID/1');
add_line(mdl, 'PID/1', 'Plant/1');

% 输出到示波器
add_line(mdl, 'Plant/1', 'Scope/1');
% 也把设定值送到示波器做对比
add_line(mdl, 'Setpoint/1', 'Scope/2');

% 反馈回路：从 Plant 输出拉回到 Error 的负输入端
add_line(mdl, 'Plant/1', 'Error/2');

% 开启信号记录：输出 y 和设定值 r
ph = get_param([mdl '/Plant'], 'PortHandles');
set_param(ph.Outport(1), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'y_out');
ph = get_param([mdl '/Setpoint'], 'PortHandles');
set_param(ph.Outport(1), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'r_ref');

%% ---------- 运行仿真 ----------
simOut = sim(mdl);

%% ---------- 绘图分析 ----------
y = simOut.logsout.getElement('y_out').Values;
r = simOut.logsout.getElement('r_ref').Values;

figure;
plot(r.Time, r.Data, 'k--', 'LineWidth', 1.5); hold on;
plot(y.Time, y.Data, 'b', 'LineWidth', 2); hold off;
legend('设定值', '实际输出', 'Location', 'southeast');
title('PID 闭环控制阶跃响应');
xlabel('时间 (s)'); ylabel('输出'); grid on;

fprintf('教程 05 完成！\n');
fprintf('动手实验：尝试修改 PID 的参数，观察响应变化\n');
fprintf('  - 只保留 P (I=0, D=0)：观察稳态误差\n');
fprintf('  - 增大 I：观察如何消除稳态误差（可能引起振荡）\n');
fprintf('  - 增大 D：观察如何抑制超调\n');
save_system(mdl, fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']));
fprintf('  - 试试积分饱和 (I 太大) 和微分噪声放大 (D 太大) 的问题\n');
