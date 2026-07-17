%% ============================================================
% 教程 03：一阶系统 — 传递函数与阶跃响应
% 目标：理解传递函数(Transfer Fcn)、阶跃信号(Step)、
%       以及时间常数对系统响应的影响
%
% 一阶系统标准形式：G(s) = K / (τs + 1)
%   K  — 稳态增益（steady-state gain）
%   τ  — 时间常数（time constant），越大响应越慢
% ============================================================

clear; close all;

mdl = 'tutorial03_first_order';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl, 'Model');
open_system(mdl);

%% ---------- 阶跃信号输入 ----------
add_block('simulink/Sources/Step', [mdl '/Step Input'], ...
    'Position', [50, 80, 110, 120]);
set_param([mdl '/Step Input'], 'Time', '1');

%% ---------- 三个不同时间常数的一阶系统 ----------
tau_values = [0.5, 2, 5];

for i = 1:3
    tau = tau_values(i);
    tauStr = strrep(num2str(tau), '.', 'p');
    blkName = ['TF_tau_' tauStr];

    add_block('simulink/Continuous/Transfer Fcn', [mdl '/' blkName], ...
        'Position', [200, 40 + i*90, 280, 80 + i*90]);
    set_param([mdl '/' blkName], ...
        'Numerator',   '[1]', ...
        'Denominator', ['[' num2str(tau) ' 1]']);

    add_line(mdl, 'Step Input/1', [blkName '/1']);
end

%% ---------- 示波器（多端口） ----------
add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [400, 120, 450, 160]);
set_param([mdl '/Scope'], 'NumInputPorts', '3');

for i = 1:3
    tau = tau_values(i);
    tauStr = strrep(num2str(tau), '.', 'p');
    blkName = ['TF_tau_' tauStr];
    add_line(mdl, [blkName '/1'], ['Scope/' num2str(i)]);

    % 开启每个传递函数输出的信号记录，便于在 MATLAB 中绘图
    ph = get_param([mdl '/' blkName], 'PortHandles');
    set_param(ph.Outport(1), 'DataLogging', 'on', ...
        'DataLoggingNameMode', 'Custom', 'DataLoggingName', blkName);
end

%% ---------- 运行仿真 ----------------
simOut = sim(mdl);

%% ---------- MATLAB 绘图对比 ----------
figure; hold on;
for i = 1:3
    tauStr = strrep(num2str(tau_values(i)), '.', 'p');
    data = simOut.logsout.getElement(['TF_tau_' tauStr]).Values;
    plot(data.Time, data.Data, 'LineWidth', 1.5);
end
hold off;
legend('\tau=0.5 (快)', '\tau=2 (中)', '\tau=5 (慢)', 'Location', 'southeast');
title('一阶系统阶跃响应：时间常数 \tau 的影响');
xlabel('时间 (s)'); ylabel('输出'); grid on;

fprintf('\n教程 03 完成！\n');
fprintf('观察：\tau 越小 → 系统响应越快，越早达到稳态值 1\n');
fprintf('稳态值 = K x 输入 = 1 x 1 = 1（三个系统最终都达到 1）\n');
