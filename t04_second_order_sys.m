
%% ============================================================
% 教程 04：二阶系统 — 质量-弹簧-阻尼与阻尼比
% 目标：理解二阶系统的动态特性，包括
%       - 欠阻尼（振荡衰减）
%       - 临界阻尼（最快无超调）
%       - 过阻尼（缓慢无超调）
%
% 标准形式：G(s) = ωn² / (s² + 2ζωn·s + ωn²)
%   ζ  — 阻尼比（damping ratio）
%   ωn — 自然频率（natural frequency）
% ============================================================

clear; close all;

mdl = 'tutorial04_second_order';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl, 'Model');
open_system(mdl);

%% ---------- 阶跃输入 ----------
add_block('simulink/Sources/Step', [mdl '/Step'], ...
    'Position', [50, 200, 110, 240]);

%% ---------- 三种阻尼比的二阶系统 ----------
wn = 2;  % 固定自然频率为 2 rad/s
zeta_values = [0.2, 1.0, 2.0];  % 欠阻尼、临界阻尼、过阻尼
zeta_names  = {'Underdamped \zeta=0.2', ...
               'Critically Damped \zeta=1.0', ...
               'Overdamped \zeta=2.0'};

for i = 1:3
    z = zeta_values(i);

    % 传递函数: wn^2 / (s^2 + 2*z*wn*s + wn^2)
    num = wn^2;
    den = [1, 2*z*wn, wn^2];

    blkName = sprintf('TF_zeta%.1f', z);

    add_block('simulink/Continuous/Transfer Fcn', [mdl '/' blkName], ...
        'Position', [180, 70 + i*120, 270, 110 + i*120]);
    set_param([mdl '/' blkName], ...
        'Numerator',   mat2str(num), ...
        'Denominator', mat2str(den));

    add_line(mdl, 'Step/1', [blkName '/1']);
end

%% ---------- 多通道示波器 ----------
add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [400, 200, 450, 240]);
set_param([mdl '/Scope'], 'NumInputPorts', '3');

for i = 1:3
    z = zeta_values(i);
    blkName = sprintf('TF_zeta%.1f', z);
    add_line(mdl, [blkName '/1'], ['Scope/' num2str(i)]);

    % 开启信号记录，便于在 MATLAB 中绘图
    ph = get_param([mdl '/' blkName], 'PortHandles');
    set_param(ph.Outport(1), 'DataLogging', 'on', ...
        'DataLoggingNameMode', 'Custom', 'DataLoggingName', blkName);
end

%% ---------- 运行仿真 ----------
simOut = sim(mdl);

%% ---------- MATLAB 绘图 ----------
figure; hold on;
styles = {'-', '--', ':'};
for i = 1:3
    blkName = sprintf('TF_zeta%.1f', zeta_values(i));
    data = simOut.logsout.getElement(blkName).Values;
    plot(data.Time, data.Data, styles{i}, 'LineWidth', 2);
end
hold off;
legend(zeta_names, 'Location', 'northeast');
title('二阶系统阶跃响应：阻尼比 \zeta 的影响');
xlabel('时间 (s)'); ylabel('输出'); grid on;

fprintf('教程 04 完成！\n');
fprintf('关键观察：\n');
fprintf('  欠阻尼 (\x03B6=0.2): 有明显超调，振荡衰减 → 适合快速响应的场景\n');
fprintf('  临界阻尼 (\x03B6=1.0): 无超调且上升最快 → 理论最优\n');
fprintf('  过阻尼 (\x03B6=2.0): 无超调但上升缓慢 → 响应过于迟钝\n');
