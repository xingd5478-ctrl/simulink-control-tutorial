%% ============================================================
% 教程 22：根轨迹法 — 极点走向决定系统行为
%
% 【什么是根轨迹？】
%   开环增益 K 从 0 → ∞ 变化时，闭环极点在复平面上的移动轨迹。
%   一条根轨迹曲线 = 系统所有可能的"性格"的集合。
%
% 【为什么需要根轨迹？】
%   时域（t03-t04）：看具体响应曲线 → 定性感觉
%   频域（t16）：看 Bode/Nyquist → 稳定性裕度
%   根轨迹：看极点位置 → 直接连接时域性能指标
%     - 极点靠左 → 快（衰减快）
%     - 极点上下 → 振荡（有虚部）
%     - 极点跑到右半平面 → 不稳定！
%
% 【本课内容】
%   1. 手画根轨迹的基本法则
%   2. Simulink 模型：可变增益反馈闭环
%   3. MATLAB rlocus() 自动画轨迹
%   4. 用根轨迹设计 Lead 补偿器
% ============================================================

clear; close all;

fprintf('============================================\n');
fprintf('  教程 22：根轨迹法 — 极点走向决定行为\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：构建示例系统 =====

% 三阶系统（用二阶的话根轨迹太简单，看不出效果）
sys = tf([1], [1, 6, 11, 6]);  % G(s) = 1/((s+1)(s+2)(s+3))

[z, p, k] = zpkdata(sys, 'v');
fprintf('【开环系统】\n');
fprintf('  极点: ');
fprintf('%.1f ', p);
fprintf('\n  零点: 无（全极系统）\n\n');

%% ===== 第 2 步：根轨迹分析 =====

figure('Name', 't22: 根轨迹', 'Position', [50, 50, 900, 500]);

subplot(1,2,1);
rlocus(sys); grid on;
title('根轨迹图');
sgrid(0.7, []);  % 叠加 ζ=0.7 阻尼线

fprintf('【根轨迹阅读指南】\n');
fprintf('  × = 开环极点 (K=0 时的闭环极点)\n');
fprintf('  ○ = 开环零点 (K→∞ 时的闭环极点)\n');
fprintf('  每条曲线 = 一个极点随 K 增大的运动轨迹\n');
fprintf('  蓝色实线: 阻尼比 ζ=0.7 线\n');
fprintf('  → 轨迹与 ζ=0.7 线的交点 = 阻尼 0.7 时的 K 值\n\n');

%% ===== 第 3 步：系统根轨迹 =====

sys2 = tf([1, 2], [1, 4, 8, 0]);  % 带零点和积分器的系统

subplot(1,2,2);
rlocus(sys2); grid on;
sgrid(0.7, 3);  % ζ=0.7 线 + ωn=3 圆
title('带零点系统：根轨迹 + 设计约束');

fprintf('【根轨迹设计思路】\n');
fprintf('  1. 画出根轨迹 → 看极点"能去哪"\n');
fprintf('  2. 叠加 sgrid → 标注目标区域（如 ζ>0.7, ωn>3）\n');
fprintf('  3. 如果轨迹不进目标区域 → 需要加补偿器"弯折"轨迹\n');
fprintf('  4. 加零点 → 轨迹向左弯（增加稳定性）\n');
fprintf('  5. 加极点 → 轨迹向右弯（减少稳定性）\n\n');

%% ===== 第 4 步：Simulink 模型 — 可变增益反馈 =====

mdl = 'tutorial22_root_locus';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl, 'Model');
open_system(mdl);

% 阶跃输入
add_block('simulink/Sources/Step', [mdl '/Step'], ...
    'Position', [50, 80, 100, 120]);
set_param([mdl '/Step'], 'Time', '0.5');

% 求和点（误差 = 参考 - 反馈）
add_block('simulink/Math Operations/Add', [mdl '/Sum'], ...
    'Position', [170, 85, 200, 115]);
set_param([mdl '/Sum'], 'Inputs', '|+-');

% 可变增益 K
add_block('simulink/Math Operations/Gain', [mdl '/Gain_K'], ...
    'Position', [280, 90, 330, 130]);
set_param([mdl '/Gain_K'], 'Gain', '5');  % 初始 K=5

% 三阶系统
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Plant'], ...
    'Position', [420, 85, 510, 135]);
set_param([mdl '/Plant'], ...
    'Numerator', '[1]', 'Denominator', '[1 6 11 6]');

% 示波器
add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [590, 85, 640, 135]);

% 连线
add_line(mdl, 'Step/1', 'Sum/1');
add_line(mdl, 'Sum/1', 'Gain_K/1');
add_line(mdl, 'Gain_K/1', 'Plant/1');
add_line(mdl, 'Plant/1', 'Scope/1');
add_line(mdl, 'Plant/1', 'Sum/2');  % 反馈

% 开启信号记录（用于 MATLAB 画图）
ph = get_param([mdl '/Plant'], 'PortHandles');
set_param(ph.Outport(1), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'y_out');

fprintf('【Simulink 模型已创建】tutorial22_root_locus.slx\n');
fprintf('  结构: 阶跃→求和→增益K→三阶系统→示波器\n');
fprintf('         ↑___________反馈___________↓\n');
fprintf('  试着改 Gain_K 的值，观察响应变化！\n\n');

%% ===== 第 5 步：不同 K 值对比仿真 =====

K_values = [1, 5, 15, 30];

figure('Name', 't22: K值对比', 'Position', [50, 50, 1000, 400]);

subplot(1,2,1);
rlocus(sys); hold on; grid on;
sgrid(0.7, []);
colors = lines(length(K_values));

for i = 1:length(K_values)
    K = K_values(i);

    % 设置 Simulink 模型增益
    set_param([mdl '/Gain_K'], 'Gain', num2str(K));
    simOut = sim(mdl);
    y = simOut.logsout.getElement('y_out').Values;

    % 画阶跃响应
    subplot(1,2,2); hold on;
    plot(y.Time, y.Data, 'Color', colors(i,:), 'LineWidth', 1.5);

    % 在根轨迹上标注对应极点
    cl_poles = feedback(K*sys, 1);
    cl_p = pole(cl_poles);
    subplot(1,2,1);
    plot(real(cl_p), imag(cl_p), 'o', 'Color', colors(i,:), ...
        'MarkerSize', 10, 'LineWidth', 2);
    if imag(cl_p(1)) ~= 0
        plot(real(cl_p), -imag(cl_p), 'o', 'Color', colors(i,:), ...
            'MarkerSize', 10, 'LineWidth', 2);
    end
end

subplot(1,2,1); hold off;
legend(['开环极点'; arrayfun(@(k) sprintf('K=%d', k), K_values, ...
    'UniformOutput', false)'], 'Location', 'best');
title('根轨迹 + 对应极点位置');

subplot(1,2,2); hold off;
legend(arrayfun(@(k) sprintf('K=%d', k), K_values, ...
    'UniformOutput', false), 'Location', 'best');
title('不同增益 K 的阶跃响应');
xlabel('时间 (s)'); ylabel('输出'); grid on;

%% ===== 第 6 步：根轨迹设计补偿器 =====

figure('Name', 't22: 补偿器设计', 'Position', [50, 50, 900, 400]);

subplot(1,2,1);
rlocus(sys); hold on; grid on;
sgrid(0.7, 3);
title('目标：ζ>0.7, ωn>3 的区域');
% 原始根轨迹偏右 → 需要 Lead 补偿器把轨迹往左拉

% 加一个零点 s=-2（把极点往左吸）
C_lead = tf([1, 2], 1);  % 零点在 -2
sys_comp = C_lead * sys;
subplot(1,2,2);
rlocus(sys_comp); grid on;
sgrid(0.7, 3);
title('加零点 s=-2 后：根轨迹被"吸"向左');
hold off;

fprintf('【补偿器设计演示】\n');
fprintf('  原始系统: 根轨迹偏右 → 响应慢\n');
fprintf('  加零点 s=-2: 轨迹向左弯 → 更快更稳定\n');
fprintf('  这就是 Lead 补偿器的根轨迹解释！\n\n');

fprintf('========================================\n');
fprintf('  教程 22 完成！\n');
fprintf('========================================\n\n');

fprintf('【根轨迹 vs 频域 vs 时域】\n');
fprintf('  时域(t03-t05): "响应长什么样" → 直观但定性\n');
fprintf('  频域(t16): "稳定裕度多少" → 量化但间接\n');
fprintf('  根轨迹(t22): "极点在复平面的位置" → 直接连接时域和频域\n');
fprintf('  三者构成经典控制理论的完整工具链！\n\n');

fprintf('→ 现在打开 tutorial22_root_locus.slx\n');
fprintf('→ 双击 Gain_K 模块，改 K=2,5,10,20\n');
fprintf('→ 每次运行仿真，观察阶跃响应和根轨迹上极点的对应关系\n');

% 保存模型
save_system(mdl, fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']));
