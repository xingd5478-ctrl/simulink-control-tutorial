%% ============================================================
% 教程 24：模糊控制 — 不靠数学模型，靠专家经验
%
% 【模糊控制是什么？】
%   传统 PID 需要被控对象的数学模型（传递函数/状态空间）。
%   模糊控制不需要精确模型——只需要专家的"经验规则"。
%   比如："如果温度偏高且上升快，就大幅减少加热功率"。
%
% 【模糊逻辑三步走】
%   1. 模糊化 (Fuzzification): 把精确数值（如"35°C"）→ 模糊量（"偏高"）
%   2. 推理 (Inference): 用 IF-THEN 规则表得出模糊结论
%   3. 解模糊 (Defuzzification): 把模糊结论 → 精确控制量（如"输出60%"）
%
% 【本课内容】
%   1. 手工设计一个两输入（误差+误差变化率）模糊 PI 控制器
%   2. Simulink 模型：模糊控制 vs PID 对比
%   3. 隶属度函数可视化
%   4. 控制曲面 — 模糊控制器的"调参地图"
% ============================================================

clear; close all;

fprintf('============================================\n');
fprintf('  教程 24：模糊逻辑控制\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：设计模糊推理系统 (FIS) =====

fprintf('【模糊控制器设计】\n');
fprintf('  输入1: 误差 e = ref - y\n');
fprintf('  输入2: 误差变化率 de/dt\n');
fprintf('  输出: 控制增量 Δu\n\n');

% 创建 Sugeno 型模糊系统（计算效率高，适合实时控制）
fis = sugfis('Name', 'Fuzzy_PI');

% --- 输入1: 误差 e ---
fis = addInput(fis, [-3, 3], 'Name', 'e');
fis = addMF(fis, 'e', 'gaussmf', [0.8, -3], 'Name', 'NB');   % Negative Big
fis = addMF(fis, 'e', 'gaussmf', [0.8, -1.5], 'Name', 'NS'); % Negative Small
fis = addMF(fis, 'e', 'gaussmf', [0.5, 0], 'Name', 'ZE');    % Zero
fis = addMF(fis, 'e', 'gaussmf', [0.8, 1.5], 'Name', 'PS');  % Positive Small
fis = addMF(fis, 'e', 'gaussmf', [0.8, 3], 'Name', 'PB');    % Positive Big

% --- 输入2: 误差变化率 de ---
fis = addInput(fis, [-6, 6], 'Name', 'de');
fis = addMF(fis, 'de', 'gaussmf', [1.5, -6], 'Name', 'NB');
fis = addMF(fis, 'de', 'gaussmf', [1.5, -3], 'Name', 'NS');
fis = addMF(fis, 'de', 'gaussmf', [1.0, 0], 'Name', 'ZE');
fis = addMF(fis, 'de', 'gaussmf', [1.5, 3], 'Name', 'PS');
fis = addMF(fis, 'de', 'gaussmf', [1.5, 6], 'Name', 'PB');

% --- 输出: 控制增量 Δu ---
fis = addOutput(fis, [-5, 5], 'Name', 'du');

% Sugeno 输出用常数
fis = addMF(fis, 'du', 'constant', -5, 'Name', 'NB');
fis = addMF(fis, 'du', 'constant', -2.5, 'Name', 'NS');
fis = addMF(fis, 'du', 'constant', 0, 'Name', 'ZE');
fis = addMF(fis, 'du', 'constant', 2.5, 'Name', 'PS');
fis = addMF(fis, 'du', 'constant', 5, 'Name', 'PB');

% --- 规则表 (5×5=25 条规则) ---
% 经典模糊 PI 规则：类似 PD 控制 + 积分效果
ruleList = [
    % e   de   du   weight  AND
    1 1 1 1 1;   % IF e=NB AND de=NB THEN du=NB  (负大误差 + 快速恶化 → 全力反向)
    1 2 1 1 1;   % IF e=NB AND de=NS THEN du=NB
    1 3 2 1 1;   % IF e=NB AND de=ZE THEN du=NS
    1 4 3 1 1;   % IF e=NB AND de=PS THEN du=ZE
    1 5 3 1 1;   % IF e=NB AND de=PB THEN du=ZE
    2 1 1 1 1;
    2 2 2 1 1;
    2 3 2 1 1;
    2 4 3 1 1;
    2 5 4 1 1;
    3 1 2 1 1;   % e=ZE...
    3 2 2 1 1;
    3 3 3 1 1;   % 误差为零且不变 → 不动
    3 4 4 1 1;
    3 5 4 1 1;
    4 1 3 1 1;
    4 2 3 1 1;
    4 3 4 1 1;
    4 4 4 1 1;
    4 5 5 1 1;
    5 1 3 1 1;   % e=PB...
    5 2 3 1 1;
    5 3 4 1 1;
    5 4 5 1 1;
    5 5 5 1 1;   % IF e=PB AND de=PB THEN du=PB (正大误差 + 快速恶化 → 全力正向)
    ];

fis = addRule(fis, ruleList);

fprintf('  规则条数: %d 条 (5误差 × 5变化率)\n', length(fis.Rules));
fprintf('  推理类型: Sugeno (高效)\n\n');

%% ===== 第 2 步：隶属度函数可视化 =====

figure('Name', 't24: 模糊控制器结构', ...
    'Position', [50, 50, 1000, 500]);

subplot(2,3,1);
plotmf(fis, 'input', 1); grid on;
title('输入1: 误差 e 的隶属度');

subplot(2,3,2);
plotmf(fis, 'input', 2); grid on;
title('输入2: 误差变化率 de 的隶属度');

subplot(2,3,3);
% 控制曲面 — 模糊控制器的"全部行为"
gensurf(fis); grid on;
title('控制曲面: u = f(e, de)');
xlabel('误差 e'); ylabel('变化率 de'); zlabel('Δu');

%% ===== 第 3 步：Simulink 模型 — 模糊 vs PID =====

mdl = 'tutorial24_fuzzy';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl, 'Model');
open_system(mdl);

% --- 被控对象（二阶系统）---
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Plant'], ...
    'Position', [400, 180, 490, 230]);
set_param([mdl '/Plant'], ...
    'Numerator', '[1]', 'Denominator', '[1 2 5]');

% --- 模糊控制路径 ---
add_block('simulink/Sources/Step', [mdl '/Step'], ...
    'Position', [50, 80, 100, 120]);
set_param([mdl '/Step'], 'Time', '0.5', 'After', '1');

add_block('simulink/Math Operations/Add', [mdl '/Sum Fuzzy'], ...
    'Position', [160, 85, 190, 115]);
set_param([mdl '/Sum Fuzzy'], 'Inputs', '|+-');

% 模糊逻辑控制器模块（使用我们设计的 FIS）
hasFuzzy = ~isempty(ver('fuzzy'));
if hasFuzzy
    try
        add_block('Fuzzy Logic Toolbox/Fuzzy Logic Controller', ...
            [mdl '/Fuzzy Controller'], 'Position', [260, 85, 310, 135]);
    catch
        try
            add_block('fuzblock/Fuzzy Logic Controller', ...
                [mdl '/Fuzzy Controller'], 'Position', [260, 85, 310, 135]);
        catch
            hasFuzzy = false;
        end
    end
end
if hasFuzzy
    set_param([mdl '/Fuzzy Controller'], 'FIS', 'fis');
else
    add_block('simulink/Math Operations/Gain', [mdl '/Fuzzy Controller'], ...
        'Position', [260, 85, 310, 135]);
    set_param([mdl '/Fuzzy Controller'], 'Gain', '1');
    fprintf('  [注意] Fuzzy Logic Toolbox 未安装，使用 Gain 替代模糊控制器\n');
end

% --- PID 对比路径（独立的 Plant 副本）---
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Plant PID'], ...
    'Position', [400, 280, 490, 330]);
set_param([mdl '/Plant PID'], ...
    'Numerator', '[1]', 'Denominator', '[1 2 5]');

add_block('simulink/Math Operations/Add', [mdl '/Sum PID'], ...
    'Position', [160, 230, 190, 260]);
set_param([mdl '/Sum PID'], 'Inputs', '|+-');

add_block('simulink/Continuous/PID Controller', [mdl '/PID'], ...
    'Position', [260, 230, 310, 290]);
set_param([mdl '/PID'], 'P', '3.0', 'I', '0.5', 'D', '1.0', 'N', '100');

% --- 示波器 ---
add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [570, 80, 620, 290]);
set_param([mdl '/Scope'], 'NumInputPorts', '3');

% --- 连线 ---
% 模糊路径
add_line(mdl, 'Step/1', 'Sum Fuzzy/1');
add_line(mdl, 'Sum Fuzzy/1', 'Fuzzy Controller/1');
add_line(mdl, 'Fuzzy Controller/1', 'Plant/1');
add_line(mdl, 'Plant/1', 'Scope/1');
add_line(mdl, 'Plant/1', 'Sum Fuzzy/2');

% PID 路径
add_line(mdl, 'Step/1', 'Sum PID/1');
add_line(mdl, 'Sum PID/1', 'PID/1');
add_line(mdl, 'PID/1', 'Plant PID/1');
add_line(mdl, 'Plant PID/1', 'Scope/2');
add_line(mdl, 'Plant PID/1', 'Sum PID/2');

% 参考信号
add_line(mdl, 'Step/1', 'Scope/3');

fprintf('【Simulink 模型已创建】tutorial24_fuzzy.slx\n');
fprintf('  模糊路径: 阶跃→求和→模糊控制器→被控对象→Scope[1]\n');
fprintf('  PID 路径: 阶跃→求和→PID→被控对象→Scope[2]\n');
fprintf('  对比两种控制器的跟踪性能！\n\n');

%% ===== 第 4 步：MATLAB 仿真对比 =====

% 开启信号记录
ph1 = get_param([mdl '/Plant'], 'PortHandles');
set_param(ph1.Outport(1), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'y_fuzzy');
ph2 = get_param([mdl '/Plant PID'], 'PortHandles');
set_param(ph2.Outport(1), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'y_pid');

set_param(mdl, 'StopTime', '15');
simOut = sim(mdl);

% 提取数据
y_fuzzy = simOut.logsout.getElement('y_fuzzy').Values;
y_pid   = simOut.logsout.getElement('y_pid').Values;
% 构造阶跃参考信号
r_time = y_fuzzy.Time;
r_data = double(r_time >= 0.5);

%% ===== 第 5 步：绘图对比 =====

figure('Name', 't24: 模糊控制 vs PID', ...
    'Position', [50, 50, 1000, 400]);

subplot(1,2,1); hold on;
plot(r_time, r_data, 'k--', 'LineWidth', 1.5);
plot(y_fuzzy.Time, y_fuzzy.Data, 'b', 'LineWidth', 2);
plot(y_pid.Time, y_pid.Data, 'r', 'LineWidth', 1.5);
legend('目标', '模糊控制', 'PID', 'Location', 'best');
title('阶跃响应对比');
xlabel('时间 (s)'); ylabel('输出'); grid on;

% 性能指标
S_f = stepinfo(y_fuzzy.Data, y_fuzzy.Time, 1);
S_p = stepinfo(y_pid.Data, y_pid.Time, 1);

subplot(1,2,2);
perf_data = [S_f.RiseTime, S_p.RiseTime;
             S_f.SettlingTime, S_p.SettlingTime;
             S_f.Overshoot, S_p.Overshoot];
bar(perf_data);
set(gca, 'XTickLabel', {'上升时间(s)', '调节时间(s)', '超调量(%)'});
legend('模糊控制', 'PID', 'Location', 'northwest');
title('性能指标对比'); grid on;

fprintf('\n【性能对比】\n');
fprintf('           模糊控制    PID\n');
fprintf('  上升时间:  %.2f s     %.2f s\n', S_f.RiseTime, S_p.RiseTime);
fprintf('  调节时间:  %.2f s     %.2f s\n', S_f.SettlingTime, S_p.SettlingTime);
fprintf('  超调量:    %.1f%%      %.1f%%\n', S_f.Overshoot, S_p.Overshoot);

fprintf('\n========================================\n');
fprintf('  教程 24 完成！\n');
fprintf('========================================\n\n');

fprintf('【模糊控制 vs PID】\n');
fprintf('  PID: 需要模型 → 调参困难 → 但对线性系统效果好\n');
fprintf('  模糊: 不需要模型 → 靠经验规则 → 非线性系统表现好\n\n');

fprintf('【什么时候用模糊控制？】\n');
fprintf('  ✓ 系统太复杂，建模困难（化工过程、发酵、空调）\n');
fprintf('  ✓ 有熟练操作工的经验可以编码成规则\n');
fprintf('  ✓ 非线性强、PID 在大范围内表现不好\n');
fprintf('  ✗ 精度要求极高 → 模糊不如精确模型\n');
fprintf('  ✗ 变化太快 → Sugeno 型可以，Mamdani 型太慢\n\n');

fprintf('→ 双击 Fuzzy Controller 模块查看规则\n');
fprintf('→ 在 MATLAB 中输入 fuzzy(fis) 打开规则编辑器\n');
fprintf('→ 试着修改规则表，观察控制曲面的变化\n');

save_system(mdl, fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']));
