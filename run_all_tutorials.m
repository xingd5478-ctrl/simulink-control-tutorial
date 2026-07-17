function run_all_tutorials()
%RUN_ALL_TUTORIALS 一键依次运行全部 21 个教程
%
%   每课运行完毕后暂停，方便查看模型和截图；
%   所有 Figure 会同时自动保存为 PNG 到 docs/images/，
%   可直接用于 README 配图。
%
%   用法：
%     >> run_all_tutorials

tutorials = { ...
    't01_signal_basics'
    't02_math_blocks'
    't03_first_order_sys'
    't04_second_order_sys'
    't05_pid_control'
    't06_sources_and_sinks'
    't07_subsystem'
    't08_masking'
    't09_state_space'
    't10_state_feedback'
    't11_observer'
    't12_kalman_filter'
    't13_dc_motor'
    't14_foc_pmsm'
    't15_code_generation'
    't16_freq_domain'
    't17_lead_lag'
    't18_system_id'
    't19_robust_control'
    't20_mpc'
    't21_sliding_mode'};

outDir = fullfile(fileparts(mfilename('fullpath')), 'docs', 'images');
if ~exist(outDir, 'dir'), mkdir(outDir); end

n = numel(tutorials);
for i = 1:n
    name = tutorials{i};
    fprintf('\n################################################\n');
    fprintf('##  [%2d/%2d]  %s\n', i, n, name);
    fprintf('################################################\n\n');

    try
        run_one(name);
    catch ME
        warning('%s 运行失败: %s', name, ME.message);
    end

    nSaved = save_figures(name, outDir);
    fprintf('\n>>> %s 完成，%d 张图已保存到 docs/images/\n', name, nSaved);

    if i < n && ~batchStartupOptionUsed
        input('>>> 现在可以截图。按回车继续下一课...', 's');
    end

    close all;
    bdclose('all');   % 关闭本课模型，避免窗口越积越多
end

fprintf('\n================================================\n');
fprintf('  全部 %d 课运行完毕！所有图像在 docs/images/\n', n);
fprintf('================================================\n');
end

function run_one(name)
% 在独立工作区中运行教程脚本，
% 这样脚本里的 clear 不会破坏主循环的变量
run(name);
end

function nSaved = save_figures(name, outDir)
figs = flipud(findall(0, 'Type', 'figure'));   % 按创建顺序排列
nSaved = numel(figs);
for k = 1:nSaved
    file = fullfile(outDir, sprintf('%s_fig%d.png', name, k));
    try
        exportgraphics(figs(k), file, 'Resolution', 120);
    catch
        saveas(figs(k), file);   % 兼容旧版本 MATLAB
    end
end
end
