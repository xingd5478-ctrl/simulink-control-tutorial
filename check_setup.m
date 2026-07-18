%% ============================================================
% check_setup — 检查 MATLAB 环境是否满足教程要求
%
% 如果你是第一次使用本项目，请先运行这个脚本：
%   >> check_setup
%
% 它会检查你的 MATLAB 版本、必要的工具箱是否安装、
% 以及项目路径是否正确配置。
% ============================================================

fprintf('============================================\n');
fprintf('  Simulink 教程 — 环境检查\n');
fprintf('============================================\n\n');

all_ok = true;
toolbox_status = struct();

%% 1. MATLAB 版本检查
v = ver('matlab');
fprintf('[1] MATLAB 版本: %s (R%s)\n', v.Release, v.Version);
% MATLAB R2020a = Version 9.8, check by version number
major_ver = str2double(v.Version);
if major_ver < 9.8  % Pre-R2020a
    fprintf('    ⚠ 版本较旧，建议升级到 R2020a 或更高\n');
else
    fprintf('    ✓ 版本满足要求 (>= R2020a)\n');
end
fprintf('\n');

%% 2. 必备工具箱检查
required = {'Simulink', 'Control System Toolbox'};
optional = {'Robust Control Toolbox', 'Model Predictive Control Toolbox', ...
            'Fuzzy Logic Toolbox', 'Simscape', 'Simulink Coder'};

fprintf('[2] 必备工具箱:\n');
for i = 1:length(required)
    info = ver(required{i});
    if isempty(info)
        fprintf('    ✗ %s — 未安装！\n', required{i});
        all_ok = false;
    else
        fprintf('    ✓ %s (v%s)\n', required{i}, info.Version);
    end
end

fprintf('\n[3] 可选工具箱 (缺少也不影响大部分教程):\n');
for i = 1:length(optional)
    info = ver(optional{i});
    if isempty(info)
        fprintf('    - %s — 未安装 (t19-t20, t24-t25 部分功能受限)\n', optional{i});
    else
        fprintf('    ✓ %s (v%s)\n', optional{i}, info.Version);
    end
end
fprintf('\n');

%% 3. 项目路径检查
project_root = fileparts(mfilename('fullpath'));
fprintf('[4] 项目路径: %s\n', project_root);

% 检查关键目录
required_dirs = {'models', 'utils', 'docs/images'};
for i = 1:length(required_dirs)
    d = fullfile(project_root, required_dirs{i});
    if exist(d, 'dir')
        fprintf('    ✓ %s/\n', required_dirs{i});
    else
        fprintf('    ✗ %s/ — 缺失！\n', required_dirs{i});
        all_ok = false;
    end
end

% 检查模型目录是否在路径中
model_path = fullfile(project_root, 'models');
path_cells = strsplit(path, pathsep);
if ~any(strcmp(path_cells, model_path))
    fprintf('    → models/ 未加入 MATLAB 路径，自动添加...\n');
    addpath(model_path);
end
fprintf('\n');

%% 4. 总结
fprintf('============================================\n');
if all_ok
    fprintf('  ✓ 环境检查通过！可以开始学习。\n');
    fprintf('\n');
    fprintf('  下一步：\n');
    fprintf('    1. 运行 >> t00_main_guide  查看课程总览\n');
    fprintf('    2. 运行 >> t01_signal_basics  开始第一课\n');
else
    fprintf('  ✗ 环境检查未通过，请先安装缺失的必备工具箱。\n');
    fprintf('\n');
    fprintf('  MATLAB 工具箱安装方法：\n');
    fprintf('    1. 打开 MATLAB\n');
    fprintf('    2. 点击"附加功能" → "获取附加功能"\n');
    fprintf('    3. 搜索并安装缺失的工具箱\n');
end
fprintf('============================================\n');
