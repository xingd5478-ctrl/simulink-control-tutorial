function data = getSimData(simOut, varName, t)
% GETSIMDATA  从 Simulink 仿真输出中提取数据，兼容多种存储格式
%
%   data = getSimData(simOut, varName, t)
%
%   输入:
%     simOut   — Simulink.SimulationOutput 对象
%     varName  — To Workspace 模块的变量名字符串
%     t        — 期望的时间向量（用于插值对齐）
%
%   输出:
%     data     — 插值到与 t 等长的数据向量
%
%   支持格式:
%     - struct with .signals.values / .time
%     - timeseries
%     - Simulink.SimulationData.Dataset
%     - 普通数值数组
%
%   查找顺序: base workspace → simOut.get() → simOut.logsout.get()

    val = [];

    % 方法1：从 base workspace
    try
        val = evalin('base', varName);
    catch
    end

    % 方法2：从 simOut.get()
    if isempty(val)
        try
            val = simOut.get(varName);
        catch
        end
    end

    % 方法3：从 simOut.logsout
    if isempty(val)
        try
            val = simOut.logsout.get(varName);
        catch
        end
    end

    if isempty(val)
        fprintf('  [WARN] getSimData: 变量 ''%s'' 未找到\n', varName);
        data = zeros(length(t), 1);
        return;
    end

    % 提取数值数据
    if isstruct(val) && isfield(val, 'signals')
        data = val.signals.values;
        if length(data) ~= length(t)
            data = interp1(val.time, data, t, 'linear', 'extrap');
        end
    elseif isa(val, 'timeseries')
        data = val.Data;
        if length(data) ~= length(t)
            data = interp1(val.Time, data, t, 'linear', 'extrap');
        end
    elseif isa(val, 'Simulink.SimulationData.Dataset')
        try
            el = val.getElement(1);
            data = el.Values.Data;
            if length(data) ~= length(t)
                data = interp1(el.Values.Time, data, t, 'linear', 'extrap');
            end
        catch
            data = zeros(length(t), 1);
        end
    elseif isnumeric(val)
        data = val;
        if length(data) ~= length(t)
            data = interp1(linspace(0, t(end), length(data))', data, t, 'linear', 'extrap');
        end
    else
        data = zeros(length(t), 1);
    end

    % 确保列向量
    data = data(:);
end
