%% ============================================================
% 教程 28：数字信号处理基础 — 采样、频谱、滤波
% 目标：理解模数转换后的信号到底发生了什么
%
% 【为什么控制工程师需要 DSP？】
%   传感器给出的都是离散采样数据。你需要知道：
%   - 采样够不够快？（采样定理）
%   - 信号里有哪些频率？（FFT 频谱分析）
%   - 怎么去除噪声？（数字滤波）
%
% 【三个核心概念】
%   采样定理 (Nyquist)：采样率 fs ≥ 2 × 最高信号频率
%   FFT：把时域波形变成频谱 — 一眼看见"藏着的"频率分量
%   滤波：低通滤波去除高频噪声，高通滤波去除漂移
%
% 【本课内容】
%   1. 采样率对比：100Hz vs 25Hz vs 12Hz
%   2. FFT 频谱分析：从含噪信号中找出 50Hz + 120Hz
%   3. Simulink 模型：信号混合 + 低通滤波
%   4. 窗函数对比：矩形 / Hamming / Hann / Blackman
% ============================================================

clear; close all;

fprintf('============================================\n');
fprintf('  教程 28：数字信号处理基础\n');
fprintf('============================================\n\n');

%% ---------- 第 1 步：采样定理 — 采慢了会怎样 ----------
f_sig = 10;  % 信号频率 10 Hz
T = 2;
t_cont = 0:0.0001:T;                     % "连续"时间（0.1ms 步长当作连续）
y_cont = sin(2*pi*f_sig * t_cont);       % 原始 10Hz 正弦波

fprintf('【采样定理】Nyquist: fs ≥ 2 × fmax = %d Hz\n', 2*f_sig);
fprintf('  fs = 100Hz (10倍) — 完美重建，点很密\n');
fprintf('  fs =  25Hz (2.5倍) — 刚好够，能看出正弦形状\n');
fprintf('  fs =  12Hz (1.2倍) — 混叠！看起来像更低频率的信号\n\n');

figure('Name', 't28: 采样率对比', 'Position', [50, 50, 1000, 600]);
fs_rates = [100, 25, 12];
for i = 1:3
    subplot(3,1,i); hold on;
    plot(t_cont, y_cont, 'Color', [0.7, 0.7, 0.7], 'LineWidth', 0.5);
    t_s = 0:1/fs_rates(i):T;
    y_s = sin(2*pi*f_sig * t_s);
    stem(t_s, y_s, 'b', 'LineWidth', 1.5, 'MarkerSize', 4);
    title(sprintf('fs = %d Hz (Nyquist 倍数: %.1fx)', fs_rates(i), fs_rates(i)/(2*f_sig)));
    xlabel('时间 (s)'); ylabel('幅值'); grid on;
end

%% ---------- 第 2 步：FFT — 时域看不清，频域一目了然 ----------
fs2 = 1000;  dt = 1/fs2;  t2 = 0:dt:1-dt;
% 信号 = 50Hz 主力 + 120Hz 干扰 + 随机噪声
sig = sin(2*pi*50*t2) + 0.5*sin(2*pi*120*t2) + 0.3*randn(size(t2));

N_len = length(sig);
Y = fft(sig);                          % 快速傅里叶变换
P2 = abs(Y/N_len);                     % 双边谱
P1 = P2(1:N_len/2+1);                  % 单边谱
P1(2:end-1) = 2*P1(2:end-1);
f = fs2*(0:(N_len/2))/N_len;            % 频率轴

figure('Name', 't28: FFT 频谱分析', 'Position', [50, 50, 1000, 400]);
subplot(2,1,1);
plot(t2, sig);
title('时域波形 — 有噪声，看不出频率成分');
xlabel('时间 (s)'); ylabel('幅值'); grid on;

subplot(2,1,2);
plot(f, P1);  xlim([0, 200]);
title('频域 (FFT) — 两个尖峰！50Hz 和 120Hz');
xlabel('频率 (Hz)'); ylabel('|P(f)|'); grid on;

fprintf('【FFT 结果】峰值在 50Hz 和 120Hz — 噪声在时域掩盖了信号\n');
fprintf('  但 FFT 把噪声分散到全频带，信号尖峰清晰可见！\n\n');

%% ---------- 第 3 步：Simulink 模型 — 信号混合 + 滤波 ----------

mdl = 'tutorial28_dsp';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

% 50Hz 主力信号
add_block('simulink/Sources/Sine Wave', [mdl '/50Hz Signal'], ...
    'Position', [50, 60, 110, 90]);
set_param([mdl '/50Hz Signal'], 'Frequency', '50', 'Amplitude', '1');

% 120Hz 干扰信号
add_block('simulink/Sources/Sine Wave', [mdl '/120Hz Noise'], ...
    'Position', [50, 150, 110, 180]);
set_param([mdl '/120Hz Noise'], 'Frequency', '120', 'Amplitude', '0.5');

% 随机噪声
add_block('simulink/Sources/Band-Limited White Noise', [mdl '/Random Noise'], ...
    'Position', [50, 250, 110, 280]);

% 求和：三路信号混合
add_block('simulink/Math Operations/Sum', [mdl '/Signal Mixer'], ...
    'Position', [180, 140, 210, 220]);
set_param([mdl '/Signal Mixer'], 'Inputs', '|+++', 'IconShape', 'round');

add_line(mdl, '50Hz Signal/1', 'Signal Mixer/1');
add_line(mdl, '120Hz Noise/1', 'Signal Mixer/2');
add_line(mdl, 'Random Noise/1', 'Signal Mixer/3');

% 低通滤波器 — 截止频率约 100Hz，滤掉 120Hz 和部分噪声
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Low Pass Filter'], ...
    'Position', [300, 140, 390, 190]);
set_param([mdl '/Low Pass Filter'], ...
    'Numerator', '[10000]', ...
    'Denominator', '[1 200 10000]');    % ωc ≈ 100 rad/s

add_line(mdl, 'Signal Mixer/1', 'Low Pass Filter/1');

% 示波器：含噪信号 vs 滤波后
add_block('simulink/Sinks/Scope', [mdl '/Signal Scope'], ...
    'Position', [500, 130, 560, 210]);
set_param([mdl '/Signal Scope'], 'NumInputPorts', '2');

add_line(mdl, 'Signal Mixer/1', 'Signal Scope/1');        % 含噪
add_line(mdl, 'Low Pass Filter/1', 'Signal Scope/2');     % 滤波后

fprintf('【Simulink 模型】三路信号混合 → 低通滤波 → 对比示波器\n');
fprintf('  蓝色=含噪信号，红色=滤波后 — 120Hz 干扰应被削弱\n\n');

%% ---------- 第 4 步：窗函数 — FFT 的"镜头" ----------
Nw = 256;
w_rect  = ones(Nw, 1);          % 矩形窗 — 相当于没加窗
w_hamm  = hamming(Nw);          % Hamming — 最常用
w_hann  = hann(Nw);             % Hann
w_black = blackman(Nw);         % Blackman — 最平滑

figure('Name', 't28: 窗函数', 'Position', [50, 50, 800, 400]);
subplot(1,2,1); hold on;
plot(w_rect); plot(w_hamm, 'LineWidth', 1.5); plot(w_hann); plot(w_black);
legend('矩形', 'Hamming', 'Hann', 'Blackman', 'Location', 'best');
title('时域窗形状 — 两边逐渐减小，减少"截断效应"');
grid on;

subplot(1,2,2); hold on;
plot(20*log10(abs(fft(w_rect, 1024)) + eps));
plot(20*log10(abs(fft(w_hamm, 1024)) + eps), 'LineWidth', 1.5);
title('频域 — 旁瓣越低越好 (Hamming: -43dB)');
ylabel('幅度 (dB)'); grid on;

fprintf('【窗函数选择】\n');
fprintf('  矩形:   旁瓣 -13dB  — 不加窗，频谱泄漏最大\n');
fprintf('  Hamming: 旁瓣 -43dB  — 日常分析首选\n');
fprintf('  Blackman:旁瓣 -58dB  — 高动态范围信号\n\n');

fprintf('========================================\n');
fprintf('  教程 28 完成！\n');
fprintf('========================================\n\n');
fprintf('动手实验：\n');
fprintf('  1. 打开 tutorial28_dsp.slx，运行仿真\n');
fprintf('  2. 观察 Scope：低通滤波后 120Hz 分量被削弱\n');
fprintf('  3. 改 Low Pass Filter 的分母，试试不同的截止频率\n');
fprintf('  4. 在 MATLAB 里改 f_sig=50Hz，看 fs=12Hz 时混叠有多严重\n');

save_system(mdl, fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']));
close_system(mdl, 0);
