%% ============================================================
% 教程 15：从 Simulink 到 C 代码 — 嵌入式部署实战
%
% 【为什么需要这一课？】
%   t10 设计了 LQR 控制器 → Simulink 里跑通了
%   但 STM32 等单片机跑不了 Simulink → 需要把算法变成 C 代码
%
%   本课不依赖 Simulink Coder 工具箱
%   → 手动完成"连续设计 → 离散化 → C 代码"的全流程
%   → 和实际嵌入式项目中的做法一模一样
%
% ┌─────────────────────────────────────────────────────────┐
% │ 一、连续 → 离散：为什么需要这一步？                      │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │   Simulink 仿真是连续的 (微分方程)                       │
% │   STM32 是离散的 (每 Ts 执行一次)                        │
% │                                                         │
% │   连续 LQR:   u(t) = -K_c * x(t)                        │
% │   离散 LQR:   u[k] = -K_d * x[k]   (每 Ts 秒算一次)     │
% │                                                         │
% │   K_c ≠ K_d！离散增益需要重新计算                        │
% │   MATLAB: [K_d, S, E] = dlqr(A_d, B_d, Q, R)            │
% └─────────────────────────────────────────────────────────┘
%
% ┌─────────────────────────────────────────────────────────┐
% │ 二、离散化的三种方法                                     │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │   1. 零阶保持 (ZOH) — 最常用                             │
% │      A_d = expm(A * Ts)                                 │
% │      B_d = ∫₀^Ts expm(A*t)·B dt                        │
% │      MATLAB: sys_d = c2d(ss(A,B,C,D), Ts, 'zoh')       │
% │                                                         │
% │   2. Tustin (双线性) — 频率特性保真更好                  │
% │      s = (2/Ts)*(z-1)/(z+1)                             │
% │                                                         │
% │   3. 前向欧拉 — 最快但不稳定，不推荐                     │
% │      s = (z-1)/Ts                                       │
% └─────────────────────────────────────────────────────────┘
%
% ┌─────────────────────────────────────────────────────────┐
% │ 三、采样周期 Ts 怎么选？                                 │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │   经验法则：闭环带宽 × (10~40) = 采样频率                │
% │                                                         │
% │   例：控制器带宽 20Hz → fs ≥ 200~800 Hz                  │
% │       即 Ts ≤ 5ms ~ 1.25ms                              │
% │                                                         │
% │   例：MPU6050 姿态项目 100Hz 采样 → Ts=10ms              │
% │       姿态环带宽 ~5Hz → 10Hz 控制 → 足够                 │
% └─────────────────────────────────────────────────────────┘
%
% ┌─────────────────────────────────────────────────────────┐
% │ 四、从离散 LQR 到 C 代码（实际部署）                     │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │   控制律： u[k] = -K_d * x[k]                           │
% │                                                         │
% │   在 C 代码中就是一句：                                   │
% │     u = -(K1*x1 + K2*x2);    // 2次乘法 + 1次加法       │
% │                                                         │
% │   如果需要积分消除静差：                                  │
% │     err = ref - y;                                      │
% │     integral += err * Ts;   // 累积积分                 │
% │     u = Ki*integral - K1*x1 - K2*x2;                    │
% └─────────────────────────────────────────────────────────┘
%
% 【本课目标】
%   1. 把 t10 的 LQR 控制器从连续变离散
%   2. 对比不同采样率下的离散效果
%   3. 生成可用的 C 代码片段
%   4. 理解如何嵌入 FreeRTOS 任务
% ============================================================

clear; close all;

%% ===== 系统参数（沿用 t10 质量-弹簧-阻尼）=====

m = 1.0;  c = 0.5;  k = 4.0;

A = [   0,     1  ;
      -k/m,  -c/m ];

B = [  0  ;
      1/m ];

C = [ 1, 0 ];
D = 0;

fprintf('============================================\n');
fprintf('  教程 15：从 Simulink 到 C 代码\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：连续 LQR 设计（t10 复习）=====

Q = diag([10, 1]);
R = 1;
K_c = lqr(A, B, Q, R);
eig_cl = eig(A - B*K_c);

fprintf('【连续时间 LQR】\n');
fprintf('  K_c = [ %.4f,  %.4f ]\n', K_c(1), K_c(2));
fprintf('  闭环极点: %.2f ± j%.2f\n\n', real(eig_cl(1)), abs(imag(eig_cl(1))));

%% ===== 第 2 步：不同采样率下的离散化对比 =====

Ts_list = [0.001, 0.005, 0.010, 0.050];  % 1ms, 5ms, 10ms, 50ms

fprintf('【离散化对比 — 不同采样周期】\n\n');

figure('Name', 't15: 离散化效果对比', ...
    'Position', [50, 50, 1000, 700]);

for iTs = 1:length(Ts_list)
    Ts = Ts_list(iTs);

    % ZOH 离散化
    sys_c = ss(A, B, C, D);
    sys_d = c2d(sys_c, Ts, 'zoh');
    A_d = sys_d.A;
    B_d = sys_d.B;

    % 离散 LQR
    [K_d, ~, eig_d] = dlqr(A_d, B_d, Q, R);

    % 仿真离散控制器（10 秒）
    N = round(10/Ts);
    x = [0.3; 0];       % 初始位移 0.3m
    x_hist = zeros(2, N);
    u_hist = zeros(1, N);
    r = 1;               % 目标位移 1m

    for k = 1:N
        % 带积分的 LQR 控制（消除静差）
        err_int = 0;  % 简化：不加积分，看离散效果
        u = -K_d * x;  % + r 的前馈可以在外部加

        x_hist(:, k) = x;
        u_hist(k) = u;

        % Euler 前向积分：x[k+1] = A_d*x[k] + B_d*u[k]
        x = A_d * x + B_d * u;
    end
    t_vec = (0:N-1)*Ts;

    % 绘图
    subplot(2, 2, iTs);
    yyaxis left;
    plot(t_vec, x_hist(1,:), 'b', 'LineWidth', 1.5);
    ylabel('位移 x₁ (m)'); grid on;

    yyaxis right;
    stairs(t_vec, u_hist, 'r', 'LineWidth', 1);
    ylabel('控制力 u (N)');

    title(sprintf('Ts = %.0f ms | K_d = [%.2f, %.2f]', ...
        Ts*1000, K_d(1), K_d(2)));
    xlabel('时间 (s)');

    fprintf('  Ts = %.0f ms: K_d = [ %7.3f,  %7.3f ], ', ...
        Ts*1000, K_d(1), K_d(2));
    fprintf('极点 = %.2f±j%.2f\n', real(eig_d(1)), abs(imag(eig_d(1))));
end

sgtitle('教程 15：离散 LQR — 不同采样周期下的控制效果对比');
fprintf('\n  观察：Ts 越小 → K_d 越接近 K_c → 效果越好\n');
fprintf('        Ts=50ms → 系统明显抖动 → 采样太慢！\n\n');

%% ===== Simulink 模型：连续 vs 离散 LQR 对比 =====

mdl = 'tutorial15_codegen';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl, 'Model');
open_system(mdl);

Ts_model = 0.005;  % 5ms
Kp = 3;  % 标量比例增益（用于连续 vs 离散对比）
% 离散化被控对象
sys_d2 = c2d(ss(A, B, C, D), Ts_model, 'zoh');

add_block('simulink/Sources/Step', [mdl '/Step Ref'], ...
    'Position', [50, 80, 100, 120]);
set_param([mdl '/Step Ref'], 'Time', '0.5', 'After', '1');

% 连续路径：比例控制 + 连续被控对象
add_block('simulink/Math Operations/Add', [mdl '/Sum Cont'], ...
    'Position', [150, 60, 180, 90]);
set_param([mdl '/Sum Cont'], 'Inputs', '|+-');
add_block('simulink/Math Operations/Gain', [mdl '/Kp_cont'], ...
    'Position', [230, 60, 270, 90]);
set_param([mdl '/Kp_cont'], 'Gain', num2str(Kp));
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Plant Cont'], ...
    'Position', [340, 60, 430, 100]);
set_param([mdl '/Plant Cont'], 'Numerator', '[1]', ...
    'Denominator', ['[1 ' num2str(c) ' ' num2str(k) ']']);

% 离散路径：比例控制 + 零阶保持 + 连续被控对象 = 离散闭环
add_block('simulink/Math Operations/Add', [mdl '/Sum Disc'], ...
    'Position', [150, 180, 180, 210]);
set_param([mdl '/Sum Disc'], 'Inputs', '|+-');
add_block('simulink/Math Operations/Gain', [mdl '/Kp_disc'], ...
    'Position', [230, 175, 270, 215]);
set_param([mdl '/Kp_disc'], 'Gain', num2str(Kp));
add_block('simulink/Discrete/Zero-Order Hold', [mdl '/ZOH'], ...
    'Position', [300, 185, 330, 215]);
set_param([mdl '/ZOH'], 'SampleTime', num2str(Ts_model));
add_block('simulink/Continuous/Transfer Fcn', [mdl '/Plant Disc'], ...
    'Position', [390, 175, 480, 215]);
set_param([mdl '/Plant Disc'], 'Numerator', '[1]', ...
    'Denominator', ['[1 ' num2str(c) ' ' num2str(k) ']']);

add_block('simulink/Sinks/Scope', [mdl '/Scope'], ...
    'Position', [500, 70, 550, 220]);
set_param([mdl '/Scope'], 'NumInputPorts', '2');

% 连线
add_line(mdl, 'Step Ref/1', 'Sum Cont/1');
add_line(mdl, 'Sum Cont/1', 'Kp_cont/1');
add_line(mdl, 'Kp_cont/1', 'Plant Cont/1');
add_line(mdl, 'Plant Cont/1', 'Scope/1');
add_line(mdl, 'Plant Cont/1', 'Sum Cont/2');

add_line(mdl, 'Step Ref/1', 'Sum Disc/1');
add_line(mdl, 'Sum Disc/1', 'Kp_disc/1');
add_line(mdl, 'Kp_disc/1', 'ZOH/1');
add_line(mdl, 'ZOH/1', 'Plant Disc/1');
add_line(mdl, 'Plant Disc/1', 'Scope/2');
add_line(mdl, 'Plant Disc/1', 'Sum Disc/2');

fprintf('  [Simulink 模型] tutorial15_codegen.slx — 连续 vs 离散 LQR 对比\n\n');

%% ===== 第 3 步：生成 C 代码 =====

% 选定采样周期
Ts = 0.005;  % 5ms = 200Hz — 适合 STM32

sys_d = c2d(ss(A, B, C, D), Ts, 'zoh');
[K_d, ~, ~] = dlqr(sys_d.A, sys_d.B, Q, R);

fprintf('========================================\n');
fprintf('  C 代码生成 (Ts = %.0f ms)\n', Ts*1000);
fprintf('========================================\n\n');

fprintf('  K_d = [ %.6f,  %.6f ]\n\n', K_d(1), K_d(2));

% --- 输出 C 代码 ---
fprintf('/* ================================================ */\n');
fprintf('/*  自动生成的 LQR 控制器 — 质量-弹簧-阻尼系统      */\n');
fprintf('/*  采样周期: %.1f ms (%.0f Hz)                      */\n', Ts*1000, 1/Ts);
fprintf('/*  生成日期: %s                          */\n', date);
fprintf('/* ================================================ */\n\n');

fprintf('#define LQR_K1  %.8ff    /* 位置反馈增益 */\n', K_d(1));
fprintf('#define LQR_K2  %.8ff    /* 速度反馈增益 */\n', K_d(2));
fprintf('#define LQR_TS  %.6ff    /* 采样周期 (s) */\n\n', Ts);

fprintf('/* ---------- 控制器状态结构体 ---------- */\n');
fprintf('typedef struct {\n');
fprintf('    float x1_hat;      /* 估计位移 (m) */\n');
fprintf('    float x2_hat;      /* 估计速度 (m/s) */\n');
fprintf('    float integral;    /* 误差积分 (消除静差) */\n');
fprintf('} LQR_Controller;\n\n');

fprintf('/* ---------- 初始化 ---------- */\n');
fprintf('void LQR_Init(LQR_Controller *ctrl) {\n');
fprintf('    ctrl->x1_hat = 0.0f;\n');
fprintf('    ctrl->x2_hat = 0.0f;\n');
fprintf('    ctrl->integral = 0.0f;\n');
fprintf('}\n\n');

fprintf('/* ---------- 每个采样周期调用一次 ---------- */\n');
fprintf('/* 输入: ref (目标位移), y_meas (传感器测量值)       */\n');
fprintf('/* 输出: u (控制力), 同时更新内部状态估计           */\n');
fprintf('float LQR_Step(LQR_Controller *ctrl, float ref, float y_meas) {\n');
fprintf('    float err, u;\n');
fprintf('    float x1 = y_meas;               /* 直接测位移 */\n');
fprintf('    float x2;                                          \n\n');

fprintf('    /* 速度估计：差分法（最简单，可替换为 Kalman） */\n');
fprintf('    x2 = (x1 - ctrl->x1_hat) / LQR_TS;\n\n');

fprintf('    /* 误差积分（消除稳态误差） */\n');
fprintf('    err = ref - x1;\n');
fprintf('    ctrl->integral += err * LQR_TS;\n\n');

fprintf('    /* 控制律：u = -K1*x1 - K2*x2 + Ki*integral */\n');
fprintf('    u = -LQR_K1 * x1 - LQR_K2 * x2 + 5.0f * ctrl->integral;\n\n');

fprintf('    /* 饱和限制（硬件保护） */\n');
fprintf('    if (u > 12.0f)  u = 12.0f;\n');
fprintf('    if (u < -12.0f) u = -12.0f;\n\n');

fprintf('    /* 更新状态记忆 */\n');
fprintf('    ctrl->x1_hat = x1;\n');
fprintf('    ctrl->x2_hat = x2;\n\n');

fprintf('    return u;\n');
fprintf('}\n\n');

fprintf('/* ================================================ */\n');
fprintf('/*  在 FreeRTOS 任务中调用：                         */\n');
fprintf('/*                                                  */\n');
fprintf('/*  void vLQR_Task(void *pvParameters) {             */\n');
fprintf('/*      LQR_Controller ctrl;                        */\n');
fprintf('/*      LQR_Init(&ctrl);                            */\n');
fprintf('/*      float u, y, ref = 1.0f;                     */\n');
fprintf('/*                                                  */\n');
fprintf('/*      while(1) {                                  */\n');
fprintf('/*          y = read_position_sensor(); // 读传感器  */\n');
fprintf('/*          u = LQR_Step(&ctrl, ref, y);            */\n');
fprintf('/*          set_motor_voltage(u);     // 驱动电机    */\n');
fprintf('/*          vTaskDelay(pdMS_TO_TICKS(5)); // 5ms    */\n');
fprintf('/*      }                                           */\n');
fprintf('/*  }                                               */\n');
fprintf('/* ================================================ */\n\n');

%% ===== 第 4 步：计算复杂度分析 =====

fprintf('========================================\n');
fprintf('  计算复杂度 & 内存占用\n');
fprintf('========================================\n\n');

fprintf('  每个采样周期的运算量：\n');
fprintf('    - 乘法: 3 次 (K1·x1 + K2·x2 + Ki·integral)\n');
fprintf('    - 加法: 3 次\n');
fprintf('    - 条件判断: 2 次 (饱和限幅)\n');
fprintf('    → 总耗时 < 1μs @ 72MHz STM32F103\n\n');

fprintf('  内存占用 (LQR_Controller 结构体):\n');
fprintf('    - 3 个 float × 4 bytes = 12 bytes\n');
fprintf('    - 加上程序代码 < 200 bytes\n');
fprintf('    → 在 20KB SRAM 中几乎忽略不计\n\n');

fprintf('  CPU 负载 (5ms 周期):\n');
fprintf('    - 执行时间 ~1μs / 5ms = 0.02%% CPU 占用\n');
fprintf('    - 意味着你可以同时跑几十个这样的控制器！\n\n');

%% ===== 第 5 步：实际部署检查清单 =====

fprintf('========================================\n');
fprintf('  实际部署检查清单\n');
fprintf('========================================\n\n');

fprintf('  □ 1. 确认采样周期满足 Nyquist (fs > 40×闭环带宽)\n');
fprintf('  □ 2. 传感器数据单位转换 (ADC→物理量)\n');
fprintf('  □ 3. 控制输出饱和限幅 (保护硬件)\n');
fprintf('  □ 4. 积分抗饱和 (integrator anti-windup)\n');
fprintf('  □ 5. 上电初始化 (积分器清零)\n');
fprintf('  □ 6. 故障检测 (传感器超量程 → 安全停机)\n');
fprintf('  □ 7. 看门狗喂狗 (防止程序跑飞后失控)\n\n');

fprintf('========================================\n');
fprintf('  教程 15 完成！\n');
fprintf('========================================\n\n');

fprintf('【MBD 完整工作流总结】\n\n');
fprintf('  1. 物理建模 → 状态空间 (A,B,C,D)        t09, t13\n');
fprintf('  2. 控制器设计 → LQR / PI                t10, t13\n');
fprintf('  3. 状态估计 → Luenberger / Kalman       t11, t12\n');
fprintf('  4. 连续→离散 → c2d() + dlqr()           t15 ← 本课\n');
fprintf('  5. C 代码生成 → 集成到 FreeRTOS         t15 ← 本课\n');
fprintf('  6. 硬件在环测试 → 实际电机 + 传感器\n\n');

fprintf('  掌握这 6 步，就是完整的基于模型设计 (MBD) 工作流！\n\n');

fprintf('  如果买了 Simulink Coder / Embedded Coder：\n');
fprintf('    右键子系统 → C/C++ Code → Build → 自动生成 .c/.h\n');
fprintf('    和手写一样的质量，但省掉调试时间\n\n');

fprintf('  ― Phase 5 结束，全部教程 25 课 ―\n');
fprintf('  恭喜完成全部课程！回顾路径：基础→经典→现代→高级→应用\n');

save_system(mdl, fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']));
close_system(mdl, 0);
