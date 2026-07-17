%% ============================================================
%  Simulink 控制工程教程 — 总览
%  ============================================================
%  本教程面向零基础初学者，采用"运行代码 → 观察模型 → 修改参数"的
%  学习路径。每个脚本会：
%    1. 自动创建一个 Simulink 模型（.slx 文件）
%    2. 添加模块、连线、设置参数
%    3. 运行仿真并生成结果图
%
%  三个阶段，21 个教程：
%
%  Phase 1 — Simulink 基础
%    t01_signal_basics      — 认识模块与信号
%    t02_math_blocks        — 数学运算与信号路由
%    t03_first_order_sys    — 一阶系统与传递函数
%    t04_second_order_sys   — 二阶系统与阻尼特性
%    t05_pid_control        — PID 反馈控制
%    t06_sources_and_sinks  — 信号导入导出
%    t07_subsystem          — 子系统与模型封装
%    t08_masking            — Mask 参数化封装
%
%  Phase 2 — 控制理论
%    t09_state_space        — 状态空间模型
%    t10_state_feedback     — 极点配置与 LQR 最优控制
%    t11_observer           — Luenberger 状态观测器
%    t12_kalman_filter      — Kalman 滤波
%    t16_freq_domain        — 频域分析：Bode/Nyquist/稳定裕度
%    t17_lead_lag           — 超前-滞后校正器设计
%    t18_system_id          — 系统辨识：从实验数据到模型
%    t19_robust_control     — H∞ 鲁棒控制与 μ 分析
%    t20_mpc                — 模型预测控制 (MPC)
%    t21_sliding_mode       — 滑模控制：非线性鲁棒控制
%
%  Phase 3 — 机电系统与部署
%    t13_dc_motor           — 直流电机建模与级联控制
%    t14_foc_pmsm           — PMSM 矢量控制 (FOC)
%    t15_code_generation    — 离散化与 C 代码生成
%
%  使用方法：
%    在 MATLAB 命令窗口中输入脚本名运行，例如：
%    >> t01_signal_basics
%
%  每次运行后，建议你：
%    - 双击模型窗口中的模块查看其参数设置
%    - 尝试修改参数（如增益值、频率等）并重新运行
%    - 双击 Scope 模块查看波形
%    - 观察 MATLAB Figure 中生成的对比图
%
%  依赖：Control System Toolbox (t09 起用到 place/lqr/lqe/c2d 等)
%  ============================================================

fprintf('=================================================\n');
fprintf('  Simulink 控制工程教程 — 15 课学习路径\n');
fprintf('=================================================\n\n');

fprintf('--- Phase 1: Simulink 基础 ---\n');
fprintf('第 1 课  | t01_signal_basics      | 信号、增益、示波器\n');
fprintf('第 2 课  | t02_math_blocks        | 加法、乘法、Mux/Demux\n');
fprintf('第 3 课  | t03_first_order_sys    | 一阶系统、传递函数、时间常数\n');
fprintf('第 4 课  | t04_second_order_sys   | 二阶系统、阻尼比、超调\n');
fprintf('第 5 课  | t05_pid_control        | PID 控制器、反馈闭环\n');
fprintf('第 6 课  | t06_sources_and_sinks  | 数据导入导出、多种信号源\n');
fprintf('第 7 课  | t07_subsystem          | 子系统、模型层次化封装\n');
fprintf('第 8 课  | t08_masking            | Mask 参数化封装\n\n');

fprintf('--- Phase 2: 控制理论 ---\n');
fprintf('第 9 课  | t09_state_space        | 状态空间模型 ẋ=Ax+Bu\n');
fprintf('第 10 课 | t10_state_feedback     | 极点配置、LQR 最优控制\n');
fprintf('第 11 课 | t11_observer           | Luenberger 状态观测器\n');
fprintf('第 12 课 | t12_kalman_filter      | Kalman 滤波、噪声建模\n');
fprintf('第 16 课 | t16_freq_domain        | 频域分析 Bode/Nyquist/稳定裕度\n');
fprintf('第 17 课 | t17_lead_lag           | 超前-滞后校正器设计\n');
fprintf('第 18 课 | t18_system_id          | 系统辨识：从数据到模型\n');
fprintf('第 19 课 | t19_robust_control     | H∞ 鲁棒控制与 μ 分析\n');
fprintf('第 20 课 | t20_mpc                | 模型预测控制 MPC\n');
fprintf('第 21 课 | t21_sliding_mode       | 滑模控制：非线性鲁棒控制\n\n');

fprintf('--- Phase 3: 机电系统与部署 ---\n');
fprintf('第 13 课 | t13_dc_motor           | 直流电机建模、级联 PI\n');
fprintf('第 14 课 | t14_foc_pmsm           | PMSM、Clarke/Park、FOC\n');
fprintf('第 15 课 | t15_code_generation    | 离散化、C 代码、嵌入式部署\n\n');

fprintf('现在就运行第一课试试：>> t01_signal_basics\n');
fprintf('=================================================\n');
