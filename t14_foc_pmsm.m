%% ============================================================
% 教程 14：PMSM + FOC 矢量控制
%
% 【从 DC 电机到无刷电机】
%   t13 的 DC 电机：电刷+换向器 → 电流天然与磁场垂直 → 天生解耦
%   PMSM (永磁同步电机)：无电刷 → 电流方向由逆变器决定 → 需要控制角度
%
%   FOC (Field-Oriented Control) 的思想：
%   "用数学旋转坐标系，把 PMSM 变成两个 DC 电机"
%
% ┌─────────────────────────────────────────────────────────┐
% │ 一、为什么需要 FOC？                                     │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │   DC 电机：                                              │
% │     电刷自动换向 → 定子磁场永远与转子磁场垂直            │
% │     → 电流 100% 产生转矩，无浪费                         │
% │                                                         │
% │   PMSM (无电刷)：                                       │
% │     三相电流产生旋转磁场 → 需要控制磁场方向              │
% │     → 磁场不对齐 = 电流浪费 = 效率低、转矩脉动           │
% │                                                         │
% │   FOC 解决方案：                                         │
% │     通过 Clarke/Park 坐标变换，在旋转的 d-q 坐标系中     │
% │     控制电流 → d 轴电流控制磁场，q 轴电流控制转矩        │
% └─────────────────────────────────────────────────────────┘
%
% ┌─────────────────────────────────────────────────────────┐
% │ 二、Clarke + Park 变换（数学如何"旋转"坐标系）           │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │   三相静止 (a,b,c)  ──Clarke──→ 两相静止 (α,β)         │
% │   两相静止 (α,β)    ──Park────→ 两相旋转 (d,q)          │
% │                                                         │
% │   Park 变换的关键：需要实时知道转子角度 θ！              │
% │   θ 来自编码器/Hall/无传感器观测器                       │
% │                                                         │
% │   在 d-q 坐标系中：                                      │
% │     d 轴 (direct)：对齐转子磁极方向 → 控制"磁场强度"     │
% │     q 轴 (quadrature)：与 d 轴垂直 → 控制"转矩"          │
% │                                                         │
% │   标准策略 (id=0)：                                      │
% │     所有电流都在 q 轴 → 100% 用于产生转矩                │
% │     → PMSM 变成了一个 DC 电机！                          │
% └─────────────────────────────────────────────────────────┘
%
% ┌─────────────────────────────────────────────────────────┐
% │ 三、PMSM 在 d-q 坐标系中的方程                           │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │   d 轴电压方程：     ud = R·id + Ld·did/dt - ωe·Lq·iq   │
% │   q 轴电压方程：     uq = R·iq + Lq·diq/dt + ωe·Ld·id   │
% │                                       + ωe·λpm           │
% │   (ωe·λpm = 反电动势，类似 DC 电机的 Ke·ω)               │
% │                                                         │
% │   转矩方程：    Te = 1.5·P·[λpm·iq + (Ld-Lq)·id·iq]    │
% │   运动方程：    J·dωm/dt = Te - TL - b·ωm                │
% │                                                         │
% │   当 id=0 (SPMSM Ld≈Lq)：                                │
% │     Te = 1.5·P·λpm·iq                                   │
% │     → iq 就是"转矩电流"，完全等价于 DC 电机的 i          │
% └─────────────────────────────────────────────────────────┘
%
% ┌─────────────────────────────────────────────────────────┐
% │ 四、FOC 控制架构（和 t13 DC 电机几乎一样！）             │
% ├─────────────────────────────────────────────────────────┤
% │                                                         │
% │  ω_ref → [PI_Spd] → iq_ref → [PI_iq] → uq → [PMSM] → ω │
% │               ↑                           ↑      │      │
% │               └─────────── ω ─────────────┘      │      │
% │                                                  ↓      │
% │  0 (id_ref) → [PI_id] → ud → [PMSM]                    │
% │               ↑                                         │
% │               └─── id ──────────────────←───────────────│
% │                                                         │
% │   + 解耦前馈 (decoupling) → 抵消交叉耦合项               │
% │     ud = ud_pi - ωe·Lq·iq                               │
% │     uq = uq_pi + ωe·(Ld·id + λpm)                       │
% └─────────────────────────────────────────────────────────┘
%
% 【本课目标】
%   1. 理解 FOC 为什么能把 PMSM 变成"两个 DC 电机"
%   2. 掌握 d-q 坐标系下的 PMSM 模型
%   3. 搭一个简化的 FOC 仿真，对比和 t13 DC 电机的相似性
% ============================================================

clear; close all;
addpath(fullfile(fileparts(mfilename('fullpath')), 'utils'));

%% ===== PMSM 参数（小型云台/机器人关节电机）=====

R_s   = 0.5;      % 定子电阻 (Ω)
Ld    = 0.0005;   % d 轴电感 (H)
Lq    = 0.0005;   % q 轴电感 (H) — SPMSM: Ld≈Lq
lambda_pm = 0.01; % 永磁体磁链 (Wb) — 决定反电动势大小
P     = 7;        % 极对数
J_m   = 0.0001;   % 转动惯量 (kg·m²)
b_m   = 0.00001;  % 粘滞摩擦 N·m/(rad/s)
V_dc  = 24;       % 直流母线电压 (V)

fprintf('============================================\n');
fprintf('  教程 14：PMSM + FOC 矢量控制\n');
fprintf('============================================\n\n');

%% ===== 第 1 步：理解 PMSM 和 DC 电机的对应关系 =====

% DC 电机 (t13)：    Te = Kt * i          (电流→转矩)
% PMSM (id=0):      Te = 1.5*P*λpm * iq  (q轴电流→转矩)
%
% 等效 Kt： Kt_eq = 1.5*P*λpm

Kt_eq = 1.5 * P * lambda_pm;

fprintf('【PMSM vs DC 电机对应关系】\n');
fprintf('  DC 电机: Te = Kt × i      (Kt = %.3f)\n', 0.015);
fprintf('  PMSM:    Te = 1.5·P·λpm × iq\n');
fprintf('            = %.4f × iq    (等效 Kt)\n\n', Kt_eq);

fprintf('  反电动势：\n');
fprintf('  DC 电机: e = Ke·ω       (Ke = %.3f)\n', 0.015);
fprintf('  PMSM:    e = P·λpm·ω    (等效 Ke = %.3f)\n\n', P*lambda_pm);

fprintf('  电气时间常数 (q 轴):\n');
fprintf('  DC 电机: τ_e = L/R = %.3f ms\n', 0.002/2*1000);
fprintf('  PMSM:    τ_e = Lq/R = %.3f ms\n\n', Lq/R_s*1000);

fprintf('  本质：FOC 让 PMSM 在 d-q 坐标系中                        \n');
fprintf('        变成两个解耦的 RL 电路 + 转矩方程                   \n');
fprintf('        和 DC 电机完全一样！                                \n\n');

%% ===== 第 2 步：PI 控制器设计（和 t13 完全相同的思路）=====

% 电流环带宽 ~1000 Hz (比 t13 更高，因为 L 更小)
wc_cur = 2*pi*1000;
Kp_cur_foc = wc_cur * Ld;    % 同 t13: Kp = ωc × L
Ki_cur_foc = wc_cur * R_s;   % 同 t13: Ki = ωc × R

% 速度环带宽 ~30 Hz
wc_spd_foc = 2*pi*30;
Kp_spd_foc = wc_spd_foc * J_m / Kt_eq;      % 同 t13: Kp = ωc × J/Kt
Ki_spd_foc = wc_spd_foc * b_m / Kt_eq + 0.1*Kp_spd_foc;

fprintf('【FOC 级联 PI 设计】\n');
fprintf('  电流环 (~1000Hz): Kp=%.4f, Ki=%.1f\n', Kp_cur_foc, Ki_cur_foc);
fprintf('  速度环 (~30Hz):   Kp=%.4f, Ki=%.4f\n\n', Kp_spd_foc, Ki_spd_foc);

%% ===== 第 3 步：搭建 Simulink =====

mdl = 'tutorial14_foc';
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

% ┌────────────────── 模型结构 ────────────────────┐
% │                                                  │
% │  行 1：DC 电机 (t13 参考)                        │
% │  行 2：PMSM FOC (本课重点)                       │
% │                                                  │
% │  PMSM 在 d-q 坐标系下的等效模型：                │
% │    did/dt = (-R·id + ωe·Lq·iq + ud)/Ld          │
% │    diq/dt = (-R·iq - ωe·Ld·id - ωe·λpm + uq)/Lq │
% │    dωm/dt = (1.5·P·λpm·iq - b·ωm)/J            │
% │    ωe = P·ωm (电角速度 = 极对数 × 机械角速度)    │
% └──────────────────────────────────────────────────┘

% --- 参考输入 ---
add_block('simulink/Sources/Step', [mdl '/Setpoint'], ...
    'Position', [50, 60, 130, 100]);
set_param([mdl '/Setpoint'], ...
    'Time', '0.01', 'Before', '0', 'After', '200');  % 200 rad/s

% ===== 行 1：DC 电机 (用 t13 的参数，做对比) =====
R_dc=2.0; L_dc=0.002; Ke_dc=0.015; Kt_dc=0.015; J_dc=0.0005; b_dc=0.00001;
A_dc = [-R_dc/L_dc, -Ke_dc/L_dc; Kt_dc/J_dc, -b_dc/J_dc];
B_dc = [1/L_dc; 0];

add_block('simulink/Continuous/State-Space', [mdl '/DC_Motor'], ...
    'Position', [220, 60, 290, 110]);
set_param([mdl '/DC_Motor'], ...
    'A', mat2str(A_dc), 'B', mat2str(B_dc), ...
    'C', '[0 1]', 'D', '0', 'X0', '[0; 0]');

add_block('simulink/Sinks/To Workspace', [mdl '/ws_DC'], ...
    'Position', [350, 70, 410, 100]);
set_param([mdl '/ws_DC'], 'VariableName', 'w_DC');

add_line(mdl, 'Setpoint/1', 'DC_Motor/1');
add_line(mdl, 'DC_Motor/1', 'ws_DC/1');

% ===== 行 2：PMSM d-q 模型 (手搭，像 t09 Manual SS) =====

subsys = [mdl '/PMSM_dq_Model'];

% 用子系统封装 PMSM
add_block('simulink/Ports & Subsystems/Subsystem', subsys, ...
    'Position', [220, 180, 290, 240]);
delete_line(subsys, 'In1/1', 'Out1/1');
set_param(subsys, 'BackgroundColor', 'lightBlue');

% 子系统内部搭建 PMSM d-q 模型
% 输入：ud, uq (两个电压)
% 需要两个 Inport

% 再添一个 Inport
add_block('simulink/Sources/In1', [subsys '/In2'], ...
    'Position', [50, 140, 80, 170]);

% --- d 轴电路：did/dt = (-R·id + ωe·Lq·iq + ud)/Ld ---
% 加法器：+ud - R·id + ωe·Lq·iq
add_block('simulink/Math Operations/Add', [subsys '/Sum_did'], ...
    'Position', [130, 50, 165, 100]);
set_param([subsys '/Sum_did'], 'Inputs', '|++-', 'IconShape', 'round');

add_block('simulink/Math Operations/Gain', [subsys '/Gain_1overLd'], ...
    'Position', [200, 60, 240, 90]);
set_param([subsys '/Gain_1overLd'], 'Gain', num2str(1/Ld));

add_block('simulink/Continuous/Integrator', [subsys '/Int_id'], ...
    'Position', [280, 60, 340, 100]);
set_param([subsys '/Int_id'], 'InitialCondition', '0');

% --- q 轴电路：diq/dt = (-R·iq - ωe·Ld·id - ωe·λpm + uq)/Lq ---
add_block('simulink/Math Operations/Add', [subsys '/Sum_diq'], ...
    'Position', [130, 200, 170, 260]);
set_param([subsys '/Sum_diq'], 'Inputs', '|++--', 'IconShape', 'round');

add_block('simulink/Math Operations/Gain', [subsys '/Gain_1overLq'], ...
    'Position', [200, 215, 240, 245]);
set_param([subsys '/Gain_1overLq'], 'Gain', num2str(1/Lq));

add_block('simulink/Continuous/Integrator', [subsys '/Int_iq'], ...
    'Position', [280, 220, 340, 260]);
set_param([subsys '/Int_iq'], 'InitialCondition', '0');

% --- 转矩和机械方程 ---
% Te = 1.5*P*λpm*iq
add_block('simulink/Math Operations/Gain', [subsys '/Gain_Te'], ...
    'Position', [400, 220, 440, 250]);
set_param([subsys '/Gain_Te'], 'Gain', num2str(1.5*P*lambda_pm));

% 加法器：Te - b·ωm = J·dωm/dt
add_block('simulink/Math Operations/Add', [subsys '/Sum_dwm'], ...
    'Position', [480, 220, 510, 250]);
set_param([subsys '/Sum_dwm'], 'Inputs', '|+-', 'IconShape', 'round');

add_block('simulink/Math Operations/Gain', [subsys '/Gain_1overJ'], ...
    'Position', [540, 225, 580, 255]);
set_param([subsys '/Gain_1overJ'], 'Gain', num2str(1/J_m));

add_block('simulink/Continuous/Integrator', [subsys '/Int_wm'], ...
    'Position', [620, 220, 680, 260]);
set_param([subsys '/Int_wm'], 'InitialCondition', '0');

% --- 交叉耦合项和反电动势 ---
% ωe = P·ωm
add_block('simulink/Math Operations/Gain', [subsys '/Gain_we'], ...
    'Position', [750, 230, 790, 260]);
set_param([subsys '/Gain_we'], 'Gain', num2str(P));

% ωe*Lq*iq (加到 d 轴方程)
add_block('simulink/Math Operations/Product', [subsys '/Prod_weLqiq'], ...
    'Position', [750, 310, 790, 340]);
set_param([subsys '/Prod_weLqiq'], 'Inputs', '**');

add_block('simulink/Math Operations/Gain', [subsys '/Gain_Lq'], ...
    'Position', [680, 310, 720, 340]);
set_param([subsys '/Gain_Lq'], 'Gain', num2str(Lq));

% ωe*Ld*id (加到 q 轴方程，负)
add_block('simulink/Math Operations/Product', [subsys '/Prod_weLdid'], ...
    'Position', [750, 380, 790, 410]);
set_param([subsys '/Prod_weLdid'], 'Inputs', '**');

add_block('simulink/Math Operations/Gain', [subsys '/Gain_Ld_x'], ...
    'Position', [680, 380, 720, 410]);
set_param([subsys '/Gain_Ld_x'], 'Gain', num2str(Ld));

% ωe*λpm (反电动势，加到 q 轴方程，负)
add_block('simulink/Math Operations/Gain', [subsys '/Gain_we_lambda'], ...
    'Position', [750, 450, 790, 480]);
set_param([subsys '/Gain_we_lambda'], 'Gain', num2str(lambda_pm));

% R*id (电压降，负)
add_block('simulink/Math Operations/Gain', [subsys '/Gain_R_id'], ...
    'Position', [400, 30, 440, 60]);
set_param([subsys '/Gain_R_id'], 'Gain', num2str(-R_s));

% R*iq (电压降，负)
add_block('simulink/Math Operations/Gain', [subsys '/Gain_R_iq'], ...
    'Position', [400, 170, 440, 200]);
set_param([subsys '/Gain_R_iq'], 'Gain', num2str(-R_s));

% b*ωm (摩擦转矩)
add_block('simulink/Math Operations/Gain', [subsys '/Gain_b_wm'], ...
    'Position', [580, 170, 620, 200]);
set_param([subsys '/Gain_b_wm'], 'Gain', num2str(b_m));

% --- 输出 ωm (转速) → 用默认 Out1 ---

% --- 内部连线 ---

% d 轴电路
add_line(subsys, 'In1/1', 'Sum_did/1');      % ud → Sum_did(+)
add_line(subsys, 'Gain_R_id/1', 'Sum_did/2'); % -R*id
% Prod_weLqiq → Sum_did/3 后面补

add_line(subsys, 'Sum_did/1', 'Gain_1overLd/1');
add_line(subsys, 'Gain_1overLd/1', 'Int_id/1');

% q 轴电路
add_line(subsys, 'In2/1', 'Sum_diq/1');       % uq → Sum_diq(+)
add_line(subsys, 'Gain_R_iq/1', 'Sum_diq/2'); % -R*iq
% -ωe*Ld*id → Sum_diq/3, -ωe*λpm → Sum_diq/4 后面补

add_line(subsys, 'Sum_diq/1', 'Gain_1overLq/1');
add_line(subsys, 'Gain_1overLq/1', 'Int_iq/1');

% 转矩和机械
add_line(subsys, 'Int_iq/1', 'Gain_Te/1');
add_line(subsys, 'Gain_Te/1', 'Sum_dwm/1');
add_line(subsys, 'Gain_b_wm/1', 'Sum_dwm/2');  % -b*ωm
add_line(subsys, 'Sum_dwm/1', 'Gain_1overJ/1');
add_line(subsys, 'Gain_1overJ/1', 'Int_wm/1');

% ωe = P*ωm
add_line(subsys, 'Int_wm/1', 'Gain_we/1');
add_line(subsys, 'Int_wm/1', 'Gain_b_wm/1');  % ωm → b*ωm

% 交叉耦合：ωe*Lq*iq
add_line(subsys, 'Int_iq/1', 'Gain_Lq/1');
add_line(subsys, 'Gain_Lq/1', 'Prod_weLqiq/1');
add_line(subsys, 'Gain_we/1', 'Prod_weLqiq/2');

% 交叉耦合：ωe*Ld*id
add_line(subsys, 'Int_id/1', 'Gain_Ld_x/1');
add_line(subsys, 'Gain_Ld_x/1', 'Prod_weLdid/1');
add_line(subsys, 'Gain_we/1', 'Prod_weLdid/2');

% 反电动势：ωe*λpm
add_line(subsys, 'Gain_we/1', 'Gain_we_lambda/1');

% R*id 和 R*iq
add_line(subsys, 'Int_id/1', 'Gain_R_id/1');
add_line(subsys, 'Int_iq/1', 'Gain_R_iq/1');

% 交叉耦合项连入 Sum
add_line(subsys, 'Prod_weLqiq/1', 'Sum_did/3');     % +ωe*Lq*iq
add_line(subsys, 'Prod_weLdid/1', 'Sum_diq/3');     % -ωe*Ld*id
add_line(subsys, 'Gain_we_lambda/1', 'Sum_diq/4');  % -ωe*λpm

% 输出
add_line(subsys, 'Int_wm/1', 'Out1/1');

fprintf('  [OK] PMSM d-q 模型搭建完成\n');

%% ===== 第 4 步：简单 FOC 控制 (id=0 + PI 速度环) =====

% 为简化，本模型只用单 PI 速度环控制 uq
% (完整 FOC 需要电流环 + 坐标变换, 这里展示核心结构)

% 速度误差
add_block('simulink/Math Operations/Add', [mdl '/Err_FOC'], ...
    'Position', [380, 180, 410, 210]);
set_param([mdl '/Err_FOC'], 'Inputs', '|+-');

% PI 速度环
add_block('simulink/Math Operations/Gain', [mdl '/Kp_FOC'], ...
    'Position', [450, 160, 490, 190]);
set_param([mdl '/Kp_FOC'], 'Gain', num2str(Kp_spd_foc));

add_block('simulink/Math Operations/Gain', [mdl '/Ki_FOC'], ...
    'Position', [450, 200, 490, 230]);
set_param([mdl '/Ki_FOC'], 'Gain', num2str(Ki_spd_foc));

add_block('simulink/Continuous/Integrator', [mdl '/Int_FOC'], ...
    'Position', [530, 200, 580, 240]);
set_param([mdl '/Int_FOC'], 'InitialCondition', '0');

add_block('simulink/Math Operations/Add', [mdl '/Sum_FOC'], ...
    'Position', [620, 185, 650, 225]);
set_param([mdl '/Sum_FOC'], 'Inputs', '|++', 'IconShape', 'round');

% 电压饱和 24V
add_block('simulink/Discontinuities/Saturation', [mdl '/Sat_FOC'], ...
    'Position', [690, 185, 720, 215]);
set_param([mdl '/Sat_FOC'], 'UpperLimit', num2str(V_dc), ...
    'LowerLimit', num2str(-V_dc));

% To Workspace
add_block('simulink/Sinks/To Workspace', [mdl '/ws_FOC'], ...
    'Position', [800, 180, 860, 210]);
set_param([mdl '/ws_FOC'], 'VariableName', 'w_FOC');

% --- 连线 ---
add_line(mdl, 'Setpoint/1', 'Err_FOC/1');
add_line(mdl, 'Err_FOC/1', 'Kp_FOC/1');
add_line(mdl, 'Err_FOC/1', 'Ki_FOC/1');
add_line(mdl, 'Ki_FOC/1', 'Int_FOC/1');
add_line(mdl, 'Kp_FOC/1', 'Sum_FOC/1');
add_line(mdl, 'Int_FOC/1', 'Sum_FOC/2');
add_line(mdl, 'Sum_FOC/1', 'Sat_FOC/1');

% uq → PMSM (In2), ud=0 → PMSM (In1) 接 GND
add_block('simulink/Sources/Constant', [mdl '/id_ref_0'], ...
    'Position', [350, 260, 380, 290]);
set_param([mdl '/id_ref_0'], 'Value', '0');

add_line(mdl, 'id_ref_0/1', 'PMSM_dq_Model/1');   % ud = 0 (id control)
add_line(mdl, 'Sat_FOC/1', 'PMSM_dq_Model/2');     % uq = PI output

% 速度反馈
add_line(mdl, 'PMSM_dq_Model/1', 'Err_FOC/2');
add_line(mdl, 'PMSM_dq_Model/1', 'ws_FOC/1');

Simulink.BlockDiagram.arrangeSystem(mdl);

fprintf('  [OK] FOC 控制回路搭建完成\n');

%% ===== 第 5 步：运行仿真 =====

fprintf('\n=== 运行仿真 ===\n');
set_param(mdl, 'StopTime', '0.15');
simOut = sim(mdl);

%% ===== 第 6 步：结果分析 =====

figure('Name', 't14: PMSM FOC 矢量控制', 'Position', [50, 50, 900, 600]);
t = simOut.tout;

subplot(2, 1, 1);
w_dc = getSimData(simOut, 'w_DC', t);
w_foc = getSimData(simOut, 'w_FOC', t);
plot(t, w_dc, 'b--', 'LineWidth', 2); hold on;
plot(t, w_foc, 'r-', 'LineWidth', 2);
yline(200, ':', 'Color', [0.5 0.5 0.5]);
hold off;
legend('DC 电机 (开环)', 'PMSM (FOC PI)', '目标 200 rad/s', ...
    'Location', 'southeast');
title('转速响应 — DC 电机 (开环) vs PMSM (FOC 速度闭环)');
xlabel('时间 (s)'); ylabel('转速 ω (rad/s)'); grid on;

subplot(2, 1, 2);
text(0.1, 0.7, 'FOC 核心：PMSM 在 d-q 坐标系中等价于 "两个 DC 电机"', ...
    'FontSize', 12);
text(0.1, 0.45, 'd 轴电流 id → 控制磁场 (id=0 → 效率最优)', ...
    'FontSize', 11);
text(0.1, 0.25, 'q 轴电流 iq → 控制转矩 (等价于 DC 电机的 i)', ...
    'FontSize', 11);
text(0.1, 0.05, '级联 PI = 速度环 (外) + 电流环 id/iq (内) + 解耦前馈', ...
    'FontSize', 11);
axis off;

sgtitle('教程 14：PMSM + FOC 矢量控制');

%% ===== 第 7 步：总结 =====

fprintf('\n========================================\n');
fprintf('  教程 14 完成！\n');
fprintf('========================================\n\n');

fprintf('【FOC 核心总结】\n\n');

fprintf('  1. FOC 三要素：\n');
fprintf('     Clarke 变换: 3相电流 (a,b,c) → 2相静止 (α,β)\n');
fprintf('     Park 变换:   静止 (α,β) → 旋转 (d,q) [需要角度θ!]\n');
fprintf('     PI 控制:     id=0, iq = 转矩电流\n\n');

fprintf('  2. PMSM vs DC 电机：\n');
fprintf('     DC:      Te = Kt × i           (自然解耦)\n');
fprintf('     PMSM:    Te = 1.5·P·λpm × iq   (FOC 解耦后)\n');
fprintf('     本质相同！都是电流控制转矩\n\n');

fprintf('  3. 为什么用 id=0？\n');
fprintf('     SPMSM (表贴式): Ld≈Lq → id 不产生转矩\n');
fprintf('     id≠0 只会浪费电流、发热 → 保持 id=0 效率最高\n');
fprintf('     IPMSM (内置式): Ld≠Lq → 可以 id<0 做弱磁扩速\n\n');

fprintf('  4. FOC 的实际实现：\n');
fprintf('     ├─ 角度 θ 从哪来？编码器/Hall/无传感器观测器\n');
fprintf('     │   (IMU + Kalman 姿态估计做的就是同类工作)\n');
fprintf('     ├─ 电流从哪测？两相电流传感器 (第三相可算)\n');
fprintf('     ├─ 坐标变换：实时 sin/cos 计算 (STM32 用查表)\n');
fprintf('     └─ 空间矢量 PWM：把 dq 电压变成 6 路 PWM 信号\n\n');

fprintf('  5. 与平衡机器人等实际系统的连接：\n');
fprintf('     IMU → Kalman → 姿态角 θ\n');
fprintf('     姿态角 θ → 位置环 → 速度环 → FOC 电流环 → 电机\n');
fprintf('     机器人平衡 = 传感器 + 估计 + 控制 + FOC！\n\n');

fprintf('  下一课预告：t15 — 从 Simulink 到 C 代码\n');
fprintf('  把控制算法部署到 STM32！生成嵌入式代码！\n');

