%% ============================================================
% 教程 30：主动悬挂控制 — 1/4 车模型
% 目标：理解汽车主动悬挂的工作原理，用 PID 让车身保持平稳
%
% 【什么是主动悬挂？】
%   传统悬挂 = 弹簧 + 减震器（被动，无法实时调节）
%   主动悬挂 = 弹簧 + 电机执行器（主动出力对抗路面颠簸）
%   传感器检测车身运动 → 控制器算出需要的力 → 电机执行
%
% 【1/4 车模型】
%   把汽车简化成 1/4：一个车轮 + 1/4 车身质量
%   车身 m_s — 乘客感受的振动
%   车轮 m_us — 跟随路面起伏
%   弹簧 k_s — 悬挂弹簧，连接车身和车轮
%   轮胎 k_t — 像一个很硬的弹簧（比悬挂硬 10 倍）
%   执行器 F_motor — 我们设计的主动力
%
% 【控制目标】
%   让车身位移 x_s → 0（不管路面怎么颠，乘客感觉不到）
%   同时保持执行器力在 ±2000N 以内（电机不能无限出力）
%
% 【本课内容】
%   1. 1/4 车动力学方块图
%   2. Simulink 模型：路面→车轮→弹簧→车身→PID→执行器
%   3. 阶跃路面响应对比
%   4. 频域分析
% ============================================================

clear; close all;

fprintf('============================================\n');
fprintf('  教程 30：主动悬挂控制\n');
fprintf('============================================\n\n');

%% ---------- 系统参数 ----------
m_s   = 250;      % 车身质量 (kg)
m_us  = 35;       % 车轮质量 (kg)
k_sus = 16000;    % 悬挂弹簧刚度 (N/m)
k_t   = 160000;   % 轮胎刚度 (N/m) — 比弹簧硬 10 倍！
F_max = 2000;     % 执行器最大出力 (N)
Kp = 8000;  Ki = 2000;  Kd = 3000;  % PID 参数

fprintf('【1/4 车模型参数】\n');
fprintf('  车身 m_s = %d kg  — 乘客在上面\n', m_s);
fprintf('  车轮 m_us = %d kg  — 贴地而行\n', m_us);
fprintf('  弹簧 k_s = %d N/m\n', k_sus);
fprintf('  轮胎 k_t = %d N/m  (弹簧的 10 倍硬)\n', k_t);
fprintf('  执行器限幅: ±%d N\n\n', F_max);

%% ---------- Simulink 模型 ----------

mdl = 'tutorial30_suspension';
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

% ===== 路面扰动 =====
add_block('simulink/Sources/Step', [mdl '/Step_Road'], 'Position', [30,40,85,65]);
set_param([mdl '/Step_Road'], 'Time', '0.5', 'Before', '0', 'After', '0.05');
add_block('simulink/Sources/Sine Wave', [mdl '/Sine_Road'], 'Position', [30,100,85,125]);
set_param([mdl '/Sine_Road'], 'Frequency', '6.28', 'Amplitude', '0.03');
add_block('simulink/Signal Routing/Manual Switch', [mdl '/Road_SW'], 'Position', [130,60,170,100]);
add_line(mdl, 'Step_Road/1', 'Road_SW/1');
add_line(mdl, 'Sine_Road/1', 'Road_SW/2');

% ===== 车轮：z_r - x_us → k_t → SumForce → 1/m_us → ∫→∫ → x_us =====
add_block('simulink/Math Operations/Sum', [mdl '/Sum_zr_xus'], 'Position', [160,55,215,80]);
set_param([mdl '/Sum_zr_xus'], 'Inputs', '|+-');
add_block('simulink/Math Operations/Gain', [mdl '/Gain_kt'], 'Position', [300,55,355,80]);
set_param([mdl '/Gain_kt'], 'Gain', num2str(k_t));
add_block('simulink/Math Operations/Sum', [mdl '/Sum_F_whl'], 'Position', [440,55,495,80]);
set_param([mdl '/Sum_F_whl'], 'Inputs', '|+--');
add_block('simulink/Math Operations/Gain', [mdl '/Gain_1m_us'], 'Position', [580,55,635,80]);
set_param([mdl '/Gain_1m_us'], 'Gain', num2str(1/m_us));
add_block('simulink/Continuous/Integrator', [mdl '/Int_v_us'], 'Position', [720,55,775,80]);
add_block('simulink/Continuous/Integrator', [mdl '/Int_x_us'], 'Position', [860,55,915,80]);

add_line(mdl, 'Road_SW/1', 'Sum_zr_xus/1');
add_line(mdl, 'Sum_zr_xus/1', 'Gain_kt/1');
add_line(mdl, 'Gain_kt/1', 'Sum_F_whl/1');
add_line(mdl, 'Sum_F_whl/1', 'Gain_1m_us/1');
add_line(mdl, 'Gain_1m_us/1', 'Int_v_us/1');
add_line(mdl, 'Int_v_us/1', 'Int_x_us/1');
add_line(mdl, 'Int_x_us/1', 'Sum_zr_xus/2');

% ===== 弹簧：F_s = k_s*(x_us - x_s) =====
add_block('simulink/Math Operations/Sum', [mdl '/Sum_xus_xs'], 'Position', [440,130,495,155]);
set_param([mdl '/Sum_xus_xs'], 'Inputs', '|+-');
add_block('simulink/Math Operations/Gain', [mdl '/Gain_ks'], 'Position', [580,130,635,155]);
set_param([mdl '/Gain_ks'], 'Gain', num2str(k_sus));
add_line(mdl, 'Int_x_us/1', 'Sum_xus_xs/1');
add_line(mdl, 'Sum_xus_xs/1', 'Gain_ks/1');
add_line(mdl, 'Gain_ks/1', 'Sum_F_whl/2');

% ===== 车身：SumForce → 1/m_s → ∫→∫ → x_s =====
add_block('simulink/Math Operations/Sum', [mdl '/Sum_F_bdy'], 'Position', [440,200,495,225]);
set_param([mdl '/Sum_F_bdy'], 'Inputs', '|++');
add_block('simulink/Math Operations/Gain', [mdl '/Gain_1m_s'], 'Position', [580,200,635,225]);
set_param([mdl '/Gain_1m_s'], 'Gain', num2str(1/m_s));
add_block('simulink/Continuous/Integrator', [mdl '/Int_v_s'], 'Position', [720,200,775,225]);
add_block('simulink/Continuous/Integrator', [mdl '/Int_x_s'], 'Position', [860,200,915,225]);
add_line(mdl, 'Gain_ks/1', 'Sum_F_bdy/1');
add_line(mdl, 'Sum_F_bdy/1', 'Gain_1m_s/1');
add_line(mdl, 'Gain_1m_s/1', 'Int_v_s/1');
add_line(mdl, 'Int_v_s/1', 'Int_x_s/1');
add_line(mdl, 'Int_x_s/1', 'Sum_xus_xs/2');

% ===== PID 控制器 =====
add_block('simulink/Sources/Constant', [mdl '/Ref_0'], 'Position', [30,280,80,310]);
set_param([mdl '/Ref_0'], 'Value', '0');
add_block('simulink/Math Operations/Sum', [mdl '/Sum_Err'], 'Position', [160,280,215,305]);
set_param([mdl '/Sum_Err'], 'Inputs', '|+-');
add_line(mdl, 'Ref_0/1', 'Sum_Err/1');
add_line(mdl, 'Int_x_s/1', 'Sum_Err/2');

% P
add_block('simulink/Math Operations/Gain', [mdl '/Gain_Kp'], 'Position', [300,260,355,285]);
set_param([mdl '/Gain_Kp'], 'Gain', num2str(Kp));
add_line(mdl, 'Sum_Err/1', 'Gain_Kp/1');
% I
add_block('simulink/Math Operations/Gain', [mdl '/Gain_Ki'], 'Position', [300,320,355,345]);
set_param([mdl '/Gain_Ki'], 'Gain', num2str(Ki));
add_block('simulink/Continuous/Integrator', [mdl '/Int_I'], 'Position', [440,320,495,345]);
add_line(mdl, 'Sum_Err/1', 'Gain_Ki/1');
add_line(mdl, 'Gain_Ki/1', 'Int_I/1');
% D
add_block('simulink/Math Operations/Gain', [mdl '/Gain_Kd'], 'Position', [300,380,355,405]);
set_param([mdl '/Gain_Kd'], 'Gain', num2str(Kd));
add_block('simulink/Continuous/Derivative', [mdl '/Deriv_D'], 'Position', [440,380,495,405]);
add_line(mdl, 'Sum_Err/1', 'Gain_Kd/1');
add_line(mdl, 'Gain_Kd/1', 'Deriv_D/1');

% PID求和 + 限幅
add_block('simulink/Math Operations/Sum', [mdl '/Sum_PID'], 'Position', [580,300,635,325]);
set_param([mdl '/Sum_PID'], 'Inputs', '|+++');
add_line(mdl, 'Gain_Kp/1', 'Sum_PID/1');
add_line(mdl, 'Int_I/1', 'Sum_PID/2');
add_line(mdl, 'Deriv_D/1', 'Sum_PID/3');

add_block('simulink/Discontinuities/Saturation', [mdl '/Motor_Sat'], 'Position', [720,300,775,325]);
set_param([mdl '/Motor_Sat'], 'UpperLimit', num2str(F_max), 'LowerLimit', num2str(-F_max));
add_line(mdl, 'Sum_PID/1', 'Motor_Sat/1');
add_line(mdl, 'Motor_Sat/1', 'Sum_F_bdy/2');
add_line(mdl, 'Motor_Sat/1', 'Sum_F_whl/3');

% ===== 可视化 =====
add_block('simulink/Signal Routing/Mux', [mdl '/Mux_3in'], 'Inputs', '3');
set_param([mdl '/Mux_3in'], 'Position', [1000,80,1020,150]);
add_line(mdl, 'Int_x_s/1', 'Mux_3in/1');
add_line(mdl, 'Int_v_s/1', 'Mux_3in/2');
add_line(mdl, 'Gain_1m_s/1', 'Mux_3in/3');
add_block('simulink/Sinks/Scope', [mdl '/Scope_All'], 'Position', [1080,80,1135,160]);
add_line(mdl, 'Mux_3in/1', 'Scope_All/1');
add_block('simulink/Sinks/Scope', [mdl '/Scope_Rd_vs_Bd'], 'NumInputPorts', '2');
set_param([mdl '/Scope_Rd_vs_Bd'], 'Position', [1080,200,1135,275]);
add_line(mdl, 'Road_SW/1', 'Scope_Rd_vs_Bd/1');
add_line(mdl, 'Int_x_s/1', 'Scope_Rd_vs_Bd/2');

fprintf('  [Simulink] tutorial30_suspension.slx 已创建\n');
fprintf('  模型: 路面→车轮→弹簧→车身→PID→执行器, 完整闭环\n\n');

%% ---------- 状态空间模型 & 对比仿真 ----------
A_sus = [0,1,0,0; -(k_t+k_sus)/m_us,0,k_sus/m_us,0; 0,0,0,1; k_sus/m_s,0,-k_sus/m_s,0];
B_sus = [0;-1/m_us;0;1/m_s]; B_dist = [0;k_t/m_us;0;0]; C_sus = [0,0,1,0];

sys_passive = ss(A_sus, B_dist, C_sus, 0);
[y_pas,t_pas] = step(sys_passive*0.05, 3);
fprintf('【被动悬挂】5cm阶跃路面 → 车身峰值位移: %.1f cm\n', max(abs(y_pas))*100);

dt=0.001; t=0:dt:3-dt; N=length(t); zr=zeros(1,N); zr(t>=0.5)=0.05;
x=zeros(4,N); ie=0;
for n=1:N-1
    e=0-x(3,n); de=-x(4,n); ie=ie+e*dt;
    u=Kp*e+Ki*ie+Kd*de; u=max(-F_max,min(F_max,u));
    x(:,n+1)=x(:,n)+(A_sus*x(:,n)+B_sus*u+B_dist*zr(n))*dt;
end
fprintf('【PID控制】车身峰值位移: %.1f cm\n', max(abs(x(3,:)))*100);

%% ---------- 绘图 ----------
figure('Name','t30: 主动悬挂对比','Position',[50,50,1000,500]);
subplot(2,2,1);hold on;
plot(t,zr*100,'k:',t_pas,y_pas*100,'Color',[0.6,0.6,0.6],'LineWidth',1.5);
plot(t,x(3,:)*100,'b','LineWidth',1.5);
legend('路面(5cm)','被动','PID');title('车身位移对比');xlabel('t(s)');ylabel('cm');grid on;
subplot(2,2,2);plot(t,x(4,:));title('车身速度');xlabel('t(s)');grid on;
subplot(2,2,3:4);bodemag(ss(A_sus,B_dist,C_sus,0));grid on;
title('被动悬挂频率响应: 路面→车身');

fprintf('\n========================================\n');
fprintf('  教程 30 完成！\n');
fprintf('========================================\n\n');
fprintf('动手实验：\n');
fprintf('  1. 打开 tutorial30_suspension.slx, 双击 Road_SW 切换路面\n');
fprintf('  2. 观察 Scope_All：车身位移、速度、加速度\n');
fprintf('  3. 改 Kp=4000 vs Kp=16000 看差异\n');
fprintf('  4. 把 m_s 改成 500（SUV）重新运行\n');

save_system(mdl, fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']));
close_system(mdl, 0);
