%% ============================================================
% 教程 26：机器人运动学 — DH 参数与正运动学
% 目标：理解 DH 参数法、正运动学的"关节角→末端位姿"计算
%
% 【什么是 DH 参数法？】
%   机械臂由多个关节串联而成。两个相邻关节之间的
%   几何关系用 4 个数 (a, α, d, θ) 唯一确定——
%   这就是 Denavit-Hartenberg (DH) 参数。
%
%   a  — 连杆长度：两根轴之间的公垂线距离
%   α  — 连杆扭转角：两根轴之间的夹角
%   d  — 关节偏距：沿前一根轴方向的偏移
%   θ  — 关节角：绕前一根轴的旋转角度（对旋转关节是变量）
%
% 【正运动学 (Forward Kinematics)】
%   已知每个关节的角度 θ₁~θ₆ → 问末端 XYZ 在哪里？
%   答案唯一：把 6 个齐次变换矩阵连乘即可。
%
% 【本课内容】
%   1. 欧拉角与旋转矩阵
%   2. PUMA560 的 DH 参数
%   3. Simulink 模型：输入关节角 → 输出 XYZ
%   4. 工作空间可视化
% ============================================================

clear; close all;

fprintf('============================================\n');
fprintf('  教程 26：机器人运动学\n');
fprintf('============================================\n\n');

%% ---------- 第 1 步：感受旋转 ------------------
% 三个基本旋转矩阵：分别绕 X / Y / Z 轴旋转
Rx = @(a) [1,0,0; 0,cos(a),-sin(a); 0,sin(a),cos(a)];
Ry = @(b) [cos(b),0,sin(b); 0,1,0; -sin(b),0,cos(b)];
Rz = @(g) [cos(g),-sin(g),0; sin(g),cos(g),0; 0,0,1];

% 试一组角度：绕 Z 60°→绕 Y -45°→绕 X 30°
alpha = deg2rad(30); beta = deg2rad(-45); gamma = deg2rad(60);
R = Rz(gamma) * Ry(beta) * Rx(alpha);
fprintf('【欧拉角 ZYX(30,-45,60)° → 旋转矩阵】\n');
disp(round(R, 4));

%% ---------- 第 2 步：读 PUMA560 的 DH 参数表 ----------
% 每一行是 [a, α, d, θ]，θ 是变量（关节角）
dh = [
    0       0       0.672   0;       % Joint 1 — 腰部旋转
    0.432  -pi/2   0.149   0;       % Joint 2 — 大臂俯仰
    0.020   pi/2   0       0;       % Joint 3 — 小臂俯仰
    0       0       0.433   0;       % Joint 4 — 腕部旋转
    ];

fprintf('\n【PUMA560 DH 参数】\n');
fprintf('  J1(腰): a=%.3f  α=%.0f°      d=%.3f\n', dh(1,1), rad2deg(dh(1,2)), dh(1,3));
fprintf('  J2(肩): a=%.3f  α=%.0f°     d=%.3f\n', dh(2,1), rad2deg(dh(2,2)), dh(2,3));
fprintf('  J3(肘): a=%.3f  α=%.0f°      d=%.3f\n', dh(3,1), rad2deg(dh(3,2)), dh(3,3));
fprintf('  J4(腕): a=%.3f  α=%.0f°      d=%.3f\n\n', dh(4,1), rad2deg(dh(4,2)), dh(4,3));

%% ---------- 第 3 步：Simulink 模型 — DH 正运动学 ----------

mdl = 'tutorial26_kinematics';
addpath(fullfile(fileparts(mfilename('fullpath')), 'models'));
if bdIsLoaded(mdl), close_system(mdl, 1); end
new_system(mdl, 'Model');
open_system(mdl);

% 关节角输入 — 6 个 Constant 块，每个代表一个关节角度（度）
fprintf('【搭建 Simulink 模型】\n');
for i = 1:6
    add_block('simulink/Sources/Constant', [mdl '/theta' num2str(i)], ...
        'Position', [30, 30+45*(i-1), 90, 52+45*(i-1)]);
    set_param([mdl '/theta' num2str(i)], 'Value', '0');
    fprintf('  theta%d = 0° — 关节 %d 初始角度\n', i, i);
end

% Mux — 6 路信号合并成 1 个向量 [θ1;θ2;θ3;θ4;θ5;θ6]
add_block('simulink/Signal Routing/Mux', [mdl '/Mux6'], ...
    'Position', [140, 60, 160, 230]);
set_param([mdl '/Mux6'], 'Inputs', '6');

% MATLAB Function — DH 正运动学核心计算
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/DH_FK'], ...
    'Position', [250, 80, 330, 220]);

% 写入正运动学算法
sf = sfroot();
mc = sf.find('-isa', 'Stateflow.Machine', 'Name', mdl);
if ~isempty(mc)
    ch = mc.find('-isa', 'Stateflow.EMChart');
    if ~isempty(ch)
        ch.Script = sprintf(['function [x,y,z] = DH_FK(theta)\n' ...
            '%% PUMA560 正运动学：关节角 → 末端XYZ\n' ...
            'a     = [0; 0.4318; 0.0203; 0; 0; 0];\n' ...
            'alpha = [pi/2; 0; -pi/2; pi/2; -pi/2; 0];\n' ...
            'd     = [0.67183; 0; 0.15005; 0.4318; 0; 0.2];\n' ...
            'T = eye(4);\n' ...
            'for i = 1:6\n' ...
            '    ct = cos(theta(i)); st = sin(theta(i));\n' ...
            '    ca = cos(alpha(i)); sa = sin(alpha(i));\n' ...
            '    T = T * [ct, -st*ca,  st*sa, a(i)*ct;\n' ...
            '             st,  ct*ca, -ct*sa, a(i)*st;\n' ...
            '              0,     sa,     ca,    d(i);\n' ...
            '              0,      0,      0,       1];\n' ...
            'end\n' ...
            'x = T(1,4); y = T(2,4); z = T(3,4);\n']);
    end
end

% 末端 XYZ 位置显示
labels = {'X', 'Y', 'Z'};
for i = 1:3
    add_block('simulink/Sinks/Display', [mdl '/Display_' labels{i}], ...
        'Position', [430, 60+45*(i-1), 510, 82+45*(i-1)]);
    set_param([mdl '/Display_' labels{i}], 'Format', 'short');
end

% ---------- 连线 ----------
for i = 1:6
    add_line(mdl, ['theta' num2str(i) '/1'], ['Mux6/' num2str(i)]);
end
add_line(mdl, 'Mux6/1', 'DH_FK/1');              % 6 路角度 → FK
add_line(mdl, 'DH_FK/1', 'Display_X/1');          % X → 显示
add_line(mdl, 'DH_FK/2', 'Display_Y/1');          % Y → 显示
add_line(mdl, 'DH_FK/3', 'Display_Z/1');          % Z → 显示

% 标注
ann = Simulink.Annotation([mdl '/Title']);
ann.Position = [180, 8];
ann.Text = 'PUMA560 DH正运动学 — 修改theta1~6看末端XYZ变化';

fprintf('  模型搭建完成：6 个角度 → Mux → DH_FK → XYZ 显示\n\n');

%% ---------- 第 4 步：验证 — 给定一组角度，手动算 XYZ ----------
theta_test = deg2rad([20; -30; 15; 0; 10; -25]);

% 手动 DH 变换矩阵连乘
a_all     = [0; 0.4318; 0.0203; 0; 0; 0];
alpha_all = [pi/2; 0; -pi/2; pi/2; -pi/2; 0];
d_all     = [0.67183; 0; 0.15005; 0.4318; 0; 0.2];
T = eye(4);
for i = 1:6
    ct = cos(theta_test(i)); st = sin(theta_test(i));
    ca = cos(alpha_all(i)); sa = sin(alpha_all(i));
    T = T * [ct, -st*ca,  st*sa, a_all(i)*ct;
             st,  ct*ca, -ct*sa, a_all(i)*st;
             0,      sa,     ca,    d_all(i);
             0,       0,      0,       1];
end
pos = T(1:3, 4);
fprintf('【手动验证】θ = [20,-30,15,0,10,-25]°\n');
fprintf('  末端位置: X=%.3f  Y=%.3f  Z=%.3f m\n\n', pos);

%% ---------- 第 5 步：工作空间可视化 ----------
figure('Name', 't26: PUMA560 工作空间', 'Position', [50, 50, 800, 600]);
hold on;
pts = [];
% 扫描前 3 个关节（后 3 个不影响位置，只影响姿态）
for i1 = 1:6
    for i2 = 1:6
        for i3 = 1:4
            th = [deg2rad(-30+15*i1); deg2rad(-45+18*i2); deg2rad(-20+13*i3); 0; 0; 0];
            T = eye(4);
            for k = 1:6
                ct = cos(th(k)); st = sin(th(k));
                ca = cos(alpha_all(k)); sa = sin(alpha_all(k));
                T = T * [ct,-st*ca,st*sa,a_all(k)*ct;
                         st,ct*ca,-ct*sa,a_all(k)*st;
                         0,sa,ca,d_all(k);
                         0,0,0,1];
            end
            pts(end+1, :) = T(1:3, 4)';
        end
    end
end
scatter3(pts(:,1), pts(:,2), pts(:,3), 20, 'b', 'filled', 'MarkerFaceAlpha', 0.2);
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('PUMA560 可达工作空间（前 3 关节扫描，144 个姿态）');
grid on; view(3); axis equal;

fprintf('========================================\n');
fprintf('  教程 26 完成！\n');
fprintf('========================================\n\n');
fprintf('动手实验：\n');
fprintf('  1. 双击打开 tutorial26_kinematics.slx\n');
fprintf('  2. 修改 theta1~theta6 的 Constant 值\n');
fprintf('  3. 运行仿真，观察 Display 中 XYZ 的变化\n');
fprintf('  4. 试试 theta=[0,0,0,0,0,0] — 机械臂完全伸直\n');
fprintf('  5. 观察工作空间图：这是一台 PUMA560 能到达的所有位置\n');

save_system(mdl, fullfile(fileparts(mfilename('fullpath')), 'models', [mdl '.slx']));
close_system(mdl, 0);
