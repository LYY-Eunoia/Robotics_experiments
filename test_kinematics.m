% =========================================================================
% 测试脚本：机器人正逆运动学
%
% 功能:
% 1. 定义机器人结构参数。
% 2. 逆运动学测试: 给定一个笛卡尔目标位姿，求解所有可能的关节角度。
% 3. 闭环测试: 给定一组关节角度，通过正运动学计算位姿，
%    再通过逆运动学求解角度，验证解的正确性。
% =========================================================================

clear;
clc;
close all;

%% 1. 定义机器人结构参数 (与 ikine6s_solver.m 中保持一致)
% -------------------------------------------------------------------------
% 连杆长度 (根据 ikine6s_solver.m 中的参考点和计算逻辑推断)
config.L1 = 491;  % 基座到关节2的高度
config.L2 = 450;  % 关节2到关节3的连杆长度 (941-491)
config.L3 = 450;  % 关节3到腕部参考点的连杆长度 (1391-941)
config.L4 = 84;   % 腕部参考点到末端法兰的长度 (1475-1391)，未使用但保留

% 机器人基准参考位姿 g0 (末端法兰相对于基座的零角度位姿)
g0 = [-1, 0, 0, 0;
      0, -1, 0, 0;
      0, 0, 1, 1475;
      0, 0, 0, 1];

% 各关节运动旋量 Xi (Screw Axes)
w0 = [0,0,0,0,0,0; 0,1,1,0,1,0; 1,0,0,1,0,1];      % 角速度分量
q0 = [0,0,0,0,0,0; 0,0,0,0,0,0; 0,491,941,941,1391,1475]; % 轴上点位置
Xi = zeros(6,6);
for i = 1:6
    w_i = w0(1:3,i);
    q_i = q0(1:3,i);
    v_i = -cross(w_i, q_i); % 线速度分量 v = -w x q
    Xi(:,i) = [v_i; w_i];
end

fprintf('机器人参数定义完成。\n\n');

%% 2. 逆运动学测试
% -------------------------------------------------------------------------
fprintf('--- 任务1: 逆运动学测试 ---\n');
% 给定一个期望的末端执行器目标位姿 T_target
% 例如，一个位于(300, 400, 1000)并有一定姿态的位姿
R_target = [0, 1, 0; 
            1, 0, 0; 
            0, 0, -1];
p_target = [300; 400; 1000];
T_target = [R_target, p_target; 0, 0, 0, 1];

fprintf('给定的目标位姿 T_target:\n');
disp(T_target);

% 使用 Ikine6s 求解逆运动学
fprintf('调用 Ikine6s 求解关节角度...\n');
theta_solutions = Ikine6s(T_target, config);

if isempty(theta_solutions)
    fprintf('对于给定的目标位姿，未找到有效的逆运动学解。\n');
else
    fprintf('成功找到 %d 组关节角度解 (单位: rad):\n', size(theta_solutions, 1));
    % 为了显示更友好，转换为角度制
    disp(rad2deg(theta_solutions));
end
fprintf('---------------------------------\n\n');

%% 3. 正逆运动学闭环测试
% -------------------------------------------------------------------------
fprintf('--- 任务2: 正逆运动学闭环测试 ---\n');
% 给定一组关节角度 (单位: rad)
theta_given = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6];

fprintf('给定的初始关节角度 (rad):\n');
disp(theta_given);
fprintf('给定的初始关节角度 (deg):\n');
disp(rad2deg(theta_given));

% (a) 正运动学: 根据给定角度计算末端位姿
fprintf('\n步骤 (a): 使用 Fkine 计算末端位姿...\n');
T_calculated = Fkine(Xi, theta_given, g0);
fprintf('计算得到的末端位姿 T_calculated:\n');
disp(T_calculated);

% (b) 逆运动学: 将计算出的位姿作为目标，反解关节角度
fprintf('\n步骤 (b): 使用 Ikine6s 将 T_calculated 作为目标，反解关节角度...\n');
theta_restored_all = Ikine6s(T_calculated, config);

if isempty(theta_restored_all)
    fprintf('错误: 未能从正解计算出的位姿中找到任何逆解！\n');
else
    fprintf('成功找到 %d 组还原的关节角度解。\n', size(theta_restored_all, 1));
    
    % (c) 验证: 检查是否存在一组解与初始角度一致
    fprintf('\n步骤 (c): 验证还原的解...\n');
    found_match = false;
    tolerance = 1e-6; % 设置一个小的容差
    for i = 1:size(theta_restored_all, 1)
        theta_restored = theta_restored_all(i, :);
        % 计算当前解与初始角度的差值
        diff = abs(theta_restored - theta_given);
        % 处理角度的周期性 (例如 2*pi 的差异)
        diff = min(diff, 2*pi - diff); 
        
        if all(diff < tolerance)
            fprintf('成功! 在第 %d 组解中找到了匹配项。\n', i);
            fprintf('还原的关节角度 (deg):\n');
            disp(rad2deg(theta_restored));
            fprintf('与初始角度误差很小，闭环测试通过！\n');
            found_match = true;
            break; % 找到匹配后即可退出循环
        end
    end
    
    if ~found_match
        fprintf('警告: 在所有解中均未找到与初始角度完全匹配的项。\n');
        fprintf('这可能由数值精度问题或函数实现中的微小偏差导致。\n');
        fprintf('以下是所有还原的解 (deg) 供您检查:\n');
        disp(rad2deg(theta_restored_all));
    end
end
fprintf('---------------------------------\n');