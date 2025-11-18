% function GenerateJointTrajectory()
%     clc; close all;
% 
%     %% ========== Step 1：从 LFPB 文件中获取轨迹 ================
%     fprintf("正在生成笛卡尔轨迹（LFPB）...\n");
%     [all_t, all_x, all_y, all_z] = LFPB();   % ← 你已有的函数（保持不动）
% 
%     N = length(all_t);
%     fprintf("轨迹点数: %d\n", N);
% 
%     %% ========== Step 2：机器人参数（与你原来完全一致）=====
%     config = LoadRobotConfig();
% 
%     % 固定姿态（可修改）
%     R_target = [0 1 0; 
%                 1 0 0; 
%                 0 0 -1];
%     % R_target = [1 0 0; 
%     %             0 1 0; 
%     %             0 0 1];
% 
%     %% ========== Step 3：计算每个时刻的 IK ====================
%     fprintf("开始进行逆运动学计算...\n");
% 
%     joint_traj = zeros(N, 6);
%     last_valid_theta = [];
% 
%     for i = 1:N
%         p = [all_x(i); all_y(i); all_z(i)];
% 
%         % 构造末端位姿 T
%         T = [R_target, p; 0 0 0 1];
% 
%         % 求 IK（8 组）
%         theta_all = Ikine6s(T, config);
% 
%         % 若无解，填充 NaN
%         if isempty(theta_all)
%             joint_traj(i,:) = nan(1,6);
%             continue;
%         end
% 
%         % 过滤关节角限制
%         theta_valid = FilterJointLimits(theta_all);
% 
%         if isempty(theta_valid)
%             joint_traj(i,:) = nan(1,6);
%             continue;
%         end
% 
%         % 选取“最连续”的 IK 解：与上一帧最接近
%         if isempty(last_valid_theta)
%             theta_pick = theta_valid(1,:);   % 初始帧随便挑一个
%         else
%             [~, idx] = min(vecnorm(theta_valid - last_valid_theta, 2, 2));
%             theta_pick = theta_valid(idx,:);
%         end
% 
%         joint_traj(i,:) = theta_pick;
%         last_valid_theta = theta_pick;
%     end
% 
%     %% ========== Step 4：绘制 6 个关节角曲线 =================
%     figure('Name','Joint Angles','Position',[200 200 1200 800]);
%     for j = 1:6
%         subplot(3,2,j);
%         plot(all_t, rad2deg(joint_traj(:,j)), 'LineWidth',1.5);
%         xlabel('Time (s)');
%         ylabel(sprintf('Joint %d (deg)', j));
%         title(sprintf('Joint %d angle vs time', j), 'FontWeight','bold');
%         grid on;
%     end
% 
%     sgtitle('6 DOF Robot Joint Space Trajectory');
%     fprintf("绘制完成。\n");
% end
% 
% function config = LoadRobotConfig()
%     config.L1 = 491;
%     config.L2 = 450;
%     config.L3 = 450;
%     config.L4 = 84;
% 
%     w0 = [0,0,0,0,0,0; 0,1,1,0,1,0; 1,0,0,1,0,1];
%     q0 = [0,0,0,0,0,0; 0,0,0,0,0,0; 0,491,941,941,1391,1475];
% 
%     Xi = zeros(6,6);
%     for i = 1:6
%         w_i = w0(1:3,i);
%         q_i = q0(1:3,i);
%         v_i = -cross(w_i, q_i); 
%         Xi(:,i) = [v_i; w_i];
%     end
% 
%     config.Xi = Xi;
% end
% 
% function theta_valid = FilterJointLimits(theta_solutions)
%     theta_deg = rad2deg(theta_solutions);
% 
%     joint_limits = [
%         -170 170;
%         -120 120;
%         -140 140;
%         -170 170;
%         -120 120;
%         -360 360
%     ];
%    % joint_limits = [
%    %  -360 360;
%    %  -360 360;
%    %  -360 360;
%    %  -360 360;
%    %  -360 360;
%    %  -360 360;
%    %  ];
% 
%     valid_mask = true(size(theta_deg,1),1);
% 
%     for i = 1:6
%         valid_mask = valid_mask & ...
%             theta_deg(:,i) >= joint_limits(i,1) & ...
%             theta_deg(:,i) <= joint_limits(i,2);
%     end
% 
%     theta_valid = theta_solutions(valid_mask,:);
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function GenerateJointTrajectory_MultiStart_JointOnly()
%     clc; close all;
% 
%     %% ========== Step 1：获取笛卡尔轨迹（LFPB） ============
%     fprintf("正在生成笛卡尔轨迹（LFPB）...\n");
%     [all_t, all_x, all_y, all_z] = LFPB();  
% 
%     N = length(all_t);
%     fprintf("轨迹点数: %d\n", N);
% 
%     %% ========== Step 2：机器人参数 ====================
%     config = LoadRobotConfig();
%     R_target = [0 1 0; 1 0 0; 0 0 -1];
% 
%     %% ========== Step 3：计算第一帧所有有效 IK 解 ==========
%     p0 = [all_x(1); all_y(1); all_z(1)];
%     T0 = [R_target, p0; 0 0 0 1];
% 
%     theta_all_first = Ikine6s(T0, config);
%     theta_valid_first = FilterJointLimits(theta_all_first);
% 
%     M = size(theta_valid_first,1);  % 第一帧有效解数量
%     fprintf("第一帧有效解数: %d\n", M);
%     if M == 0
%         error('第一帧无有效 IK 解，无法生成轨迹');
%     end
% 
%     %% ========== Step 4：针对每组初始解生成关节角轨迹 =========
%     joint_trajs = nan(N, 6, M);  % 存储多条轨迹
% 
%     for m = 1:M
%         last_valid_theta = theta_valid_first(m,:);
%         joint_trajs(1,:,m) = last_valid_theta;
% 
%         for i = 2:N
%             p = [all_x(i); all_y(i); all_z(i)];
%             T = [R_target, p; 0 0 0 1];
% 
%             theta_all = Ikine6s(T, config);
%             if isempty(theta_all)
%                 joint_trajs(i,:,m) = nan(1,6);
%                 continue;
%             end
%             theta_valid = FilterJointLimits(theta_all);
%             if isempty(theta_valid)
%                 joint_trajs(i,:,m) = nan(1,6);
%                 continue;
%             end
% 
%             % 选取与上一帧最接近的解
%             [~, idx] = min(vecnorm(theta_valid - last_valid_theta, 2, 2));
%             theta_pick = theta_valid(idx,:);
%             joint_trajs(i,:,m) = theta_pick;
%             last_valid_theta = theta_pick;
%         end
%     end
% 
%     %% ========== Step 5：绘制关节角曲线 =====================
%     figure('Name','Joint Angles','Position',[200 200 1200 800]);
%     colors = lines(M);  % 不同轨迹不同颜色
% 
%     for j = 1:6
%         subplot(3,2,j); hold on; grid on;
%         for m = 1:M
%             plot(all_t, rad2deg(joint_trajs(:,j,m)), 'LineWidth',1.5, 'Color', colors(m,:), ...
%                  'DisplayName', sprintf('轨迹 %d', m));
%         end
%         xlabel('Time (s)');
%         ylabel(sprintf('Joint %d (deg)', j));
%         title(sprintf('Joint %d angle vs time', j), 'FontWeight','bold');
%         if j==1
%             legend show;
%         end
%     end
%     sgtitle('6 DOF Robot Joint Space Trajectory (Multiple Starts)');
%     fprintf("绘制完成。\n");
% end
% 
% function config = LoadRobotConfig()
%     config.L1 = 491;
%     config.L2 = 450;
%     config.L3 = 450;
%     config.L4 = 84;
% 
%     w0 = [0,0,0,0,0,0; 0,1,1,0,1,0; 1,0,0,1,0,1];
%     q0 = [0,0,0,0,0,0; 0,0,0,0,0,0; 0,491,941,941,1391,1475];
% 
%     Xi = zeros(6,6);
%     for i = 1:6
%         w_i = w0(1:3,i);
%         q_i = q0(1:3,i);
%         v_i = -cross(w_i, q_i); 
%         Xi(:,i) = [v_i; w_i];
%     end
% 
%     config.Xi = Xi;
% end
% 
% function theta_valid = FilterJointLimits(theta_solutions)
%     theta_deg = rad2deg(theta_solutions);
% 
%     joint_limits = [
%         -170 170;
%         -120 120;
%         -140 140;
%         -170 170;
%         -120 120;
%         -360 360
%     ];
%    % joint_limits = [
%    %  -360 360;
%    %  -360 360;
%    %  -360 360;
%    %  -360 360;
%    %  -360 360;
%    %  -360 360;
%    %  ];
% 
%     valid_mask = true(size(theta_deg,1),1);
% 
%     for i = 1:6
%         valid_mask = valid_mask & ...
%             theta_deg(:,i) >= joint_limits(i,1) & ...
%             theta_deg(:,i) <= joint_limits(i,2);
%     end
% 
%     theta_valid = theta_solutions(valid_mask,:);
% end

function GenerateJointTrajectory()
    clc; close all;

    %% ========== Step 1：获取笛卡尔轨迹（LFPB） ============
    fprintf("正在生成笛卡尔轨迹（LFPB）...\n");
    [all_t, all_x, all_y, all_z] = LFPB();  

    N = length(all_t);
    fprintf("轨迹点数: %d\n", N);

    %% ========== Step 2：机器人参数 ====================
    config = LoadRobotConfig();
    R_target = [0 1 0; 1 0 0; 0 0 -1];

    %% ========== Step 3：计算第一帧所有有效 IK 解 ==========
    p0 = [all_x(1); all_y(1); all_z(1)];
    T0 = [R_target, p0; 0 0 0 1];

    theta_all_first = Ikine6s(T0, config);
    theta_valid_first = FilterJointLimits(theta_all_first);

    M = size(theta_valid_first,1);  
    fprintf("第一帧有效解数: %d\n", M);
    if M == 0
        error('第一帧无有效 IK 解，无法生成轨迹');
    end

    % --- 显示所有第一帧解，供用户选择 ---
    disp('第一帧有效解列表（单位：deg）:');
    disp(rad2deg(theta_valid_first));
    selected_idx = input(sprintf('请选择第一帧解的编号（1 ~ %d）：', M));
    if selected_idx < 1 || selected_idx > M
        error('选择编号无效！');
    end

    last_valid_theta = theta_valid_first(selected_idx,:);
    joint_traj = nan(N,6);
    joint_traj(1,:) = last_valid_theta;

    %% ========== Step 4：生成轨迹 ====================
    for i = 2:N
        p = [all_x(i); all_y(i); all_z(i)];
        T = [R_target, p; 0 0 0 1];

        theta_all = Ikine6s(T, config);
        if isempty(theta_all)
            joint_traj(i,:) = nan(1,6);
            continue;
        end
        theta_valid = FilterJointLimits(theta_all);
        if isempty(theta_valid)
            joint_traj(i,:) = nan(1,6);
            continue;
        end

        [~, idx] = min(vecnorm(theta_valid - last_valid_theta, 2, 2));
        theta_pick = theta_valid(idx,:);
        joint_traj(i,:) = theta_pick;
        last_valid_theta = theta_pick;
    end

    %% ========== Step 5：绘制关节角曲线 =====================
    figure('Name','Joint Angles','Position',[200 200 1200 800]);
    for j = 1:6
        subplot(3,2,j);
        plot(all_t, rad2deg(joint_traj(:,j)), 'LineWidth',1.5);
        xlabel('Time (s)');
        ylabel(sprintf('Joint %d (deg)', j));
        title(sprintf('Joint %d angle vs time', j), 'FontWeight','bold');
        grid on;
    end
    sgtitle(sprintf('6 DOF Robot Joint Space Trajectory (Start #%d)', selected_idx));
    fprintf("绘制完成。\n");
end

function config = LoadRobotConfig()
    config.L1 = 491;
    config.L2 = 450;
    config.L3 = 450;
    config.L4 = 84;

    w0 = [0,0,0,0,0,0; 0,1,1,0,1,0; 1,0,0,1,0,1];
    q0 = [0,0,0,0,0,0; 0,0,0,0,0,0; 0,491,941,941,1391,1475];

    Xi = zeros(6,6);
    for i = 1:6
        w_i = w0(1:3,i);
        q_i = q0(1:3,i);
        v_i = -cross(w_i, q_i); 
        Xi(:,i) = [v_i; w_i];
    end

    config.Xi = Xi;
end

function theta_valid = FilterJointLimits(theta_solutions)
    theta_deg = rad2deg(theta_solutions);

    joint_limits = [
        -170 170;
        -120 120;
        -140 140;
        -170 170;
        -120 120;
        -360 360
    ];
   % joint_limits = [
   %  -360 360;
   %  -360 360;
   %  -360 360;
   %  -360 360;
   %  -360 360;
   %  -360 360;
   %  ];

    valid_mask = true(size(theta_deg,1),1);

    for i = 1:6
        valid_mask = valid_mask & ...
            theta_deg(:,i) >= joint_limits(i,1) & ...
            theta_deg(:,i) <= joint_limits(i,2);
    end

    theta_valid = theta_solutions(valid_mask,:);
end