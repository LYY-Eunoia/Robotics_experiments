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

    %% ========== Step 4：生成关节角轨迹 ====================
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

        % 与上一帧最接近的解
        [~, idx] = min(vecnorm(theta_valid - last_valid_theta, 2, 2));
        theta_pick = theta_valid(idx,:);
        joint_traj(i,:) = theta_pick;
        last_valid_theta = theta_pick;
    end

    %% ========== Step 5：数值计算 qd, qdd ========================
    % joint_traj: N x 6, all_t: 1 x N (行向量)
    all_t = all_t(:);           % 转成列向量 N x 1
    q = joint_traj;             % N x 6

    % 先处理：剔除含 NaN 的时间点
    nan_rows = any(isnan(q), 2);   % 有任意一个关节是 NaN 的行
    if any(nan_rows)
        fprintf('警告：有 %d 个采样点无 IK 解，将被删除。\n', sum(nan_rows));
    end
    t_clean = all_t(~nan_rows);
    q_clean = q(~nan_rows, :);
    N_clean = length(t_clean);

    % 数值计算速度 qd（中心差分为主，首尾用前向/后向差分）
    qd_clean = zeros(N_clean, 6);
    % 首点：前向差分
    dt1 = t_clean(2) - t_clean(1);
    qd_clean(1,:) = (q_clean(2,:) - q_clean(1,:)) / dt1;
    % 中间点：中心差分
    for k = 2:N_clean-1
        dt = t_clean(k+1) - t_clean(k-1);
        qd_clean(k,:) = (q_clean(k+1,:) - q_clean(k-1,:)) / dt;
    end
    % 末点：后向差分
    dtN = t_clean(N_clean) - t_clean(N_clean-1);
    qd_clean(N_clean,:) = (q_clean(N_clean,:) - q_clean(N_clean-1,:)) / dtN;

    % 数值计算加速度 qdd（对 qd 再做一次差分）
    qdd_clean = zeros(N_clean, 6);
    % 首点
    dt1 = t_clean(2) - t_clean(1);
    qdd_clean(1,:) = (qd_clean(2,:) - qd_clean(1,:)) / dt1;
    % 中间点
    for k = 2:N_clean-1
        dt = t_clean(k+1) - t_clean(k-1);
        qdd_clean(k,:) = (qd_clean(k+1,:) - qd_clean(k-1,:)) / dt;
    end
    % 末点
    dtN = t_clean(N_clean) - t_clean(N_clean-1);
    qdd_clean(N_clean,:) = (qd_clean(N_clean,:) - qd_clean(N_clean-1,:)) / dtN;

    %% ========== Step 6：绘制关节角曲线 =====================
    figure('Name','Joint Angles','Position',[200 200 1200 800]);
    for j = 1:6
        subplot(3,2,j);
        plot(t_clean, rad2deg(q_clean(:,j)), 'LineWidth',1.5);
        xlabel('Time (s)');
        ylabel(sprintf('Joint %d (deg)', j));
        title(sprintf('Joint %d angle vs time', j), 'FontWeight','bold');
        grid on;
    end
    sgtitle(sprintf('6 DOF Robot Joint Space Trajectory (Start #%d)', selected_idx));
    fprintf("关节角绘制完成。\n");

    %% ========== Step 7：绘制速度和加速度（可选） ============
    figure('Name','Joint Velocities','Position',[200 200 1200 800]);
    for j = 1:6
        subplot(3,2,j);
        plot(t_clean, qd_clean(:,j), 'LineWidth',1.5);
        xlabel('Time (s)');
        ylabel(sprintf('dJoint %d (rad/s)', j));
        title(sprintf('Joint %d velocity', j), 'FontWeight','bold');
        grid on;
    end
    sgtitle('Joint Velocities');

    figure('Name','Joint Accelerations','Position',[200 200 1200 800]);
    for j = 1:6
        subplot(3,2,j);
        plot(t_clean, qdd_clean(:,j), 'LineWidth',1.5);
        xlabel('Time (s)');
        ylabel(sprintf('ddJoint %d (rad/s^2)', j));
        title(sprintf('Joint %d acceleration', j), 'FontWeight','bold');
        grid on;
    end
    sgtitle('Joint Accelerations');

    %% ========== Step 8：保存数据 ============================
    % 保存为 MAT 文件
    save('joint_traj_data.mat', 't_clean', 'q_clean', 'qd_clean', 'qdd_clean');
    fprintf("轨迹数据已保存到 joint_traj_data.mat\n");

    % 同时保存为 CSV：t | q1..q6 | qd1..qd6 | qdd1..qdd6
    data_export = [t_clean, q_clean, qd_clean, qdd_clean];
    writematrix(data_export, 'joint_traj_data.csv');
    fprintf("轨迹数据已保存到 joint_traj_data.csv\n");
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

    valid_mask = true(size(theta_deg,1),1);

    for i = 1:6
        valid_mask = valid_mask & ...
            theta_deg(:,i) >= joint_limits(i,1) & ...
            theta_deg(:,i) <= joint_limits(i,2);
    end

    theta_valid = theta_solutions(valid_mask,:);
end