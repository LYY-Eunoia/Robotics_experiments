function GenerateJointTrajectory()
    clc; close all;

    %% ========== CONFIGURATION (Modify as needed) ====================
    % If you get "No valid IK solution" errors, try:
    % 1. Run FixNotePositionsIK() to find the right settings
    % 2. Adjust the parameters below based on recommendations
    
    % Euler angle convention for note.txt
    % Options: 'ZYX', 'XYZ', 'ZXY', 'YXZ', 'XZY', 'YZX'
    euler_convention = 'ZYX';  % Default: ZYX (Roll-Pitch-Yaw)
    
    % Fixed orientation (if needed)
    % Set to [] to use orientations from note.txt
    % Set to [rx, ry, rz] to use fixed orientation for all notes
    % Example: [0, 180, 0] for downward pointing
    use_fixed_orientation = [];  % Default: use note.txt orientations
    
    %% ========== Step 1: Load xylophone note positions and musical score ============
    fprintf("=== Xylophone Playing Project: Twinkle Twinkle Little Star ===\n\n");
    
    % Load note positions from note.txt
    note_file = 'note.txt';
    if ~exist(note_file, 'file')
        error('Note file not found: %s', note_file);
    end
    
    % Load with specified configuration
    if isempty(use_fixed_orientation)
        note_data = LoadNotePositions(note_file, 'EulerConvention', euler_convention);
    else
        note_data = LoadNotePositions(note_file, 'UseFixedOrientation', use_fixed_orientation);
        fprintf('Using fixed orientation: [%.1f, %.1f, %.1f] degrees\n', use_fixed_orientation);
    end
    
    % Get musical score
    [note_sequence, time_intervals] = GetTwinkleTwinkleScore();
    num_notes = length(note_sequence);
    
    fprintf("\n=== Musical Score ===\n");
    fprintf("Number of notes: %d\n", num_notes);
    fprintf("Total song duration: %.2f seconds\n\n", sum(time_intervals));

    %% ========== Step 2: Robot configuration ====================
    config = LoadRobotConfig();
    
    % Parameters for motion planning
    sample_time = 0.001;  % 1ms sampling
    strike_angle = deg2rad(90);  % Joint 5 rotation angle for striking
    strike_duration = 0.1;  % Strike action duration (seconds)
    
    %% ========== Step 3: Find note positions and compute IK for each note ==========
    fprintf("=== Computing inverse kinematics for each note ===\n");
    
    % Storage for note configurations
    note_configs = cell(num_notes, 1);
    note_transforms = cell(num_notes, 1);
    
    for i = 1:num_notes
        note_name = note_sequence{i};
        
        % Find note data
        note_idx = find(strcmp({note_data.name}, note_name), 1);
        if isempty(note_idx)
            error('Note not found in database: %s', note_name);
        end
        
        % Get transformation matrix for this note
        T_target = note_data(note_idx).transform;
        note_transforms{i} = T_target;
        
        % Compute inverse kinematics
        theta_all = Ikine6s(T_target, config);
        if isempty(theta_all)
            error('No IK solution found for note %s (index %d)', note_name, i);
        end
        
        % Filter by joint limits
        theta_valid = FilterJointLimits(theta_all);
        if isempty(theta_valid)
            error('No valid IK solution within joint limits for note %s', note_name);
        end
        
        note_configs{i} = theta_valid;
        fprintf("  Note %d (%s): %d valid IK solutions\n", i, note_name, size(theta_valid, 1));
    end
    
    %% ========== Step 4: Select IK solutions for smooth trajectory ==========
    fprintf("\n=== Selecting optimal IK solutions ===\n");
    
    % For first note, let user choose
    fprintf("First note: %s\n", note_sequence{1});
    fprintf("Available solutions (degrees):\n");
    disp(rad2deg(note_configs{1}));
    
    M = size(note_configs{1}, 1);
    selected_idx = input(sprintf('Select solution number (1-%d): ', M));
    if selected_idx < 1 || selected_idx > M
        error('Invalid selection!');
    end
    
    % Selected configurations for each note
    selected_configs = zeros(num_notes, 6);
    selected_configs(1, :) = note_configs{1}(selected_idx, :);
    
    % For subsequent notes, choose solution closest to previous
    for i = 2:num_notes
        theta_prev = selected_configs(i-1, :);
        theta_candidates = note_configs{i};
        
        % Find closest solution
        distances = vecnorm(theta_candidates - theta_prev, 2, 2);
        [~, min_idx] = min(distances);
        
        selected_configs(i, :) = theta_candidates(min_idx, :);
    end
    
    fprintf("IK solutions selected for all %d notes\n", num_notes);

    %% ========== Step 5: Generate complete trajectory with striking actions ==========
    fprintf("\n=== Generating trajectory with striking actions ===\n");
    
    % Initialize trajectory storage
    all_t = [];
    all_q = [];
    all_qd = [];
    all_qdd = [];
    
    time_offset = 0;
    
    for i = 1:num_notes
        fprintf("  Planning note %d/%d (%s)...\n", i, num_notes, note_sequence{i});
        
        % Current note configuration
        q_note = selected_configs(i, :);
        
        % 1. Plan motion to note position (if not first note)
        if i > 1
            q_prev = selected_configs(i-1, :);
            motion_duration = time_intervals(i-1) - strike_duration * 2;  % Reserve time for strike
            
            if motion_duration > 0.05  % Minimum motion time
                [t_motion, q_motion, qd_motion, qdd_motion] = ...
                    PlanJointSpaceMotion(q_prev, q_note, motion_duration, sample_time);
                
                % Adjust time
                t_motion = t_motion + time_offset;
                
                % Append (skip first point to avoid duplication)
                if ~isempty(all_t)
                    all_t = [all_t; t_motion(2:end)];
                    all_q = [all_q; q_motion(2:end, :)];
                    all_qd = [all_qd; qd_motion(2:end, :)];
                    all_qdd = [all_qdd; qdd_motion(2:end, :)];
                else
                    all_t = t_motion;
                    all_q = q_motion;
                    all_qd = qd_motion;
                    all_qdd = qdd_motion;
                end
                
                time_offset = time_offset + motion_duration;
            end
        end
        
        % 2. Plan striking action at note position
        [t_strike, q_strike, qd_strike, qdd_strike] = ...
            PlanStrikeAction(q_note, config, strike_angle, strike_duration, sample_time);
        
        % Adjust time
        t_strike = t_strike + time_offset;
        
        % Append
        if isempty(all_t)
            all_t = t_strike;
            all_q = q_strike;
            all_qd = qd_strike;
            all_qdd = qdd_strike;
        else
            all_t = [all_t; t_strike(2:end)];
            all_q = [all_q; q_strike(2:end, :)];
            all_qd = [all_qd; qd_strike(2:end, :)];
            all_qdd = [all_qdd; qdd_strike(2:end, :)];
        end
        
        time_offset = time_offset + strike_duration * 2;
    end
    
    % Final trajectory data
    t_clean = all_t;
    q_clean = all_q;
    qd_clean = all_qd;
    qdd_clean = all_qdd;
    N_clean = length(t_clean);
    
    fprintf("\nTrajectory generated: %d time points, %.2f seconds total\n", N_clean, t_clean(end));

    %% ========== Step 6: Visualization - Joint Angles =====================
    fprintf("\n=== Generating visualizations ===\n");
    
    figure('Name','Xylophone Playing - Joint Angles','Position',[200 200 1200 800]);
    for j = 1:6
        subplot(3,2,j);
        plot(t_clean, rad2deg(q_clean(:,j)), 'LineWidth',1.5);
        xlabel('Time (s)');
        ylabel(sprintf('Joint %d (deg)', j));
        title(sprintf('Joint %d angle vs time', j), 'FontWeight','bold');
        grid on;
    end
    sgtitle('Xylophone Playing - Joint Space Trajectory (Twinkle Twinkle Little Star)');
    fprintf("Joint angle plots generated.\n");

    %% ========== Step 7: Visualization - Velocities ============
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

    %% ========== Step 8: Visualization - Accelerations ============
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
    
    %% ========== Step 9: Visualization - Musical Score Timeline ============
    figure('Name','Musical Score Timeline','Position',[200 200 1200 400]);
    
    % Calculate approximate time of each note strike
    note_times = zeros(num_notes, 1);
    time_acc = 0;
    for i = 1:num_notes
        if i > 1
            motion_dur = time_intervals(i-1) - strike_duration * 2;
            if motion_dur > 0.05
                time_acc = time_acc + motion_dur;
            end
        end
        note_times(i) = time_acc + strike_duration;  % Middle of strike
        time_acc = time_acc + strike_duration * 2;
    end
    
    % Plot note timeline
    subplot(2,1,1);
    hold on;
    for i = 1:num_notes
        plot([note_times(i), note_times(i)], [0, 1], 'b-', 'LineWidth', 2);
        text(note_times(i), 1.1, note_sequence{i}, 'HorizontalAlignment', 'center', 'FontSize', 8);
    end
    hold off;
    ylim([-0.5, 1.5]);
    xlim([0, max(note_times) + 1]);
    xlabel('Time (s)');
    ylabel('Note Events');
    title('Musical Score Timeline', 'FontWeight', 'bold');
    grid on;
    
    % Plot joint 5 angle (striking joint)
    subplot(2,1,2);
    plot(t_clean, rad2deg(q_clean(:,5)), 'r-', 'LineWidth', 1.5);
    xlabel('Time (s)');
    ylabel('Joint 5 Angle (deg)');
    title('Striking Joint (Joint 5) Motion', 'FontWeight', 'bold');
    grid on;
    
    sgtitle('Xylophone Performance - Twinkle Twinkle Little Star');

    %% ========== Step 10: Save trajectory data ============================
    fprintf("\n=== Saving trajectory data ===\n");
    
    % Save as MAT file
    save('joint_traj_data.mat', 't_clean', 'q_clean', 'qd_clean', 'qdd_clean', ...
         'note_sequence', 'time_intervals', 'selected_configs');
    fprintf("Trajectory data saved to joint_traj_data.mat\n");

    % Save as CSV: t | q1..q6 | qd1..qd6 | qdd1..qdd6
    data_export = [t_clean, q_clean, qd_clean, qdd_clean];
    writematrix(data_export, 'joint_traj_data.csv');
    fprintf("Trajectory data saved to joint_traj_data.csv\n");
    
    % Save musical score information
    score_info = cell(num_notes, 3);
    for i = 1:num_notes
        score_info{i, 1} = note_sequence{i};
        score_info{i, 2} = note_times(i);
        score_info{i, 3} = time_intervals(i);
    end
    writecell(score_info, 'musical_score.csv');
    fprintf("Musical score information saved to musical_score.csv\n");
    
    fprintf("\n=== Trajectory generation complete! ===\n");
    fprintf("Total trajectory duration: %.2f seconds\n", t_clean(end));
    fprintf("Number of trajectory points: %d\n", N_clean);
    fprintf("Average sampling rate: %.4f seconds\n", mean(diff(t_clean)));
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
