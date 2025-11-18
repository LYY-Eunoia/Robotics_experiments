function FixNotePositionsIK()
    % FixNotePositionsIK - Comprehensive solution for IK failures with real robot data
    %
    % This function helps fix "No valid IK solution within joint limits" errors
    % by trying different approaches:
    %   1. Different Euler angle conventions
    %   2. Position-only with fixed orientation
    %   3. Adjusted orientations
    %
    % Usage:
    %   FixNotePositionsIK()  % Interactive mode
    
    clc;
    fprintf('=======================================================\n');
    fprintf('    IK Problem Solver for Xylophone Note Positions    \n');
    fprintf('=======================================================\n\n');
    
    % Load robot configuration
    config = LoadRobotConfig();
    
    % Check if note.txt exists
    note_file = 'note.txt';
    if ~exist(note_file, 'file')
        error('Note file not found: %s', note_file);
    end
    
    fprintf('Step 1: Analyzing current note.txt...\n\n');
    
    % Try to load with default settings
    try
        note_data = LoadNotePositions(note_file);
        fprintf('Successfully loaded %d notes\n\n', length(note_data));
    catch ME
        fprintf('Error loading notes: %s\n', ME.message);
        return;
    end
    
    % Test IK for first note with different approaches
    fprintf('Step 2: Testing different solutions for first note (%s)...\n\n', note_data(1).name);
    
    solutions_found = false;
    
    %% Approach 1: Try different Euler conventions
    fprintf('Approach 1: Testing different Euler angle conventions\n');
    conventions = {'ZYX', 'XYZ', 'ZXY', 'YXZ', 'XZY', 'YZX'};
    
    for i = 1:length(conventions)
        conv = conventions{i};
        try
            % Read first note data
            fid = fopen(note_file, 'r');
            while ~feof(fid)
                line = fgetl(fid);
                if ischar(line) && ~isempty(strtrim(line)) && line(1) ~= '%'
                    parts = strsplit(strtrim(line), ',');
                    if length(parts) == 7
                        x = str2double(parts{2});
                        y = str2double(parts{3});
                        z = str2double(parts{4});
                        rx = str2double(parts{5});
                        ry = str2double(parts{6});
                        rz = str2double(parts{7});
                        break;
                    end
                end
            end
            fclose(fid);
            
            % Create rotation matrix with this convention
            R = EulerToRotation(rx, ry, rz, conv);
            p = [x; y; z];
            T = [R, p; 0, 0, 0, 1];
            
            % Test IK
            theta_all = Ikine6s(T, config);
            if ~isempty(theta_all)
                theta_valid = FilterJointLimits(theta_all);
                if ~isempty(theta_valid)
                    fprintf('  ✓ %s: Found %d valid solutions\n', conv, size(theta_valid, 1));
                    solutions_found = true;
                    recommended_convention = conv;
                else
                    fprintf('  ✗ %s: %d solutions, but none within joint limits\n', conv, size(theta_all, 1));
                end
            else
                fprintf('  ✗ %s: No IK solutions\n', conv);
            end
        catch
            fprintf('  ✗ %s: Error\n', conv);
        end
    end
    
    if solutions_found
        fprintf('\n✓ SOLUTION FOUND!\n');
        fprintf('Use Euler convention: %s\n\n', recommended_convention);
        fprintf('To apply this fix:\n');
        fprintf('1. Replace LoadNotePositions.m with LoadNotePositions_v2.m\n');
        fprintf('2. In GenerateJointTrajectory.m, change:\n');
        fprintf('   note_data = LoadNotePositions(note_file);\n');
        fprintf('   to:\n');
        fprintf('   note_data = LoadNotePositions(note_file, ''EulerConvention'', ''%s'');\n\n', recommended_convention);
        return;
    end
    
    %% Approach 2: Try position-only with fixed orientations
    fprintf('\nApproach 2: Testing position-only with fixed orientations\n');
    
    fixed_orientations = {
        [0, 180, 0], 'Downward (standard)';
        [180, 0, 180], 'Downward (rotated)';
        [0, 0, 180], 'Downward (alternative)';
        [90, 90, 0], 'Angled approach 1';
        [0, 90, 0], 'Horizontal'
    };
    
    for i = 1:size(fixed_orientations, 1)
        orient = fixed_orientations{i, 1};
        desc = fixed_orientations{i, 2};
        
        % Read position from first note
        fid = fopen(note_file, 'r');
        while ~feof(fid)
            line = fgetl(fid);
            if ischar(line) && ~isempty(strtrim(line)) && line(1) ~= '%'
                parts = strsplit(strtrim(line), ',');
                if length(parts) == 7
                    x = str2double(parts{2});
                    y = str2double(parts{3});
                    z = str2double(parts{4});
                    break;
                end
            end
        end
        fclose(fid);
        
        % Create transformation with fixed orientation
        R = EulerToRotation(orient(1), orient(2), orient(3), 'ZYX');
        p = [x; y; z];
        T = [R, p; 0, 0, 0, 1];
        
        % Test IK
        theta_all = Ikine6s(T, config);
        if ~isempty(theta_all)
            theta_valid = FilterJointLimits(theta_all);
            if ~isempty(theta_valid)
                fprintf('  ✓ [%.0f, %.0f, %.0f] %s: Found %d valid solutions\n', ...
                    orient, desc, size(theta_valid, 1));
                solutions_found = true;
                recommended_orientation = orient;
                recommended_desc = desc;
            else
                fprintf('  ✗ [%.0f, %.0f, %.0f] %s: %d solutions, none valid\n', ...
                    orient, desc, size(theta_all, 1));
            end
        else
            fprintf('  ✗ [%.0f, %.0f, %.0f] %s: No IK solutions\n', orient, desc);
        end
    end
    
    if solutions_found
        fprintf('\n✓ SOLUTION FOUND!\n');
        fprintf('Use fixed orientation: [%.0f, %.0f, %.0f] - %s\n\n', ...
            recommended_orientation, recommended_desc);
        fprintf('To apply this fix:\n');
        fprintf('1. Replace LoadNotePositions.m with LoadNotePositions_v2.m\n');
        fprintf('2. In GenerateJointTrajectory.m, change:\n');
        fprintf('   note_data = LoadNotePositions(note_file);\n');
        fprintf('   to:\n');
        fprintf('   note_data = LoadNotePositions(note_file, ''UseFixedOrientation'', [%.0f, %.0f, %.0f]);\n\n', ...
            recommended_orientation);
        return;
    end
    
    %% No solutions found
    fprintf('\n❌ NO SOLUTIONS FOUND\n\n');
    fprintf('The position may be outside the robot workspace.\n');
    fprintf('Position: [%.3f, %.3f, %.3f] mm\n', x, y, z);
    fprintf('Distance from base: %.3f mm\n', sqrt(x^2 + y^2 + z^2));
    fprintf('Maximum reach: ~984 mm (L2+L3+L4)\n\n');
    fprintf('Recommendations:\n');
    fprintf('1. Verify the position is reachable\n');
    fprintf('2. Check if note.txt uses correct units (mm not m)\n');
    fprintf('3. Adjust xylophone position closer to robot\n');
    fprintf('4. Consider different robot base position\n');
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


function R = EulerToRotation(rx, ry, rz, convention)
    % Same as in LoadNotePositions_v2.m
    if nargin < 4
        convention = 'ZYX';
    end
    
    rx_rad = deg2rad(rx);
    ry_rad = deg2rad(ry);
    rz_rad = deg2rad(rz);
    
    Rx = [1, 0, 0; 0, cos(rx_rad), -sin(rx_rad); 0, sin(rx_rad), cos(rx_rad)];
    Ry = [cos(ry_rad), 0, sin(ry_rad); 0, 1, 0; -sin(ry_rad), 0, cos(ry_rad)];
    Rz = [cos(rz_rad), -sin(rz_rad), 0; sin(rz_rad), cos(rz_rad), 0; 0, 0, 1];
    
    switch upper(convention)
        case 'ZYX'
            R = Rz * Ry * Rx;
        case 'XYZ'
            R = Rx * Ry * Rz;
        case 'ZXY'
            R = Rz * Rx * Ry;
        case 'YXZ'
            R = Ry * Rx * Rz;
        case 'XZY'
            R = Rx * Rz * Ry;
        case 'YZX'
            R = Ry * Rz * Rx;
        otherwise
            R = Rz * Ry * Rx;
    end
end
