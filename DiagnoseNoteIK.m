function DiagnoseNoteIK()
    % DiagnoseNoteIK - Diagnose IK issues for note positions
    % This function helps identify why certain note positions fail IK
    
    clc;
    fprintf('=== Diagnosing Note Position IK Solutions ===\n\n');
    
    % Load robot configuration
    config = LoadRobotConfig();
    
    % Load note positions
    note_file = 'note.txt';
    if ~exist(note_file, 'file')
        error('Note file not found: %s', note_file);
    end
    note_data = LoadNotePositions(note_file);
    
    fprintf('Loaded %d notes\n\n', length(note_data));
    
    % Analyze each note
    for i = 1:length(note_data)
        fprintf('--- Note %d: %s ---\n', i, note_data(i).name);
        fprintf('Position: [%.3f, %.3f, %.3f] mm\n', note_data(i).position);
        
        % Extract Euler angles from rotation matrix (approximate)
        R = note_data(i).rotation;
        fprintf('Rotation matrix:\n');
        disp(R);
        
        % Get transformation matrix
        T_target = note_data(i).transform;
        
        % Compute IK
        theta_all = Ikine6s(T_target, config);
        
        if isempty(theta_all)
            fprintf('  ❌ NO IK solutions found!\n');
            fprintf('  Possible reasons:\n');
            fprintf('    - Position may be unreachable\n');
            fprintf('    - Orientation may be incompatible\n');
            fprintf('  Suggestion: Check if position is within workspace\n');
        else
            fprintf('  ✓ Found %d IK solutions\n', size(theta_all, 1));
            
            % Show solutions in degrees
            fprintf('  Solutions (degrees):\n');
            for j = 1:size(theta_all, 1)
                fprintf('    Sol %d: [', j);
                fprintf('%.1f ', rad2deg(theta_all(j, :)));
                fprintf(']\n');
            end
            
            % Filter by joint limits
            theta_valid = FilterJointLimits(theta_all);
            
            if isempty(theta_valid)
                fprintf('  ❌ NO solutions within joint limits!\n');
                fprintf('  Joint limit violations:\n');
                
                joint_limits = [
                    -170 170;
                    -120 120;
                    -140 140;
                    -170 170;
                    -120 120;
                    -360 360
                ];
                
                theta_deg = rad2deg(theta_all);
                for j = 1:size(theta_all, 1)
                    violations = [];
                    for k = 1:6
                        if theta_deg(j, k) < joint_limits(k, 1) || theta_deg(j, k) > joint_limits(k, 2)
                            violations = [violations, k];
                        end
                    end
                    if ~isempty(violations)
                        fprintf('    Sol %d violates joints: [', j);
                        fprintf('%d ', violations);
                        fprintf(']\n');
                        for v = violations
                            fprintf('      Joint %d: %.1f° (limit: [%.0f, %.0f])\n', ...
                                v, theta_deg(j, v), joint_limits(v, 1), joint_limits(v, 2));
                        end
                    end
                end
            else
                fprintf('  ✓ %d solutions within joint limits\n', size(theta_valid, 1));
                fprintf('  Valid solutions (degrees):\n');
                for j = 1:size(theta_valid, 1)
                    fprintf('    Sol %d: [', j);
                    fprintf('%.1f ', rad2deg(theta_valid(j, :)));
                    fprintf(']\n');
                end
            end
        end
        fprintf('\n');
    end
    
    fprintf('=== Diagnosis Complete ===\n');
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
