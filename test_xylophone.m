% Test Script for Xylophone Playing Robot
% This script performs basic validation of the new functions

%% Test 1: Load Note Positions
fprintf('=== Test 1: Loading Note Positions ===\n');
try
    note_data = LoadNotePositions('note.txt');
    fprintf('SUCCESS: Loaded %d notes\n', length(note_data));
    fprintf('Note names: ');
    for i = 1:length(note_data)
        fprintf('%s ', note_data(i).name);
    end
    fprintf('\n');
    fprintf('First note position: [%.1f, %.1f, %.1f]\n', note_data(1).position);
catch ME
    fprintf('FAILED: %s\n', ME.message);
end

%% Test 2: Load Musical Score
fprintf('\n=== Test 2: Loading Musical Score ===\n');
try
    [note_sequence, time_intervals] = GetTwinkleTwinkleScore();
    fprintf('SUCCESS: Loaded %d notes in sequence\n', length(note_sequence));
    fprintf('Total duration: %.2f seconds\n', sum(time_intervals));
    fprintf('First 10 notes: ');
    for i = 1:min(10, length(note_sequence))
        fprintf('%s ', note_sequence{i});
    end
    fprintf('\n');
catch ME
    fprintf('FAILED: %s\n', ME.message);
end

%% Test 3: Plan Strike Action
fprintf('\n=== Test 3: Planning Strike Action ===\n');
try
    % Test joint configuration
    q_test = [0, 0, 0, 0, 0, 0];
    config.L1 = 491;
    config.L2 = 450;
    config.L3 = 450;
    config.L4 = 84;
    
    [t_strike, q_strike, qd_strike, qdd_strike] = PlanStrikeAction(q_test, config);
    fprintf('SUCCESS: Generated strike trajectory\n');
    fprintf('Duration: %.3f seconds\n', t_strike(end));
    fprintf('Number of points: %d\n', length(t_strike));
    fprintf('Max joint 5 angle change: %.2f degrees\n', rad2deg(max(q_strike(:,5)) - min(q_strike(:,5))));
catch ME
    fprintf('FAILED: %s\n', ME.message);
end

%% Test 4: Plan Joint Space Motion
fprintf('\n=== Test 4: Planning Joint Space Motion ===\n');
try
    q_start = [0, 0, 0, 0, 0, 0];
    q_end = [0.1, 0.2, -0.1, 0, 0, 0];
    duration = 1.0;
    
    [t_seg, q_seg, qd_seg, qdd_seg] = PlanJointSpaceMotion(q_start, q_end, duration);
    fprintf('SUCCESS: Generated joint space trajectory\n');
    fprintf('Duration: %.3f seconds\n', t_seg(end));
    fprintf('Number of points: %d\n', length(t_seg));
    fprintf('Start position: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', q_seg(1,:));
    fprintf('End position: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', q_seg(end,:));
    
    % Verify boundary conditions
    if max(abs(qd_seg(1,:))) < 1e-6 && max(abs(qd_seg(end,:))) < 1e-6
        fprintf('Velocity boundary conditions: PASSED\n');
    else
        fprintf('WARNING: Velocity not zero at boundaries\n');
    end
    
    if max(abs(qdd_seg(1,:))) < 1e-6 && max(abs(qdd_seg(end,:))) < 1e-6
        fprintf('Acceleration boundary conditions: PASSED\n');
    else
        fprintf('WARNING: Acceleration not zero at boundaries\n');
    end
catch ME
    fprintf('FAILED: %s\n', ME.message);
end

%% Test 5: Verify note.txt format
fprintf('\n=== Test 5: Verifying note.txt Format ===\n');
try
    fid = fopen('note.txt', 'r');
    if fid == -1
        error('Cannot open note.txt');
    end
    
    line_count = 0;
    data_count = 0;
    while ~feof(fid)
        line = fgetl(fid);
        if ischar(line)
            line_count = line_count + 1;
            % Check if it's a data line (not comment or empty)
            if ~isempty(strtrim(line)) && line(1) ~= '%'
                data_count = data_count + 1;
            end
        end
    end
    fclose(fid);
    
    fprintf('SUCCESS: note.txt has %d lines, %d data lines\n', line_count, data_count);
    if data_count == 7
        fprintf('Correct number of notes (7): PASSED\n');
    else
        fprintf('WARNING: Expected 7 notes, found %d\n', data_count);
    end
catch ME
    fprintf('FAILED: %s\n', ME.message);
    if fid ~= -1
        fclose(fid);
    end
end

%% Test Summary
fprintf('\n=== Test Summary ===\n');
fprintf('Basic function tests completed.\n');
fprintf('Note: Full integration test requires running GenerateJointTrajectory()\n');
fprintf('      and selecting an initial IK solution interactively.\n');
