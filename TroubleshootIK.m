% Troubleshooting Guide for IK Issues with Real Robot Positions
% 
% This script helps diagnose and fix "No valid IK solution within joint limits" errors
% when using actual robot measurements in note.txt
%
% Common Issues and Solutions:
%
% 1. EULER ANGLE CONVENTION MISMATCH
%    Problem: Different robots use different Euler angle conventions
%    Solution: Try different conventions in LoadNotePositions.m
%    
%    Common conventions:
%    - ZYX (current): R = Rz(rz) * Ry(ry) * Rx(rx)  
%    - XYZ: R = Rx(rx) * Ry(ry) * Rz(rz)
%    - ZYZ: R = Rz(rz1) * Ry(ry) * Rz(rz2)
%
% 2. POSITION OUT OF WORKSPACE
%    Problem: Position is too far or at singular configuration
%    Solution: Verify positions are reachable
%    
%    Robot reach: ~984mm max (L2+L3+L4 = 450+450+84)
%    Your positions: Check distance from base
%
% 3. ORIENTATION INCOMPATIBLE
%    Problem: Orientation requires joint angles outside limits
%    Solution: Adjust orientation or relax constraints
%
% 4. USING MEASURED POSE DIRECTLY
%    Problem: Measured pose is from FK, not suitable for target
%    Solution: Extract only position, use fixed orientation
%
% RECOMMENDED WORKFLOW:
%
% Step 1: Run DiagnoseNoteIK() to see which notes fail
%         >> DiagnoseNoteIK
%
% Step 2: If all notes fail with same pattern, likely Euler convention issue
%         Try: UpdateEulerConvention('XYZ') or UpdateEulerConvention('ZYZ')
%
% Step 3: If specific notes fail, check workspace limits
%         Distance from base = sqrt(x^2 + y^2 + z^2)
%         Should be < 984mm
%
% Step 4: If orientation seems wrong, try using position-only with fixed orientation
%         See: ConvertToPositionOnlyNotes()
%
% Step 5: If joint limits are violated, you may need to:
%         - Adjust xylophone position
%         - Modify robot base position
%         - Change end-effector orientation

%% Quick Fix: Use Position Only with Fixed Orientation
function ConvertToPositionOnlyNotes()
    fprintf('Converting note.txt to use positions only with fixed orientation...\n');
    
    % Read current positions
    fid = fopen('note.txt', 'r');
    lines = {};
    while ~feof(fid)
        lines{end+1} = fgetl(fid);
    end
    fclose(fid);
    
    % Create backup
    copyfile('note.txt', 'note_backup.txt');
    fprintf('Backup saved to note_backup.txt\n');
    
    % Extract positions and use fixed orientation
    % Fixed orientation: pointing downward (0, 180, 0) or (180, 0, 180)
    fprintf('\nChoose fixed orientation:\n');
    fprintf('1. (0, 180, 0) - End-effector pointing straight down\n');
    fprintf('2. (180, 0, 180) - End-effector pointing down, rotated\n');
    fprintf('3. Keep original orientations\n');
    choice = input('Enter choice (1-3): ');
    
    if choice == 1
        fixed_angles = [0, 180, 0];
    elseif choice == 2
        fixed_angles = [180, 0, 180];
    else
        fprintf('Keeping original orientations.\n');
        return;
    end
    
    % Write new file
    fid_out = fopen('note.txt', 'w');
    
    for i = 1:length(lines)
        line = lines{i};
        if ischar(line) && ~isempty(strtrim(line)) && line(1) ~= '%'
            % Parse data line
            parts = strsplit(strtrim(line), ',');
            if length(parts) == 7
                % Write with new orientation
                fprintf(fid_out, '%s, %s, %s, %s, %.3f, %.3f, %.3f\n', ...
                    parts{1}, parts{2}, parts{3}, parts{4}, ...
                    fixed_angles(1), fixed_angles(2), fixed_angles(3));
            else
                fprintf(fid_out, '%s\n', line);
            end
        else
            fprintf(fid_out, '%s\n', line);
        end
    end
    
    fclose(fid_out);
    fprintf('Updated note.txt with fixed orientation [%.1f, %.1f, %.1f]\n', fixed_angles);
    fprintf('Run DiagnoseNoteIK to verify solutions exist.\n');
end

%% Change Euler Convention in LoadNotePositions
function UpdateEulerConvention(convention)
    fprintf('Updating Euler angle convention to: %s\n', convention);
    fprintf('This requires manual edit of LoadNotePositions.m\n');
    fprintf('In the EulerToRotation function, change the line:\n');
    fprintf('    R = Rz * Ry * Rx;  (ZYX convention)\n');
    fprintf('to:\n');
    
    switch upper(convention)
        case 'XYZ'
            fprintf('    R = Rz * Ry * Rx;  (XYZ convention)\n');
        case 'ZYZ'
            fprintf('    % For ZYZ, use different rotation\n');
        otherwise
            fprintf('Unknown convention: %s\n', convention);
    end
end
