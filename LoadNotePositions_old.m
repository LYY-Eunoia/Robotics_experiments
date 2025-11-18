function note_data = LoadNotePositions(filename)
    % LoadNotePositions - Read xylophone note positions from file
    %
    % Input:
    %   filename - Path to note.txt file
    %
    % Output:
    %   note_data - Structure array with fields:
    %       .name - Note name (Do, Re, Mi, etc.)
    %       .position - [x; y; z] in mm
    %       .rotation - 3x3 rotation matrix
    %       .transform - 4x4 transformation matrix
    
    % Open file
    fid = fopen(filename, 'r');
    if fid == -1
        error('Cannot open file: %s', filename);
    end
    
    % Skip header lines (lines starting with %)
    line = fgetl(fid);
    while ischar(line) && ~isempty(line) && line(1) == '%'
        line = fgetl(fid);
    end
    
    % Initialize storage
    note_data = struct('name', {}, 'position', {}, 'rotation', {}, 'transform', {});
    idx = 1;
    
    % Read data lines
    while ischar(line)
        % Skip empty lines and comments
        if isempty(strtrim(line)) || line(1) == '%'
            line = fgetl(fid);
            continue;
        end
        
        % Parse line: Note, x, y, z, rx, ry, rz
        parts = strsplit(strtrim(line), ',');
        if length(parts) == 7
            note_name = strtrim(parts{1});
            x = str2double(parts{2});
            y = str2double(parts{3});
            z = str2double(parts{4});
            rx = str2double(parts{5});  % degrees
            ry = str2double(parts{6});  % degrees
            rz = str2double(parts{7});  % degrees
            
            % Convert Euler angles to rotation matrix
            R = EulerToRotation(rx, ry, rz);
            
            % Create transformation matrix
            p = [x; y; z];
            T = [R, p; 0, 0, 0, 1];
            
            % Store data
            note_data(idx).name = note_name;
            note_data(idx).position = p;
            note_data(idx).rotation = R;
            note_data(idx).transform = T;
            idx = idx + 1;
        end
        
        line = fgetl(fid);
    end
    
    fclose(fid);
    
    if isempty(note_data)
        error('No valid note data found in file: %s', filename);
    end
    
    fprintf('Loaded %d note positions from %s\n', length(note_data), filename);
end


function R = EulerToRotation(rx, ry, rz)
    % EulerToRotation - Convert Euler angles (ZYX convention) to rotation matrix
    %
    % Input:
    %   rx, ry, rz - Euler angles in degrees (rotation about X, Y, Z axes)
    %
    % Output:
    %   R - 3x3 rotation matrix
    %
    % Convention: R = Rz(rz) * Ry(ry) * Rx(rx)
    
    % Convert to radians
    rx_rad = deg2rad(rx);
    ry_rad = deg2rad(ry);
    rz_rad = deg2rad(rz);
    
    % Rotation matrix about X-axis
    Rx = [1,           0,            0;
          0,  cos(rx_rad), -sin(rx_rad);
          0,  sin(rx_rad),  cos(rx_rad)];
    
    % Rotation matrix about Y-axis
    Ry = [ cos(ry_rad), 0, sin(ry_rad);
                     0, 1,           0;
          -sin(ry_rad), 0, cos(ry_rad)];
    
    % Rotation matrix about Z-axis
    Rz = [cos(rz_rad), -sin(rz_rad), 0;
          sin(rz_rad),  cos(rz_rad), 0;
                    0,            0, 1];
    
    % Combined rotation (ZYX convention)
    R = Rz * Ry * Rx;
end
