function note_data = LoadNotePositions(filename, varargin)
    % LoadNotePositions - Read xylophone note positions from file
    %
    % Input:
    %   filename - Path to note.txt file
    %   Optional parameters:
    %     'EulerConvention' - Euler angle convention (default: 'ZYX')
    %                         Options: 'ZYX', 'XYZ', 'ZYZ', 'XYX'
    %     'UseFixedOrientation' - Use fixed orientation instead of file values
    %                             Value: [rx, ry, rz] in degrees (default: use file)
    %
    % Output:
    %   note_data - Structure array with fields:
    %       .name - Note name (Do, Re, Mi, etc.)
    %       .position - [x; y; z] in mm
    %       .rotation - 3x3 rotation matrix
    %       .transform - 4x4 transformation matrix
    %
    % Examples:
    %   note_data = LoadNotePositions('note.txt');  % Default ZYX
    %   note_data = LoadNotePositions('note.txt', 'EulerConvention', 'XYZ');
    %   note_data = LoadNotePositions('note.txt', 'UseFixedOrientation', [0, 180, 0]);
    
    % Parse optional arguments
    p = inputParser;
    addParameter(p, 'EulerConvention', 'ZYX', @ischar);
    addParameter(p, 'UseFixedOrientation', [], @(x) isempty(x) || (isnumeric(x) && length(x)==3));
    parse(p, varargin{:});
    
    euler_convention = p.Results.EulerConvention;
    fixed_orientation = p.Results.UseFixedOrientation;
    
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
            
            % Use fixed orientation if specified, otherwise read from file
            if ~isempty(fixed_orientation)
                rx = fixed_orientation(1);
                ry = fixed_orientation(2);
                rz = fixed_orientation(3);
            else
                rx = str2double(parts{5});  % degrees
                ry = str2double(parts{6});  % degrees
                rz = str2double(parts{7});  % degrees
            end
            
            % Convert Euler angles to rotation matrix
            R = EulerToRotation(rx, ry, rz, euler_convention);
            
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
    if ~isempty(fixed_orientation)
        fprintf('Using fixed orientation: [%.1f, %.1f, %.1f] degrees\n', fixed_orientation);
    end
    fprintf('Euler convention: %s\n', euler_convention);
end


function R = EulerToRotation(rx, ry, rz, convention)
    % EulerToRotation - Convert Euler angles to rotation matrix
    %
    % Input:
    %   rx, ry, rz - Euler angles in degrees
    %   convention - Euler angle convention (default: 'ZYX')
    %
    % Output:
    %   R - 3x3 rotation matrix
    
    if nargin < 4
        convention = 'ZYX';
    end
    
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
    
    % Combined rotation based on convention
    switch upper(convention)
        case 'ZYX'
            % Roll-Pitch-Yaw (most common in robotics)
            R = Rz * Ry * Rx;
            
        case 'XYZ'
            % Alternative convention
            R = Rx * Ry * Rz;
            
        case 'ZYZ'
            % Euler angles (proper)
            % Note: In this case, rx=alpha, ry=beta, rz=gamma
            R = Rz * Ry * Rz;
            
        case 'XYX'
            % Euler angles (proper)
            R = Rx * Ry * Rx;
            
        case 'YXZ'
            R = Ry * Rx * Rz;
            
        case 'YZX'
            R = Ry * Rz * Rx;
            
        case 'XZY'
            R = Rx * Rz * Ry;
            
        case 'ZXY'
            R = Rz * Rx * Ry;
            
        otherwise
            error('Unknown Euler convention: %s. Use ZYX, XYZ, ZYZ, or XYX', convention);
    end
end
