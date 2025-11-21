% Simple visual test to verify trapezoidal profile characteristics
clear;
clc;

% Test case: 3-axis synchronized motion
TotalLength = [1000, 500, 200];
Vmax = [500, 400, 300];
Accel = [2000, 2000, 2000];
Decel = [2000, 2000, 2000];
Ts = 0.001;

MotionData = MotionPlanning(TotalLength, Vmax, Accel, Decel, Ts);

% Display key results
fprintf('=== MotionPlanning Visual Verification ===\n\n');
fprintf('Total synchronized time: %.4f s\n', MotionData.maxtime);
fprintf('Number of time samples: %d\n', length(MotionData.t));
fprintf('\n');

% Check each axis
for i = 1:3
    fprintf('Axis %d:\n', i);
    fprintf('  Target displacement: %.2f mm\n', TotalLength(i));
    fprintf('  Final position: %.4f mm\n', MotionData.s(i, end));
    fprintf('  Max velocity: %.2f mm/s (limit: %.2f mm/s)\n', ...
        max(abs(MotionData.v(i, :))), Vmax(i));
    fprintf('  Max acceleration: %.2f mm/s^2 (limit: %.2f mm/s^2)\n', ...
        max(abs(MotionData.a(i, :))), Accel(i));
    fprintf('  Position error: %.6f mm\n', abs(MotionData.s(i, end) - TotalLength(i)));
    fprintf('  Final velocity: %.6f mm/s\n', MotionData.v(i, end));
    fprintf('\n');
end

% Verify continuity and smoothness
fprintf('=== Continuity Checks ===\n');
for i = 1:3
    % Check velocity continuity (no jumps)
    v_diff = diff(MotionData.v(i, :));
    max_v_jump = max(abs(v_diff));
    expected_max_jump = max(Accel(i), Decel(i)) * Ts * 1.1; % Allow 10% margin
    fprintf('Axis %d velocity continuity: max jump = %.4f mm/s (should be <= %.4f mm/s)\n', ...
        i, max_v_jump, expected_max_jump);
end
fprintf('\n');

% Verify that axes finish at the same time (within tolerance)
fprintf('=== Synchronization Check ===\n');
for i = 1:3
    % Find when axis stops moving (velocity becomes zero)
    moving_idx = find(abs(MotionData.v(i, :)) > 1e-6, 1, 'last');
    if isempty(moving_idx)
        stop_time = 0;
    else
        stop_time = MotionData.t(moving_idx);
    end
    fprintf('Axis %d stops at t = %.4f s\n', i, stop_time);
end
fprintf('Maximum time (synchronized): %.4f s\n', MotionData.maxtime);
fprintf('\n');

fprintf('=== Verification Complete ===\n');
