% Integration test demonstrating practical usage of MotionPlanning
% This simulates a 6-DOF robot arm motion planning scenario
clear;
clc;

fprintf('=== MotionPlanning Integration Test ===\n');
fprintf('Simulating 6-DOF robot arm trajectory planning\n\n');

% Define a realistic 6-axis robot motion
% Joint displacements in radians (converted to match mm units for consistency)
theta_start = [0, 0, 0, 0, 0, 0];  % Starting joint angles (rad)
theta_end = [0.5, -0.3, 0.8, 0.2, -0.4, 0.6];  % Target joint angles (rad)

% Total displacement for each joint
TotalLength = theta_end - theta_start;

% Maximum velocities for each joint (rad/s)
Vmax = [1.0, 1.0, 1.5, 2.0, 2.0, 2.5];

% Acceleration for each joint (rad/s^2)
Accel = [5.0, 5.0, 7.5, 10.0, 10.0, 12.5];

% Deceleration for each joint (rad/s^2)
Decel = [5.0, 5.0, 7.5, 10.0, 10.0, 12.5];

% Sampling time (1 kHz control loop)
Ts = 0.001;

% Generate motion plan
fprintf('Planning trajectory...\n');
MotionData = MotionPlanning(TotalLength, Vmax, Accel, Decel, Ts);

fprintf('\n--- Planning Results ---\n');
fprintf('Total motion time: %.4f seconds\n', MotionData.maxtime);
fprintf('Number of control points: %d\n', length(MotionData.t));
fprintf('Control frequency: %.0f Hz\n\n', 1/Ts);

fprintf('--- Joint Analysis ---\n');
for i = 1:6
    fprintf('Joint %d:\n', i);
    fprintf('  Displacement: %.4f rad (%.2f deg)\n', ...
        TotalLength(i), rad2deg(TotalLength(i)));
    fprintf('  Final position: %.4f rad\n', theta_start(i) + MotionData.s(i, end));
    fprintf('  Expected position: %.4f rad\n', theta_end(i));
    fprintf('  Position error: %.6f rad\n', abs(theta_end(i) - (theta_start(i) + MotionData.s(i, end))));
    fprintf('  Peak velocity: %.4f rad/s (limit: %.4f rad/s)\n', ...
        max(abs(MotionData.v(i, :))), Vmax(i));
    fprintf('  Peak acceleration: %.4f rad/s^2 (limit: %.4f rad/s^2)\n', ...
        max(abs(MotionData.a(i, :))), Accel(i));
    fprintf('\n');
end

% Verify all joints reach target positions
fprintf('--- Verification ---\n');
all_pass = true;
for i = 1:6
    final_pos = theta_start(i) + MotionData.s(i, end);
    error = abs(final_pos - theta_end(i));
    if error > 1e-6
        fprintf('✗ Joint %d: Position error too large (%.6f rad)\n', i, error);
        all_pass = false;
    else
        fprintf('✓ Joint %d: Reached target position\n', i);
    end
end

% Verify all joints stop at the end
all_stopped = true;
for i = 1:6
    if abs(MotionData.v(i, end)) > 1e-6
        fprintf('✗ Joint %d: Not stopped (velocity = %.6f rad/s)\n', i, MotionData.v(i, end));
        all_stopped = false;
    end
end

if all_stopped
    fprintf('✓ All joints stopped at final position\n');
end

if all_pass && all_stopped
    fprintf('\n=== Integration test PASSED ===\n');
else
    fprintf('\n=== Integration test FAILED ===\n');
    error('Integration test failed');
end

% Demonstrate trajectory interpolation at specific time points
fprintf('\n--- Sample Trajectory Points ---\n');
sample_times = [0, MotionData.maxtime/4, MotionData.maxtime/2, 3*MotionData.maxtime/4, MotionData.maxtime];
fprintf('Time (s) | J1 pos  | J2 pos  | J3 pos  | J4 pos  | J5 pos  | J6 pos\n');
fprintf('---------|---------|---------|---------|---------|---------|--------\n');
for t_sample = sample_times
    idx = find(MotionData.t >= t_sample, 1, 'first');
    if isempty(idx)
        idx = length(MotionData.t);
    end
    fprintf('%8.4f |', t_sample);
    for i = 1:6
        fprintf(' %7.4f |', theta_start(i) + MotionData.s(i, idx));
    end
    fprintf('\n');
end

fprintf('\n=== Integration test complete ===\n');
