% =========================================================================
% Test script for MotionPlanning function
%
% Tests trapezoidal velocity planning with:
% 1. Single axis motion (trapezoidal profile)
% 2. Single axis motion (triangular profile)
% 3. Multi-axis synchronized motion
% =========================================================================

clear;
clc;
close all;

fprintf('=== MotionPlanning Test Suite ===\n\n');

%% Test 1: Single axis - Trapezoidal profile
fprintf('Test 1: Single axis trapezoidal profile\n');
fprintf('----------------------------------------\n');

TotalLength = 1000;  % 1000 mm displacement
Vmax = 500;          % 500 mm/s max velocity
Accel = 2000;        % 2000 mm/s^2 acceleration
Decel = 2000;        % 2000 mm/s^2 deceleration
Ts = 0.001;          % 1 ms sampling time

MotionData1 = MotionPlanning(TotalLength, Vmax, Accel, Decel, Ts);

fprintf('Total time: %.4f s\n', MotionData1.maxtime);
fprintf('Number of samples: %d\n', length(MotionData1.t));
fprintf('Final position: %.4f mm\n', MotionData1.s(end));
fprintf('Max velocity achieved: %.4f mm/s\n', max(abs(MotionData1.v)));

% Verify final position
assert(abs(MotionData1.s(end) - TotalLength) < 0.01, 'Test 1 failed: Final position incorrect');
fprintf('✓ Test 1 passed\n\n');

%% Test 2: Single axis - Triangular profile
fprintf('Test 2: Single axis triangular profile\n');
fprintf('----------------------------------------\n');

TotalLength2 = 100;   % Short distance to force triangular profile
Vmax2 = 500;          % High max velocity (won''t be reached)
Accel2 = 2000;
Decel2 = 2000;
Ts2 = 0.001;

MotionData2 = MotionPlanning(TotalLength2, Vmax2, Accel2, Decel2, Ts2);

fprintf('Total time: %.4f s\n', MotionData2.maxtime);
fprintf('Final position: %.4f mm\n', MotionData2.s(end));
fprintf('Max velocity achieved: %.4f mm/s (should be < %.4f mm/s)\n', ...
    max(abs(MotionData2.v)), Vmax2);

% Verify final position
assert(abs(MotionData2.s(end) - TotalLength2) < 0.01, 'Test 2 failed: Final position incorrect');
% Verify it's a triangular profile (max velocity not reached)
assert(max(abs(MotionData2.v)) < Vmax2, 'Test 2 failed: Should be triangular profile');
fprintf('✓ Test 2 passed\n\n');

%% Test 3: Multi-axis synchronized motion
fprintf('Test 3: Multi-axis synchronized motion\n');
fprintf('----------------------------------------\n');

% Different displacements for 3 axes
TotalLength3 = [1000, 500, 200];
Vmax3 = [500, 400, 300];
Accel3 = [2000, 2000, 2000];
Decel3 = [2000, 2000, 2000];
Ts3 = 0.001;

MotionData3 = MotionPlanning(TotalLength3, Vmax3, Accel3, Decel3, Ts3);

fprintf('Total time: %.4f s\n', MotionData3.maxtime);
fprintf('Final positions:\n');
for i = 1:3
    fprintf('  Axis %d: %.4f mm (expected: %.4f mm)\n', ...
        i, MotionData3.s(i, end), TotalLength3(i));
end

% Verify all axes reach their target positions
for i = 1:3
    assert(abs(MotionData3.s(i, end) - TotalLength3(i)) < 0.01, ...
        sprintf('Test 3 failed: Axis %d final position incorrect', i));
end

% Verify all axes have zero velocity at the end
for i = 1:3
    assert(abs(MotionData3.v(i, end)) < 0.01, ...
        sprintf('Test 3 failed: Axis %d final velocity not zero', i));
end

fprintf('✓ Test 3 passed\n\n');

%% Test 4: Negative displacement
fprintf('Test 4: Negative displacement\n');
fprintf('----------------------------------------\n');

TotalLength4 = -1000;  % Negative displacement
Vmax4 = 500;
Accel4 = 2000;
Decel4 = 2000;
Ts4 = 0.001;

MotionData4 = MotionPlanning(TotalLength4, Vmax4, Accel4, Decel4, Ts4);

fprintf('Final position: %.4f mm (expected: %.4f mm)\n', ...
    MotionData4.s(end), TotalLength4);

% Verify final position
assert(abs(MotionData4.s(end) - TotalLength4) < 0.01, ...
    'Test 4 failed: Final position incorrect for negative displacement');
fprintf('✓ Test 4 passed\n\n');

%% Test 5: Zero displacement
fprintf('Test 5: Zero displacement on one axis\n');
fprintf('----------------------------------------\n');

TotalLength5 = [1000, 0, 500];
Vmax5 = [500, 400, 300];
Accel5 = [2000, 2000, 2000];
Decel5 = [2000, 2000, 2000];
Ts5 = 0.001;

MotionData5 = MotionPlanning(TotalLength5, Vmax5, Accel5, Decel5, Ts5);

fprintf('Final positions:\n');
for i = 1:3
    fprintf('  Axis %d: %.4f mm (expected: %.4f mm)\n', ...
        i, MotionData5.s(i, end), TotalLength5(i));
end

% Verify all axes reach their target positions
for i = 1:3
    assert(abs(MotionData5.s(i, end) - TotalLength5(i)) < 0.01, ...
        sprintf('Test 5 failed: Axis %d final position incorrect', i));
end

fprintf('✓ Test 5 passed\n\n');

%% Visualization Test (optional, commented out for automated testing)
% Uncomment to see plots
%{
figure('Position', [100, 100, 1200, 800]);

% Plot Test 3 results (multi-axis)
subplot(3, 1, 1);
plot(MotionData3.t, MotionData3.s');
xlabel('Time (s)');
ylabel('Position (mm)');
title('Position Profiles - Multi-axis Synchronized');
legend('Axis 1', 'Axis 2', 'Axis 3');
grid on;

subplot(3, 1, 2);
plot(MotionData3.t, MotionData3.v');
xlabel('Time (s)');
ylabel('Velocity (mm/s)');
title('Velocity Profiles - Multi-axis Synchronized');
legend('Axis 1', 'Axis 2', 'Axis 3');
grid on;

subplot(3, 1, 3);
plot(MotionData3.t, MotionData3.a');
xlabel('Time (s)');
ylabel('Acceleration (mm/s^2)');
title('Acceleration Profiles - Multi-axis Synchronized');
legend('Axis 1', 'Axis 2', 'Axis 3');
grid on;
%}

fprintf('=== All tests passed! ===\n');
