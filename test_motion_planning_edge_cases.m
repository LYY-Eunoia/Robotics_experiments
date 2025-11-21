% Comprehensive edge case test for MotionPlanning
clear;
clc;

fprintf('=== MotionPlanning Edge Case Tests ===\n\n');

%% Test 1: Different acceleration and deceleration values
fprintf('Test 1: Different acceleration and deceleration\n');
fprintf('------------------------------------------------\n');

TotalLength1 = 1000;
Vmax1 = 500;
Accel1 = 2000;  % Fast acceleration
Decel1 = 1000;  % Slow deceleration
Ts1 = 0.001;

MotionData1 = MotionPlanning(TotalLength1, Vmax1, Accel1, Decel1, Ts1);

fprintf('Acceleration time: %.4f s\n', Vmax1/Accel1);
fprintf('Deceleration time: %.4f s\n', Vmax1/Decel1);
fprintf('Total time: %.4f s\n', MotionData1.maxtime);
fprintf('Final position: %.4f mm (expected: %.4f mm)\n', MotionData1.s(end), TotalLength1);

assert(abs(MotionData1.s(end) - TotalLength1) < 0.01, 'Test 1 failed');
fprintf('✓ Test 1 passed\n\n');

%% Test 2: Very small displacement (triangular profile)
fprintf('Test 2: Very small displacement\n');
fprintf('--------------------------------\n');

TotalLength2 = 10;  % Very small
Vmax2 = 1000;
Accel2 = 5000;
Decel2 = 5000;
Ts2 = 0.001;

MotionData2 = MotionPlanning(TotalLength2, Vmax2, Accel2, Decel2, Ts2);

fprintf('Total time: %.4f s\n', MotionData2.maxtime);
fprintf('Max velocity: %.4f mm/s (much less than %.4f mm/s)\n', ...
    max(abs(MotionData2.v)), Vmax2);
fprintf('Final position: %.4f mm (expected: %.4f mm)\n', MotionData2.s(end), TotalLength2);

assert(abs(MotionData2.s(end) - TotalLength2) < 0.01, 'Test 2 failed');
assert(max(abs(MotionData2.v)) < Vmax2, 'Test 2 failed: should be triangular');
fprintf('✓ Test 2 passed\n\n');

%% Test 3: Multiple axes with vastly different parameters
fprintf('Test 3: Vastly different axis parameters\n');
fprintf('-----------------------------------------\n');

TotalLength3 = [10000, 100, 1];  % Very different distances
Vmax3 = [1000, 50, 10];          % Different max velocities
Accel3 = [5000, 1000, 100];      % Different accelerations
Decel3 = [5000, 1000, 100];
Ts3 = 0.001;

MotionData3 = MotionPlanning(TotalLength3, Vmax3, Accel3, Decel3, Ts3);

fprintf('Total time: %.4f s\n', MotionData3.maxtime);
for i = 1:3
    fprintf('Axis %d: final position = %.4f (expected: %.4f)\n', ...
        i, MotionData3.s(i, end), TotalLength3(i));
    assert(abs(MotionData3.s(i, end) - TotalLength3(i)) < 0.01, ...
        sprintf('Test 3 failed for axis %d', i));
end
fprintf('✓ Test 3 passed\n\n');

%% Test 4: Mixed positive and negative displacements
fprintf('Test 4: Mixed positive and negative displacements\n');
fprintf('--------------------------------------------------\n');

TotalLength4 = [1000, -500, 250, -750];
Vmax4 = [500, 400, 300, 600];
Accel4 = [2000, 2000, 2000, 2000];
Decel4 = [2000, 2000, 2000, 2000];
Ts4 = 0.001;

MotionData4 = MotionPlanning(TotalLength4, Vmax4, Accel4, Decel4, Ts4);

fprintf('Total time: %.4f s\n', MotionData4.maxtime);
for i = 1:4
    fprintf('Axis %d: final position = %.4f (expected: %.4f)\n', ...
        i, MotionData4.s(i, end), TotalLength4(i));
    assert(abs(MotionData4.s(i, end) - TotalLength4(i)) < 0.01, ...
        sprintf('Test 4 failed for axis %d', i));
    % Also verify final velocity is zero
    assert(abs(MotionData4.v(i, end)) < 0.01, ...
        sprintf('Test 4 failed: axis %d final velocity not zero', i));
end
fprintf('✓ Test 4 passed\n\n');

%% Test 5: Single axis with very low sampling time
fprintf('Test 5: High resolution sampling\n');
fprintf('---------------------------------\n');

TotalLength5 = 1000;
Vmax5 = 500;
Accel5 = 2000;
Decel5 = 2000;
Ts5 = 0.0001;  % Very small sampling time

MotionData5 = MotionPlanning(TotalLength5, Vmax5, Accel5, Decel5, Ts5);

fprintf('Sampling time: %.6f s\n', Ts5);
fprintf('Number of samples: %d\n', length(MotionData5.t));
fprintf('Final position: %.6f mm (expected: %.6f mm)\n', MotionData5.s(end), TotalLength5);

assert(abs(MotionData5.s(end) - TotalLength5) < 0.001, 'Test 5 failed');
fprintf('✓ Test 5 passed\n\n');

%% Test 6: Multiple axes all with zero displacement
fprintf('Test 6: All axes with zero displacement\n');
fprintf('----------------------------------------\n');

TotalLength6 = [0, 0, 0];
Vmax6 = [500, 400, 300];
Accel6 = [2000, 2000, 2000];
Decel6 = [2000, 2000, 2000];
Ts6 = 0.001;

MotionData6 = MotionPlanning(TotalLength6, Vmax6, Accel6, Decel6, Ts6);

fprintf('Total time: %.4f s\n', MotionData6.maxtime);
for i = 1:3
    fprintf('Axis %d: final position = %.4f (expected: 0.0000)\n', ...
        i, MotionData6.s(i, end));
    assert(abs(MotionData6.s(i, end)) < 0.01, ...
        sprintf('Test 6 failed for axis %d', i));
    % All positions should be zero throughout
    assert(all(abs(MotionData6.s(i, :)) < 0.01), ...
        sprintf('Test 6 failed: axis %d moved when it should not', i));
end
fprintf('✓ Test 6 passed\n\n');

%% Test 7: Verify structure fields
fprintf('Test 7: Verify MotionData structure\n');
fprintf('------------------------------------\n');

TotalLength7 = [100, 200];
Vmax7 = [100, 200];
Accel7 = [500, 500];
Decel7 = [500, 500];
Ts7 = 0.01;

MotionData7 = MotionPlanning(TotalLength7, Vmax7, Accel7, Decel7, Ts7);

% Check that all required fields exist
assert(isfield(MotionData7, 's'), 'Missing field: s');
assert(isfield(MotionData7, 'v'), 'Missing field: v');
assert(isfield(MotionData7, 'a'), 'Missing field: a');
assert(isfield(MotionData7, 't'), 'Missing field: t');
assert(isfield(MotionData7, 'maxtime'), 'Missing field: maxtime');

% Check dimensions
N = length(TotalLength7);
M = length(MotionData7.t);
assert(size(MotionData7.s, 1) == N && size(MotionData7.s, 2) == M, ...
    'Position matrix has wrong dimensions');
assert(size(MotionData7.v, 1) == N && size(MotionData7.v, 2) == M, ...
    'Velocity matrix has wrong dimensions');
assert(size(MotionData7.a, 1) == N && size(MotionData7.a, 2) == M, ...
    'Acceleration matrix has wrong dimensions');
assert(length(MotionData7.t) == M && size(MotionData7.t, 1) == M, ...
    'Time vector has wrong dimensions');

fprintf('All structure fields present and correctly sized\n');
fprintf('✓ Test 7 passed\n\n');

fprintf('=== All edge case tests passed! ===\n');
