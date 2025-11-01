# MotionPlanning Function Implementation Summary

## Overview
Implemented the `MotionPlanning` function in MATLAB for trapezoidal velocity planning in robotics applications.

## Function Signature
```matlab
function MotionData = MotionPlanning(TotalLength, Vmax, Accel, Decel, Ts)
```

## Input Parameters
- **TotalLength**: 1xN array of total displacements for each axis (units: m or rad)
- **Vmax**: 1xN array of maximum velocities (units: m/s or rad/s)
- **Accel**: 1xN array of accelerations (units: m/s² or rad/s²)
- **Decel**: 1xN array of decelerations (units: m/s² or rad/s²)
- **Ts**: Sampling time in seconds (e.g., 0.001 for 1 kHz)

## Output Structure (MotionData)
- **s**: NxM position profiles matrix (units: m or rad)
- **v**: NxM velocity profiles matrix (units: m/s or rad/s)
- **a**: NxM acceleration profiles matrix (units: m/s² or rad/s²)
- **t**: Mx1 time vector (units: s)
- **maxtime**: Total synchronized time (units: s)

Where N = number of axes, M = number of time samples

## Key Features

### 1. Trapezoidal and Triangular Profiles
- Automatically selects between trapezoidal and triangular velocity profiles
- **Trapezoidal**: Used when sufficient distance to reach maximum velocity
- **Triangular**: Used when distance is too short to reach maximum velocity
- Peak velocity calculation for triangular profiles: `v_peak = sqrt(2*S*A*D/(A+D))`

### 2. Multi-Axis Synchronization
- Plans each axis independently
- Synchronizes all axes using the maximum time across all axes
- Axes that finish early hold their final position until all complete

### 3. Input Validation
- Validates that all accelerations, decelerations, and velocities are positive
- Validates that sampling time is positive
- Provides clear error messages for invalid inputs

### 4. Robust Handling
- Supports positive and negative displacements
- Handles zero displacement correctly
- Works with different acceleration and deceleration values
- Maintains velocity continuity

## Testing
Comprehensive test suite includes:

### Basic Tests (`test_motion_planning.m`)
- Single axis trapezoidal profile
- Single axis triangular profile
- Multi-axis synchronized motion
- Negative displacement
- Zero displacement on one axis

### Edge Case Tests (`test_motion_planning_edge_cases.m`)
- Different acceleration and deceleration values
- Very small displacements
- Vastly different axis parameters
- Mixed positive and negative displacements
- High resolution sampling
- All axes with zero displacement
- Structure field verification

### Integration Test (`test_integration.m`)
- 6-DOF robot arm simulation
- Practical usage demonstration
- Trajectory interpolation examples

### Verification (`verify_motion_planning.m`)
- Position accuracy verification
- Velocity and acceleration limit compliance
- Continuity checks
- Synchronization verification

## Usage Example
```matlab
% Define motion parameters for 3 axes
TotalLength = [1000, 500, 200];  % mm
Vmax = [500, 400, 300];          % mm/s
Accel = [2000, 2000, 2000];      % mm/s^2
Decel = [2000, 2000, 2000];      % mm/s^2
Ts = 0.001;                       % 1 ms sampling

% Generate motion plan
MotionData = MotionPlanning(TotalLength, Vmax, Accel, Decel, Ts);

% Access results
time = MotionData.t;              % Time vector
position = MotionData.s;          % Position profiles (3xM)
velocity = MotionData.v;          % Velocity profiles (3xM)
acceleration = MotionData.a;      % Acceleration profiles (3xM)
total_time = MotionData.maxtime;  % Total synchronized time
```

## Implementation Details

### Phase Calculations
1. **Acceleration Phase** (0 to t1):
   - a(t) = A
   - v(t) = A * t
   - s(t) = 0.5 * A * t²

2. **Constant Velocity Phase** (t1 to t2):
   - a(t) = 0
   - v(t) = V
   - s(t) = s_acc + V * (t - t1)

3. **Deceleration Phase** (t2 to t3):
   - a(t) = -D
   - v(t) = V - D * (t - t2)
   - s(t) = s_before + V * (t - t2) - 0.5 * D * (t - t2)²

4. **Hold Phase** (t > t3):
   - a(t) = 0
   - v(t) = 0
   - s(t) = S (final position)

## Validation Results
All tests pass successfully with:
- Position errors < 0.01 units
- Velocity limits respected
- Acceleration limits respected
- Final velocities = 0
- Proper synchronization across all axes
