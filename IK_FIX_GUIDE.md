# IK Troubleshooting Guide for Real Robot Integration

## Problem Description

When updating `note.txt` with actual robot measurements, you may encounter:
```
Error using GenerateJointTrajectory
No valid IK solution within joint limits for note Do
```

## Root Causes

This error occurs due to one or more of these issues:

1. **Euler Angle Convention Mismatch** - Different robot systems use different Euler angle conventions (12 possible!)
2. **Position Out of Workspace** - Position too far or at singular configuration
3. **Orientation Incompatible** - Required joint angles exceed limits
4. **Direct Use of Measured Pose** - Pose from FK may not be suitable as target

## Quick Solutions

### Option 1: Automatic Fix (Recommended)

Run the automatic solver:
```matlab
FixNotePositionsIK()
```

This will:
- Test all common Euler angle conventions
- Test different fixed orientations
- Find working solutions
- Provide specific recommendations

### Option 2: Use Fixed Orientation

If Euler angles don't match, use positions only with fixed orientation.

Edit `GenerateJointTrajectory.m` (line ~15):
```matlab
use_fixed_orientation = [0, 180, 0];  % Fixed downward orientation
```

This uses your positions `(539.136, y, 438.036)` but replaces the orientations with a fixed downward pose.

### Option 3: Change Euler Convention

Different robots use different conventions. Try alternatives.

Edit `GenerateJointTrajectory.m` (line ~9):
```matlab
euler_convention = 'XYZ';  % Try: 'XYZ', 'ZXY', 'YXZ', 'XZY', 'YZX'
```

Common conventions:
- `'ZYX'` - Roll-Pitch-Yaw (default, most common)
- `'XYZ'` - Alternative convention
- `'ZXY'`, `'YXZ'`, `'XZY'`, `'YZX'` - Other options

## Detailed Workflow

### Step 1: Diagnose the Problem

Run diagnostics to see what's failing:
```matlab
DiagnoseNoteIK()
```

This shows:
- Number of IK solutions per note
- Which joints violate limits
- Specific limit violations

### Step 2: Apply Automatic Fix

Let the solver find the solution:
```matlab
FixNotePositionsIK()
```

Output example:
```
Approach 1: Testing different Euler angle conventions
  ✓ XYZ: Found 4 valid solutions
  
SOLUTION FOUND!
Use Euler convention: XYZ

To apply this fix:
1. In GenerateJointTrajectory.m, change:
   euler_convention = 'XYZ';
```

### Step 3: Apply Recommended Settings

Follow the recommendations from Step 2 by editing `GenerateJointTrajectory.m`:

```matlab
function GenerateJointTrajectory()
    % ... 
    %% CONFIGURATION
    euler_convention = 'XYZ';  % ← Change based on recommendation
    use_fixed_orientation = []; % or [0, 180, 0] if recommended
    % ...
```

### Step 4: Generate Trajectory

Run the main script:
```matlab
GenerateJointTrajectory()
```

## Understanding Your Data

Your note positions:
```
Do,  539.136, -112.935, 438.036, 149.543, 179.447, -24.463
Re,  539.136,  -87.365, 438.037, 149.544, 179.447, -24.462
...
```

**Position Analysis:**
- X-distance: 539.136 mm
- Y-range: -112.935 to 40.485 mm  
- Z-height: 438.036 mm
- Distance from base: ~680 mm (within 984mm max reach ✓)

**Orientation Analysis:**
- Euler angles: (149.543°, 179.447°, -24.463°)
- These are very different from default (0°, 180°, 0°)
- Likely measured from your robot's FK
- **Issue**: Convention mismatch with IK solver

## Common Scenarios and Solutions

### Scenario 1: All notes fail with "No IK solutions"

**Cause**: Euler convention mismatch

**Solution**: Run `FixNotePositionsIK()` and use recommended convention

### Scenario 2: Some notes work, others fail

**Cause**: Specific positions/orientations exceed limits

**Solution**: 
1. Check which notes fail with `DiagnoseNoteIK()`
2. Adjust those specific positions in `note.txt`
3. Or use fixed orientation: `use_fixed_orientation = [0, 180, 0]`

### Scenario 3: "Solutions found but none within joint limits"

**Cause**: Orientation requires extreme joint angles

**Solution**: Use fixed orientation
```matlab
use_fixed_orientation = [0, 180, 0];
```

### Scenario 4: Position seems unreachable

**Cause**: Distance > 984mm or wrong units

**Solution**: 
1. Verify units are mm (not m)
2. Check calculation: `sqrt(x^2 + y^2 + z^2) < 984`
3. Adjust xylophone or robot position

## Tool Reference

| Tool | Purpose |
|------|---------|
| `FixNotePositionsIK()` | Automatic solver - finds working configuration |
| `DiagnoseNoteIK()` | Detailed diagnostics - shows exactly what's failing |
| `GenerateJointTrajectory()` | Main script - generates trajectory (modify configuration section) |
| `LoadNotePositions()` | Enhanced loader - supports multiple conventions |

## Configuration Options in GenerateJointTrajectory.m

```matlab
%% CONFIGURATION (lines 4-15)

% Option 1: Change Euler convention
euler_convention = 'XYZ';  % Try: ZYX, XYZ, ZXY, YXZ, XZY, YZX

% Option 2: Use fixed orientation
use_fixed_orientation = [0, 180, 0];  % Downward pointing

% Option 3: Use file orientations (default)
euler_convention = 'ZYX';
use_fixed_orientation = [];
```

## Advanced: Understanding Euler Conventions

Different conventions multiply rotation matrices in different orders:

- **ZYX**: R = Rz(γ) × Ry(β) × Rx(α) - Most common in robotics
- **XYZ**: R = Rx(α) × Ry(β) × Rz(γ) - Alternative
- **ZXY**: R = Rz(γ) × Rx(α) × Ry(β)
- etc.

Your robot's documentation should specify which convention it uses. If unknown, `FixNotePositionsIK()` will find it automatically.

## Example Workflow

### Problem: 
```
Error: No valid IK solution within joint limits for note Do
```

### Solution Steps:

1. **Run diagnostics:**
   ```matlab
   >> DiagnoseNoteIK
   --- Note 1: Do ---
   Position: [539.136, -112.935, 438.036] mm
   ❌ NO solutions within joint limits!
   ```

2. **Run automatic fix:**
   ```matlab
   >> FixNotePositionsIK
   Approach 2: Testing position-only with fixed orientations
     ✓ [0, 180, 0] Downward (standard): Found 6 valid solutions
   
   ✓ SOLUTION FOUND!
   Use fixed orientation: [0, 180, 0] - Downward (standard)
   ```

3. **Apply fix in GenerateJointTrajectory.m:**
   ```matlab
   use_fixed_orientation = [0, 180, 0];
   ```

4. **Run main script:**
   ```matlab
   >> GenerateJointTrajectory
   === Xylophone Playing Project: Twinkle Twinkle Little Star ===
   Using fixed orientation: [0.0, 180.0, 0.0] degrees
   ✓ Success!
   ```

## Need More Help?

If none of the above solutions work:

1. Save diagnostic output:
   ```matlab
   diary diagnosis.txt
   DiagnoseNoteIK()
   diary off
   ```

2. Check:
   - Units in note.txt (mm vs m)
   - Position measurements accuracy
   - Workspace limits

3. Consider:
   - Re-measuring positions
   - Adjusting xylophone placement
   - Using different end-effector approach

## Files Added

- `FixNotePositionsIK.m` - Automatic problem solver
- `DiagnoseNoteIK.m` - Detailed diagnostics
- `IK_FIX_GUIDE_CN.md` - This guide in Chinese
- `LoadNotePositions.m` - Enhanced with multi-convention support
- Updated `GenerateJointTrajectory.m` - Configuration section added
