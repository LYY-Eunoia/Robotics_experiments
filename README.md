# Xylophone Playing Robot Project

This project transforms a robot stacking experiment into a xylophone-playing demonstration that performs "Twinkle Twinkle Little Star".

## Overview

The robot arm is programmed to strike xylophone keys in sequence to play a musical melody. The system computes inverse kinematics for each note position, plans smooth trajectories between notes, and executes striking actions using joint 5 rotation.

## Files

### Main Scripts
- **GenerateJointTrajectory.m** - Main script to generate the complete trajectory for playing the xylophone
- **note.txt** - Configuration file containing xylophone key positions (Do, Re, Mi, Fa, Sol, La, Si)

### Supporting Functions
- **LoadNotePositions.m** - Reads note positions from note.txt and converts Euler angles to rotation matrices
- **GetTwinkleTwinkleScore.m** - Defines the musical score for "Twinkle Twinkle Little Star"
- **PlanStrikeAction.m** - Plans the striking motion for joint 5 (90-degree rotation and return)
- **PlanJointSpaceMotion.m** - Plans smooth joint-space motion between configurations using 5th-order polynomials

### Existing Functions (Used)
- **Ikine6s.m** - Inverse kinematics solver for 6-DOF robot
- **Fkine.m** - Forward kinematics solver
- **LFPB.m** - Linear Function with Parabolic Blends trajectory planner (not used in xylophone project)

## Usage

1. **Run the main script:**
   ```matlab
   GenerateJointTrajectory
   ```

2. **Select initial configuration:**
   - The script will display available IK solutions for the first note
   - Enter the number corresponding to your preferred configuration
   - Subsequent notes will automatically select the closest configuration for smooth motion

3. **Output files:**
   - `joint_traj_data.mat` - Complete trajectory data (MATLAB format)
   - `joint_traj_data.csv` - Trajectory data in CSV format (time, positions, velocities, accelerations)
   - `musical_score.csv` - Musical score information (note names, timings)

## Xylophone Configuration (note.txt)

The xylophone keys are arranged in a horizontal line with the following layout:

| Note | X (mm) | Y (mm)  | Z (mm) | Orientation (rx, ry, rz) |
|------|--------|---------|--------|--------------------------|
| Do   | 400.0  | -200.0  | 800.0  | 0°, 180°, 0°            |
| Re   | 400.0  | -133.3  | 800.0  | 0°, 180°, 0°            |
| Mi   | 400.0  | -66.7   | 800.0  | 0°, 180°, 0°            |
| Fa   | 400.0  | 0.0     | 800.0  | 0°, 180°, 0°            |
| Sol  | 400.0  | 66.7    | 800.0  | 0°, 180°, 0°            |
| La   | 400.0  | 133.3   | 800.0  | 0°, 180°, 0°            |
| Si   | 400.0  | 200.0   | 800.0  | 0°, 180°, 0°            |

You can modify these positions in `note.txt` to match your actual xylophone setup.

## Musical Score

"Twinkle Twinkle Little Star" is encoded as:
- Do Do Sol Sol La La Sol
- Fa Fa Mi Mi Re Re Do
- Sol Sol Fa Fa Mi Mi Re
- Sol Sol Fa Fa Mi Mi Re
- Do Do Sol Sol La La Sol
- Fa Fa Mi Mi Re Re Do

Time intervals:
- Quarter notes: 0.5 seconds
- Half notes (end of phrases): 1.0 seconds

## Motion Planning Details

### Trajectory Generation Process

1. **Load Configuration:** Read note positions and musical score
2. **Inverse Kinematics:** Compute joint angles for each note position
3. **Solution Selection:** Choose IK solutions for smooth continuous motion
4. **Motion Planning:** For each note:
   - Plan joint-space motion from previous note to current note
   - Execute striking action (joint 5 rotates 90° down and back up)
5. **Data Generation:** Sample trajectory at 1ms intervals
6. **Visualization:** Generate plots of joint angles, velocities, and accelerations
7. **Save Data:** Export trajectory data in MAT and CSV formats

### Striking Action

- **Joint:** Joint 5 (wrist rotation)
- **Angle:** 90 degrees (π/2 radians)
- **Duration:** 0.1 seconds down + 0.1 seconds up = 0.2 seconds total
- **Motion Profile:** Smooth sinusoidal acceleration/deceleration

### Joint Space Motion

- **Method:** 5th-order polynomial (quintic) trajectory
- **Boundary Conditions:** Zero velocity and acceleration at start and end
- **Smoothness:** C² continuous (position, velocity, and acceleration)

## Visualizations

The script generates four figure windows:

1. **Joint Angles** - Position vs time for all 6 joints
2. **Joint Velocities** - Velocity vs time for all 6 joints
3. **Joint Accelerations** - Acceleration vs time for all 6 joints
4. **Musical Score Timeline** - Note events and joint 5 striking motion

## Customization

### Change the Song

Edit `GetTwinkleTwinkleScore.m` to modify:
- `note_sequence` - Array of note names
- `time_intervals` - Duration for each note

### Adjust Xylophone Positions

Edit `note.txt` to change:
- Position coordinates (x, y, z in mm)
- Orientation (rx, ry, rz in degrees)

### Modify Striking Parameters

In `GenerateJointTrajectory.m`, adjust:
- `strike_angle` - Rotation angle for striking (default: 90°)
- `strike_duration` - Time for strike motion (default: 0.1s)
- `sample_time` - Trajectory sampling rate (default: 1ms)

## Robot Parameters

- **Link lengths:**
  - L1 = 491 mm (base to joint 2)
  - L2 = 450 mm (joint 2 to joint 3)
  - L3 = 450 mm (joint 3 to wrist)
  - L4 = 84 mm (wrist to end-effector)

- **Joint limits:**
  - Joint 1: -170° to 170°
  - Joint 2: -120° to 120°
  - Joint 3: -140° to 140°
  - Joint 4: -170° to 170°
  - Joint 5: -120° to 120°
  - Joint 6: -360° to 360°

## Dependencies

- MATLAB (tested with R2020a or later)
- No additional toolboxes required

## Notes

- The trajectory is generated entirely in joint space for predictable motion
- All transitions use smooth polynomial trajectories to avoid jerky motion
- The striking action is localized to joint 5 to minimize disturbance to positioning
- IK solutions are selected to minimize joint motion between consecutive notes
