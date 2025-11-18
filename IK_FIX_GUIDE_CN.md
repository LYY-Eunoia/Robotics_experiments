# 解决 IK 问题的指南 (Guide to Fixing IK Issues)

## 问题 (Problem)

当使用实际机器人测量的位置更新 `note.txt` 后，运行代码报错：
```
错误使用 GenerateJointTrajectory
No valid IK solution within joint limits for note Do
```

## 原因分析 (Root Cause Analysis)

这个错误通常由以下原因之一引起：

1. **欧拉角约定不匹配** - 不同的机器人系统使用不同的欧拉角约定（有12种！）
2. **位置超出工作空间** - 位置太远或在奇异配置处
3. **姿态不兼容** - 特定姿态需要的关节角超出限制
4. **直接使用测量位姿** - 从FK得到的位姿可能不适合作为目标位姿

## 解决方案 (Solutions)

### 方法1：自动诊断并修复（推荐）

运行自动修复脚本：
```matlab
FixNotePositionsIK()
```

这个脚本会：
- 测试所有常用的欧拉角约定
- 测试不同的固定姿态
- 找到有效的解决方案
- 给出具体的修改建议

### 方法2：手动配置

#### 步骤1：诊断问题
```matlab
DiagnoseNoteIK()
```

这会显示：
- 每个音符的IK解数量
- 哪些关节超出限制
- 具体的超限值

#### 步骤2：尝试不同的欧拉角约定

在 `GenerateJointTrajectory.m` 中修改配置部分：

```matlab
% 修改这行（第9行左右）
euler_convention = 'ZYX';  % 改为 'XYZ', 'ZXY', 'YXZ', 'XZY', 或 'YZX'
```

常用约定：
- `'ZYX'` - Roll-Pitch-Yaw（最常见）
- `'XYZ'` - 备选约定
- `'ZXY'`, `'YXZ'`, `'XZY'`, `'YZX'` - 其他约定

#### 步骤3：使用固定姿态（如果约定不匹配）

如果欧拉角约定无法解决问题，使用固定姿态：

```matlab
% 修改这行（第15行左右）
use_fixed_orientation = [0, 180, 0];  % 末端执行器向下
```

推荐的固定姿态：
- `[0, 180, 0]` - 标准向下
- `[180, 0, 180]` - 向下（旋转）
- `[0, 0, 180]` - 向下（备选）

### 方法3：修改 note.txt

如果上述方法都不work，可能需要调整位置：

#### 检查距离
```matlab
% 计算到基座的距离
x = 539.136;
y = -112.935;
z = 438.036;
distance = sqrt(x^2 + y^2 + z^2);
fprintf('Distance: %.2f mm\n', distance);
fprintf('Max reach: ~984 mm\n');
```

#### 调整位置
如果距离超过984mm，需要：
1. 移动木琴更靠近机器人
2. 调整机器人基座位置
3. 修改 note.txt 中的坐标

## 详细示例 (Detailed Example)

### 你的实际数据
```
Do,  539.136, -112.935, 438.036, 149.543, 179.447, -24.463
```

### 问题
欧拉角 `(149.543, 179.447, -24.463)` 可能使用不同的约定

### 解决方案A：尝试不同约定

修改 `GenerateJointTrajectory.m`：
```matlab
euler_convention = 'XYZ';  % 尝试XYZ约定
```

### 解决方案B：使用位置+固定姿态

修改 `GenerateJointTrajectory.m`：
```matlab
use_fixed_orientation = [0, 180, 0];  % 只使用位置，姿态固定
```

这样会：
- 保留你的位置 `(539.136, -112.935, 438.036)`
- 使用固定姿态 `(0, 180, 0)` 代替 `(149.543, 179.447, -24.463)`

## 工作流程 (Workflow)

```
1. 更新 note.txt 为实际测量值
   ↓
2. 运行 FixNotePositionsIK()
   ↓
3. 按照推荐修改 GenerateJointTrajectory.m
   ↓
4. 运行 GenerateJointTrajectory()
   ↓
5. 如果仍有问题，运行 DiagnoseNoteIK() 获取详细信息
```

## 文件说明 (File Descriptions)

| 文件 | 用途 |
|------|------|
| `FixNotePositionsIK.m` | 自动查找解决方案 |
| `DiagnoseNoteIK.m` | 诊断具体问题 |
| `LoadNotePositions.m` | 支持多种欧拉角约定 |
| `GenerateJointTrajectory.m` | 主脚本（已更新配置选项）|

## 常见问题 (FAQ)

### Q1: 为什么需要改欧拉角约定？
A: 不同机器人系统使用不同的欧拉角定义。你的机器人可能使用XYZ约定，而默认代码使用ZYX约定。

### Q2: 什么是"固定姿态"？
A: 固定姿态意味着只使用note.txt中的位置(x,y,z)，而姿态(rx,ry,rz)使用固定值。这对敲击木琴通常足够。

### Q3: 如何知道哪个约定是正确的？
A: 运行 `FixNotePositionsIK()`，它会自动测试所有约定并告诉你哪个有效。

### Q4: 距离检查显示可达，但仍无解？
A: 这通常是姿态问题。尝试使用固定姿态：
```matlab
use_fixed_orientation = [0, 180, 0];
```

## 需要帮助？ (Need Help?)

如果以上方法都不work：

1. 运行诊断并保存输出：
```matlab
DiagnoseNoteIK() > diagnosis_output.txt
```

2. 检查：
   - note.txt 使用的单位是否正确（mm而非m）
   - 位置是否在工作空间内
   - 测量是否准确

3. 考虑：
   - 重新测量位置
   - 调整木琴位置
   - 使用不同的末端执行器姿态
