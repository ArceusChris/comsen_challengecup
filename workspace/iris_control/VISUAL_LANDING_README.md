# 视觉降落控制器集成说明

## 概述

已成功将视觉降落控制器集成到 `MultirotorControl` 类中。现在该类支持两种降落模式：
1. **常规降落** - 降落到指定高度
2. **视觉降落** - 基于摄像头视觉信息精确降落到目标平台

## 新增功能

### 1. 视觉降落方法

```python
def visual_landing(self, target_type="landing_target_camo"):
    """
    基于视觉的精确降落控制
    Args:
        target_type: 目标类型，可选 "landing_target_camo" 或 "landing_target_red"
    Returns:
        bool: 降落是否成功
    """
```

### 2. 增强的execute_mission方法

```python
def execute_mission(self, altitude_high=20, altitude_low=0.65, target_position=[1495, -105], 
                   use_visual_landing=False, target_type="landing_target_camo"):
    """
    执行完整任务：起飞 -> 移动到目标 -> 降落
    Args:
        altitude_high: 起飞高度
        altitude_low: 降落高度（仅在非视觉降落时使用）
        target_position: 目标位置
        use_visual_landing: 是否使用视觉降落
        target_type: 视觉降落目标类型
    """
```

## 使用方法

### 方法1：直接调用视觉降落

```python
from drone_control import DroneController
from multirotor_control import MultirotorControl

# 创建控制器
controller = DroneController("iris", 0, "vel")
multirotor_control = MultirotorControl(controller)

# 使用伪装目标降落
success = multirotor_control.visual_landing("landing_target_camo")

# 或使用红色目标降落
success = multirotor_control.visual_landing("landing_target_red")
```

### 方法2：在完整任务中使用视觉降落

```python
# 执行带视觉降落的完整任务
success = multirotor_control.execute_mission(
    altitude_high=15.0,           # 起飞到15米
    target_position=[100, 200],   # 移动到目标位置
    use_visual_landing=True,      # 启用视觉降落
    target_type="landing_target_red"  # 使用红色目标
)
```

### 方法3：在原有代码中使用

原有的 `multirotor_control.py` 已经升级，您可以直接修改其中的 `execute_mission` 调用：

```python
# 原来的调用
multirotor_control.execute_mission(target_position=original_poses[1])

# 新的调用（启用视觉降落）
multirotor_control.execute_mission(
    target_position=original_poses[1],
    use_visual_landing=True,
    target_type="landing_target_camo"
)
```

## 命令行使用

### 测试脚本

提供了 `test_visual_landing.py` 测试脚本：

```bash
# 基本测试
python3 test_visual_landing.py iris 0 vel landing_target_camo

# 使用红色目标
python3 test_visual_landing.py iris 0 vel landing_target_red
```

### 原有脚本集成

在原有的 `multirotor_control.py` 中，可以通过修改以下部分来启用视觉降落：

```python
# 在 main() 函数中修改
multirotor_control.execute_mission(
    target_position=original_poses[1],
    use_visual_landing=True,  # 启用视觉降落
    target_type="landing_target_camo"  # 或 "landing_target_red"
)
```

## 参数配置

### PID控制参数

视觉降落使用独立的PID控制器，可以在 `MultirotorControl.__init__()` 中调整：

```python
# X轴PID参数
self.kp_x = 0.8      # 比例增益
self.ki_x = 0.1      # 积分增益
self.kd_x = 0.15     # 微分增益

# Y轴PID参数
self.kp_y = 0.8
self.ki_y = 0.1
self.kd_y = 0.15

# Z轴PID参数
self.kp_z = 0.6
self.ki_z = 0.05
self.kd_z = 0.12
```

### 降落参数

```python
self.landing_threshold = 30    # 像素误差阈值（开始降落的条件）
self.min_altitude = 0.8        # 最小安全高度
self.descent_rate = 0.25       # 降落速率 m/s
self.max_vel_xy = 1.5          # XY方向最大速度
self.max_vel_z = 0.8           # Z方向最大速度
self.target_timeout = 3.0      # 目标丢失超时时间(秒)
```

## 状态机说明

视觉降落采用状态机控制：

1. **SEARCHING** - 搜索降落目标
2. **TRACKING** - 跟踪目标并调整位置
3. **DESCENDING** - 保持跟踪同时下降
4. **LANDING** - 执行最终降落
5. **LANDED** - 降落完成

## 话题要求

确保以下话题正常发布：

- `/landing_target_camo` - 伪装目标位置 (geometry_msgs/PointStamped)
- `/landing_target_red` - 红色目标位置 (geometry_msgs/PointStamped)

话题消息格式：
```
header:
  stamp: [时间戳]
  frame_id: "camera_frame"
point:
  x: [目标在图像中的x坐标，像素]
  y: [目标在图像中的y坐标，像素]
  z: 0.0
```

## 注意事项

1. **安全高度**: 视觉降落会在 `min_altitude` 高度时切换到最终降落模式
2. **目标丢失**: 如果目标丢失超过 `target_timeout` 秒，会自动切换到搜索状态
3. **PID调试**: 系统会输出详细的PID控制信息，便于调试
4. **兼容性**: 原有的常规降落功能保持不变，可以同时使用

## 故障排除

### 常见问题

1. **目标不断丢失**
   - 检查摄像头是否正常工作
   - 确认目标检测算法正常发布话题
   - 调整 `target_timeout` 参数

2. **降落不稳定**
   - 调整PID参数，特别是 `kp_x`, `kp_y`
   - 检查 `landing_threshold` 是否合适
   - 调整 `descent_rate` 降低下降速度

3. **无法检测到目标**
   - 确认话题名称正确 (`landing_target_camo` 或 `landing_target_red`)
   - 检查话题是否正常发布：`rostopic echo /landing_target_camo`
   - 确认相机参数设置正确

### 调试命令

```bash
# 检查话题
rostopic list | grep landing_target

# 监控目标检测
rostopic echo /landing_target_camo

# 监控无人机状态
rostopic echo /iris_0/mavros/local_position/pose
```

## 集成完成

视觉降落控制器已成功集成到 `MultirotorControl` 类中，您现在可以：

1. 使用 `visual_landing()` 方法进行视觉降落
2. 在 `execute_mission()` 中启用视觉降落选项
3. 根据不同目标类型选择相应的检测话题
4. 保持原有功能的完整性和兼容性
