# 视觉降落功能使用说明

## 概述

`multirotor_control.py` 现已集成了来自 `landing.py` 的视觉降落控制器功能。新的视觉降落功能提供了基于计算机视觉的精确降落控制，支持多种降落目标类型。

## 主要特性

### 1. PID控制器
- 独立的X、Y、Z轴PID控制器
- 积分项防饱和处理
- 输出限幅保护
- 动态参数调整

### 2. 状态机控制
- **SEARCHING**: 搜索降落目标
- **TRACKING**: 跟踪目标并调整位置
- **DESCENDING**: 下降到目标位置
- **LANDING**: 执行最终降落
- **LANDED**: 降落完成

### 3. 支持的目标类型
- `landing_target_camo`: 迷彩降落目标
- `landing_target_red`: 红色降落目标  
- `landing_target_custom`: 自定义降落目标

## 使用方法

### 1. 基础使用

```python
# 创建控制器
controller = DroneController(multirotor_type, multirotor_id, control_type)
multirotor_control = MultirotorControl(controller)

# 起飞
multirotor_control.takeoff(altitude=20)

# 移动到降落区域
multirotor_control.go_to_position([0, 0], stop=True)

# 执行视觉降落
success = multirotor_control.visual_landing(target_type="landing_target_camo")
```

### 2. 在任务中使用

```python
# 在execute_mission中启用视觉降落
multirotor_control.execute_mission(
    target_position=[100, 100],
    use_visual_landing=True,
    visual_target_type="landing_target_red"
)
```

### 3. 命令行演示

```bash
# 基础模式
python3 multirotor_control.py iris 0 vel demo_visual_landing

# 指定目标类型
python3 multirotor_control.py iris 0 vel demo_visual_landing landing_target_red
```

## 参数配置

### 相机参数
- `image_width`: 图像宽度 (默认: 640)
- `image_height`: 图像高度 (默认: 480)

### PID参数
- X轴: kp=0.8, ki=0.1, kd=0.15
- Y轴: kp=0.8, ki=0.1, kd=0.15  
- Z轴: kp=0.6, ki=0.05, kd=0.12

### 降落参数
- `landing_threshold`: 像素误差阈值 (默认: 30px)
- `min_altitude`: 最小安全高度 (默认: 0.8m)
- `descent_rate`: 降落速率 (默认: 0.25 m/s)
- `target_timeout`: 目标丢失超时 (默认: 3.0s)

## 工作流程

1. **搜索阶段**: 无人机悬停并等待检测到降落目标
2. **跟踪阶段**: 检测到目标后，使用PID控制器调整位置使目标居中
3. **下降阶段**: 当目标足够居中时开始下降，同时继续微调位置
4. **降落阶段**: 到达最小安全高度后执行最终降落
5. **完成阶段**: 降落完成

## 错误处理

- **目标丢失**: 如果在跟踪或下降过程中目标丢失超过设定时间，系统会返回搜索状态
- **超时保护**: 各个阶段都有超时保护机制
- **安全限制**: 设置最小安全高度防止过度下降

## 注意事项

1. 确保降落目标话题正常发布
2. 调整PID参数以适应具体的无人机和环境
3. 在实际使用前先在安全环境中测试
4. 确保有足够的光照条件用于目标检测
