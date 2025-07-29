# 视觉降落控制器使用说明

## 功能描述

这个控制器通过下视摄像头获取降落平台的视觉信息，控制无人机精确降落到目标位置。

## 主要特性

1. **多目标支持**: 支持订阅不同的降落目标话题
   - `/landing_target_camo` - 迷彩目标检测
   - `/landing_target_red` - 红色目标检测

2. **状态机控制**: 实现完整的降落流程
   - SEARCHING: 搜索目标
   - TRACKING: 跟踪目标并调整位置
   - DESCENDING: 精确下降
   - LANDING: 执行最终降落
   - LANDED: 降落完成

3. **PD控制**: 使用比例-微分控制器确保平稳精确的控制

4. **安全机制**: 
   - 目标丢失检测和超时保护
   - 最小安全高度限制
   - 速度限制保护

## 使用方法

### 1. 基本启动

```bash
# 使用默认目标话题 (landing_target_camo)
python3 landing.py

# 指定目标话题
python3 landing.py landing_target_red
```

### 2. 启动降落过程

发布降落使能命令：
```bash
# 启用降落
rostopic pub /visual_landing/enable std_msgs/Bool "data: true"

# 停止降落
rostopic pub /visual_landing/enable std_msgs/Bool "data: false"
```

### 3. 监控状态

查看降落状态：
```bash
rostopic echo /visual_landing/status
```

查看像素误差：
```bash
rostopic echo /visual_landing/error
```

## 参数配置

### 控制参数
- `kp_x, kp_y`: XY方向比例增益 (默认: 0.8)
- `kp_z`: Z方向比例增益 (默认: 0.4)
- `kd_x, kd_y`: XY方向微分增益 (默认: 0.15)
- `kd_z`: Z方向微分增益 (默认: 0.08)

### 降落参数
- `landing_threshold`: 像素误差阈值 (默认: 30像素)
- `min_altitude`: 最小安全高度 (默认: 0.8米)
- `descent_rate`: 降落速率 (默认: 0.25米/秒)
- `max_vel_xy`: XY方向最大速度 (默认: 1.5米/秒)
- `target_timeout`: 目标丢失超时 (默认: 3秒)

## 话题接口

### 订阅话题
- `/iris_0/mavros/state` - 无人机状态
- `/iris_0/mavros/local_position/pose` - 位置信息
- `/{target_topic}` - 降落目标位置 (PointStamped)
- `/visual_landing/enable` - 降落使能命令

### 发布话题
- `/xtdrone/iris_0/cmd_vel_flu` - 速度控制命令
- `/xtdrone/iris_0/cmd` - 飞行指令
- `/visual_landing/status` - 降落状态
- `/visual_landing/error` - 像素误差信息

## 工作流程

1. **初始化**: 启动控制器，等待降落使能命令
2. **目标搜索**: 等待检测到降落目标
3. **位置调整**: 通过XY方向移动将目标居中
4. **精确下降**: 当目标居中后开始下降，同时保持目标跟踪
5. **最终降落**: 到达最小高度后执行mavros降落命令

## 注意事项

1. 确保无人机已经起飞并处于offboard模式
2. 确保下视摄像头工作正常，目标检测算法已启动
3. 建议在良好光照条件下使用
4. 降落前确认降落区域安全
5. 可以随时发送disable命令停止降落过程

## 故障排除

### 目标检测不稳定
- 检查光照条件
- 调整目标检测算法参数
- 确认降落标识清晰可见

### 降落过程震荡
- 减少PD控制器增益
- 检查无人机姿态控制器调参
- 降低控制频率

### 目标频繁丢失
- 增加`target_timeout`参数
- 检查目标检测算法稳定性
- 优化图像处理性能
