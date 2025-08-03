# 多旋翼无人机PID动态参数调整

本功能允许在无人机运行过程中实时调整PID控制参数，无需重启节点。

## 功能特性

- **实时参数调整**: 在无人机飞行过程中动态修改PID参数
- **自动参数同步**: 参数变化每0.5秒自动检测并应用
- **多种调整方式**: 支持命令行、ROS参数服务器、配置文件等多种方式
- **参数预设**: 内置保守、激进、默认等多种参数预设
- **安全保护**: 参数范围检查和异常处理

## 支持的参数

### PID控制参数
- `pid_x/kp`, `pid_x/ki`, `pid_x/kd`: X轴PID参数
- `pid_y/kp`, `pid_y/ki`, `pid_y/kd`: Y轴PID参数  
- `pid_z/kp`, `pid_z/ki`, `pid_z/kd`: Z轴PID参数

### 速度和运动参数
- `max_vel_xy`: XY方向最大速度 (m/s)
- `max_vel_z`: Z方向最大速度 (m/s)

### 视觉降落参数
- `landing_threshold`: 降落像素误差阈值
- `min_altitude`: 最小安全高度 (m)
- `descent_rate`: 降落速率 (m/s)
- `target_timeout`: 目标丢失超时时间 (s)

## 使用方法

### 1. 启动带参数调整功能的节点

```bash
# 使用默认参数启动
roslaunch iris_control multirotor_with_pid_tuning.launch

# 或指定特定参数启动
roslaunch iris_control multirotor_with_pid_tuning.launch multirotor_type:=iris multirotor_id:=1
```

### 2. 使用交互式PID调优工具

```bash
# 启动PID调优工具
rosrun iris_control pid_tuner.py

# 或指定特定节点名称
rosrun iris_control pid_tuner.py multirotor_control_node
```

#### 调优工具命令

```
PID调优> help              # 显示帮助
PID调优> show              # 显示所有当前参数
PID调优> set pid_x_kp 3.0  # 设置X轴比例增益为3.0
PID调优> get pid_x_kp      # 获取X轴比例增益
PID调优> preset aggressive # 加载激进参数预设
PID调优> quit              # 退出
```

### 3. 使用ROS命令行直接调整

```bash
# 调整X轴PID参数
rosparam set /multirotor_control_node/pid_x/kp 3.0
rosparam set /multirotor_control_node/pid_x/ki 0.1
rosparam set /multirotor_control_node/pid_x/kd 0.05

# 调整速度限制
rosparam set /multirotor_control_node/max_vel_xy 2.0
rosparam set /multirotor_control_node/max_vel_z 1.5

# 调整降落参数
rosparam set /multirotor_control_node/landing_threshold 25
rosparam set /multirotor_control_node/descent_rate 0.3
```

### 4. 使用配置文件

编辑 `config/multirotor_pid_params.yaml` 文件，然后重新加载参数：

```bash
rosparam load $(rospack find iris_control)/config/multirotor_pid_params.yaml /multirotor_control_node/
```

## 参数调优指南

### PID参数调优原则

1. **比例增益 (Kp)**
   - 增加Kp: 响应更快，但可能引起振荡
   - 减少Kp: 响应更慢，但更稳定
   - 推荐范围: 0.5 - 5.0

2. **积分增益 (Ki)**
   - 增加Ki: 消除稳态误差，但可能引起超调
   - 减少Ki: 减少超调，但可能有稳态误差
   - 推荐范围: 0.0 - 0.5

3. **微分增益 (Kd)**
   - 增加Kd: 减少振荡和超调
   - 减少Kd: 可能增加振荡
   - 推荐范围: 0.0 - 1.0

### 调优步骤

1. **从默认参数开始**: 使用 `preset default`
2. **调整响应速度**: 修改Kp值
3. **消除稳态误差**: 适当增加Ki值
4. **减少振荡**: 适当增加Kd值
5. **调整速度限制**: 根据飞行环境调整max_vel参数
6. **优化降落参数**: 根据视觉精度调整landing_threshold

### 预设参数说明

- **default**: 平衡的默认参数，适合大多数场景
- **conservative**: 保守参数，响应慢但稳定，适合精确任务
- **aggressive**: 激进参数，响应快，适合快速机动

## 监控和调试

### 查看参数更新日志
```bash
# 查看节点日志
rosnode info /multirotor_control_node
rostopic echo /rosout | grep "multirotor_control"
```

### 实时监控参数
```bash
# 监控特定参数
rosparam get /multirotor_control_node/pid_x/kp

# 监控所有参数
rosparam list | grep multirotor_control_node
```

## 注意事项

1. **安全第一**: 在实际飞行中调整参数要小心，建议先在仿真中测试
2. **渐进调整**: 每次只调整一个参数，观察效果后再继续
3. **记录参数**: 找到好的参数组合后，记录到配置文件中
4. **应急处理**: 如果参数调整导致异常，可以快速加载默认预设

## 故障排除

### 常见问题

1. **参数不生效**: 检查参数名称是否正确，节点是否正常运行
2. **响应过慢**: 增加Kp值或检查speed_limits参数
3. **振荡严重**: 减少Kp值或增加Kd值
4. **降落不精确**: 调整landing_threshold和descent_rate

### 恢复默认参数
```bash
# 使用调优工具
PID调优> preset default

# 或使用命令行
rosparam load $(rospack find iris_control)/config/multirotor_pid_params.yaml /multirotor_control_node/
```

## 示例调优场景

### 场景1: 提高响应速度
```bash
rosparam set /multirotor_control_node/pid_x/kp 3.0
rosparam set /multirotor_control_node/pid_y/kp 3.0
rosparam set /multirotor_control_node/max_vel_xy 2.0
```

### 场景2: 提高降落精度
```bash
rosparam set /multirotor_control_node/landing_threshold 20
rosparam set /multirotor_control_node/descent_rate 0.15
rosparam set /multirotor_control_node/pid_x/kp 2.5
rosparam set /multirotor_control_node/pid_y/kp 2.5
```

### 场景3: 在风力环境中稳定飞行
```bash
rosparam set /multirotor_control_node/pid_x/kd 0.1
rosparam set /multirotor_control_node/pid_y/kd 0.1
rosparam set /multirotor_control_node/pid_z/kd 0.05
rosparam set /multirotor_control_node/max_vel_xy 1.0
```
