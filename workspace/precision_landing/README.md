# 精准降落包 (Precision Landing)

基于视觉检测的无人机精准降落控制系统，支持多种降落平台图案检测。

## 功能特性

- 🎯 **多图案支持**: 支持基础白色、迷彩和红色降落平台
- 🤖 **智能状态机**: 完整的降落流程管理
- 📐 **精准控制**: PID控制器和视觉伺服控制
- 🔄 **实时监控**: 完整的状态反馈和调试信息
- 🛡️ **安全机制**: 多重安全检查和紧急中止功能

## 系统架构

### 核心组件

1. **precision_landing_controller.py** - 精准降落控制器
   - PID位置控制
   - 多检测器融合
   - 安全降落逻辑

2. **landing_state_machine.py** - 降落状态机
   - 完整降落流程管理
   - 状态转换和监控
   - 自动搜索和接近

3. **visual_servo_controller.py** - 视觉伺服控制器
   - 基于图像特征的控制
   - 交互矩阵计算
   - 高精度视觉对准

4. **test_precision_landing.py** - 交互式测试客户端
   - 实时状态监控
   - 键盘控制接口
   - 调试信息显示

### 状态机流程

```
IDLE → SEARCHING → APPROACHING → ALIGNING → DESCENDING → FINAL_DESCENT → LANDED
  ↓                    ↓              ↓           ↓            ↓
ABORTED ←─────────────────────────────────────────────────────┘
```

## 话题接口

### 输入话题
- `/iris_0/camera/image_raw` - 摄像头图像
- `/landing_target*` - 降落平台检测结果
- `/iris_0/mavros/*` - 飞行器状态信息

### 输出话题
- `/xtdrone/iris_0/cmd_vel_flu` - 速度控制命令
- `/xtdrone/iris_0/cmd` - 飞行模式命令
- `/precision_landing/state` - 降落状态
- `/precision_landing/status` - 控制器状态
- `/precision_landing/progress` - 降落进度

### 控制话题
- `/precision_landing/start` - 开始降落
- `/precision_landing/abort` - 中止降落
- `/precision_landing/enable` - 使能精准控制

## 使用方法

### 1. 编译包
```bash
cd ~/catkin_ws
catkin build precision_landing
source devel/setup.bash
```

### 2. 启动系统

#### 完整系统启动
```bash
# 启动精准降落系统(多图案检测)
roslaunch precision_landing precision_landing.launch

# 启动测试环境(包含调试界面)
roslaunch precision_landing test_precision_landing.launch
```

#### 指定检测器启动
```bash
# 使用迷彩图案检测器
roslaunch precision_landing precision_landing.launch detector_type:=camouflage

# 使用红色图案检测器
roslaunch precision_landing precision_landing.launch detector_type:=red

# 启用视觉伺服控制
roslaunch precision_landing precision_landing.launch enable_visual_servo:=true
```

### 3. 测试和操作

#### 使用测试客户端
```bash
# 启动交互式测试客户端
rosrun precision_landing test_precision_landing.py
```

测试客户端控制键：
- `s` - 开始降落序列
- `a` - 中止降落
- `e` - 启用精准控制器
- `d` - 禁用精准控制器
- `r` - 重置到IDLE状态
- `q` - 退出

#### 手动控制
```bash
# 开始降落
rostopic pub /precision_landing/start std_msgs/Bool "data: true"

# 中止降落
rostopic pub /precision_landing/abort std_msgs/Bool "data: true"

# 启用精准控制
rostopic pub /precision_landing/enable std_msgs/Bool "data: true"
```

### 4. 监控状态
```bash
# 查看降落状态
rostopic echo /precision_landing/state

# 查看降落进度
rostopic echo /precision_landing/progress

# 查看控制状态
rostopic echo /precision_landing/status

# 查看位置误差
rostopic echo /precision_landing/error
```

## 参数配置

### 控制器参数
```yaml
# 图像参数
image_width: 640
image_height: 480
camera_fov_h: 60.0  # 水平视场角
camera_fov_v: 45.0  # 垂直视场角

# PID参数
pid_x_kp: 0.5
pid_x_ki: 0.01
pid_x_kd: 0.1

# 降落参数
landing_threshold: 50   # 像素误差阈值
min_altitude: 0.5      # 最小降落高度
descent_rate: 0.3      # 下降速率
max_vel: 2.0           # 最大速度限制
```

### 状态机参数
```yaml
search_altitude: 10.0      # 搜索高度
approach_altitude: 5.0     # 接近高度
alignment_altitude: 3.0    # 对准高度
final_altitude: 1.0        # 最终下降高度
search_timeout: 30.0       # 搜索超时时间
approach_timeout: 15.0     # 接近超时时间
```

## 安全特性

### 多重安全检查
1. **高度限制**: 最小安全高度保护
2. **目标丢失处理**: 自动悬停和重新搜索
3. **超时保护**: 各阶段超时自动处理
4. **紧急中止**: 随时可中止降落序列
5. **连接检查**: 飞行器连接状态监控

### 故障处理
- 目标丢失 → 返回搜索状态
- 接近超时 → 重新搜索
- 下降过程目标丢失 → 紧急中止
- 通信中断 → 自动悬停

## 调试和优化

### 1. 参数调优
- 根据实际摄像头参数调整视场角
- 根据飞行高度调整PID参数
- 根据降落平台大小调整阈值

### 2. 调试工具
```bash
# 查看调试图像
rqt_image_view

# 监控话题
rqt_topic
rqt_plot

# 查看日志
rqt_console
```

### 3. 性能监控
```bash
# 控制频率监控
rostopic hz /xtdrone/iris_0/cmd_vel_flu

# 检测延迟监控
rostopic hz /landing_target*
```

## 依赖项

- ROS Noetic
- landing_detection 包
- MAVROS
- XTDrone
- OpenCV
- NumPy

## 注意事项

1. **启动顺序**: 先启动Gazebo仿真环境，再启动精准降落系统
2. **图案要求**: 确保降落平台图案清晰，对比度足够
3. **光照条件**: 避免强光直射或过暗环境
4. **安全距离**: 保持足够的安全操作空间
5. **参数调整**: 根据实际环境调整检测和控制参数

## 故障排除

### 常见问题
1. **无法检测到目标**:
   - 检查降落平台图案
   - 调整检测器参数
   - 确认摄像头工作正常

2. **控制不稳定**:
   - 调整PID参数
   - 检查控制频率
   - 优化图像处理延迟

3. **降落精度不够**:
   - 使用视觉伺服控制器
   - 降低下降速率
   - 增加对准时间

4. **状态机卡住**:
   - 检查超时参数设置
   - 查看日志信息
   - 手动重置状态
