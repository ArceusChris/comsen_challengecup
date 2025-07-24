# 无人机控制脚本使用说明

## 目录
- [概述](#概述)
- [项目结构](#项目结构)
- [功能特性](#功能特性)
- [使用方法](#使用方法)
- [测试脚本详解](#测试脚本详解)
- [完整工作流程](#完整工作流程)
- [监控和调试](#监控和调试)
- [故障排除](#故障排除)
- [扩展功能和高级用法](#扩展功能和高级用法)
- [性能调优建议](#性能调优建议)
- [版本历史和更新](#版本历史和更新)
- [技术支持和贡献](#技术支持和贡献)

## 概述
`drone_control.py` 是一个基于ROS的无人机速度控制脚本，参考了 `multirotor_keyboard_control.py`，支持从 `/vel_cmd` 话题订阅速度命令并控制无人机。该脚本专为XTDrone仿真环境设计，支持iris和standard_vtol等多种无人机类型。

## 项目结构
```
comsen_challengecup/
├── drone_control.py           # 主控制脚本
├── test_drone_control.py      # 测试和手动控制脚本
├── launch_physical.sh         # 物理机环境启动脚本
├── stop_all_physical.sh       # 停止所有进程脚本
├── DRONE_CONTROL_README.md    # 本说明文档
└── multirotor_keyboard_control.py # 参考的键盘控制脚本
```

## 功能特性

### 1. 速度控制
- **多格式支持**：支持 Float32MultiArray 格式 `[x, y, z, omega]` 和标准 Twist 格式
- **安全限制**：自动限制速度范围（线性速度 ±20 m/s，角速度 ±3 rad/s）
- **实时控制**：10Hz频率发布速度命令，确保平滑控制
- **坐标系统**：使用FLU（Front-Left-Up）坐标系

### 2. 自动起飞流程
实现推荐的起飞流程，符合PX4 OFFBOARD模式要求：
1. **预设速度**：设置上升速度到 0.5 m/s（大于0.3 m/s的要求）
2. **模式切换**：切换到 OFFBOARD 模式
3. **解锁起飞**：解锁无人机并开始上升
4. **状态监控**：实时监控起飞状态

### 3. 控制命令
支持完整的无人机控制命令集：
- **ARM/DISARM**：解锁/上锁无人机
- **TAKEOFF/LAND**：标准起飞/降落模式
- **OFFBOARD/HOVER**：OFFBOARD模式/悬停模式
- **RTL**：返回起飞点（Return to Launch）
- **EMERGENCY**：紧急停止所有运动
- **AUTO_TAKEOFF**：自动执行完整起飞流程
- **AUTO_LAND**：自动执行安全降落流程

### 4. 状态监控
- **实时状态**：1Hz频率发布无人机状态信息
- **日志记录**：详细的ROS日志输出
- **错误处理**：自动错误检测和恢复机制
- **线程安全**：多线程设计，避免阻塞

## 使用方法

### 1. 环境准备
```bash
# 确保ROS环境已配置
source /opt/ros/noetic/setup.bash  # 或melodic
source ~/catkin_ws/devel/setup.bash

# 检查必要的ROS包
rospack find geometry_msgs
rospack find std_msgs
```

### 2. 基本启动
```bash
# 手动控制模式
python3 drone_control.py iris 0 vel

# 自动起飞模式（推荐用于快速测试）
python3 drone_control.py iris 0 vel auto_takeoff

# 垂起无人机控制
python3 drone_control.py standard_vtol 0 vel

# 加速度控制模式
python3 drone_control.py iris 0 accel
```

### 3. 参数说明
| 参数 | 说明 | 可选值 | 示例 |
|------|------|--------|------|
| `multirotor_type` | 无人机类型 | iris, standard_vtol, px4_vision | iris |
| `multirotor_id` | 无人机编号 | 0, 1, 2, ... | 0 |
| `control_type` | 控制类型 | vel, accel | vel |
| `auto_takeoff` | 自动起飞标志 | auto_takeoff | 可选 |

### 4. 速度命令示例

#### 使用 Float32MultiArray 格式（推荐）
```bash
# 基本移动命令
rostopic pub /vel_cmd std_msgs/Float32MultiArray '{data: [1.0, 0.0, 0.0, 0.0]}'  # 前进1m/s
rostopic pub /vel_cmd std_msgs/Float32MultiArray '{data: [-1.0, 0.0, 0.0, 0.0]}' # 后退1m/s
rostopic pub /vel_cmd std_msgs/Float32MultiArray '{data: [0.0, 1.0, 0.0, 0.0]}'  # 左移1m/s
rostopic pub /vel_cmd std_msgs/Float32MultiArray '{data: [0.0, -1.0, 0.0, 0.0]}' # 右移1m/s
rostopic pub /vel_cmd std_msgs/Float32MultiArray '{data: [0.0, 0.0, 0.5, 0.0]}'  # 上升0.5m/s
rostopic pub /vel_cmd std_msgs/Float32MultiArray '{data: [0.0, 0.0, -0.5, 0.0]}' # 下降0.5m/s

# 旋转命令
rostopic pub /vel_cmd std_msgs/Float32MultiArray '{data: [0.0, 0.0, 0.0, 0.5]}'  # 逆时针旋转
rostopic pub /vel_cmd std_msgs/Float32MultiArray '{data: [0.0, 0.0, 0.0, -0.5]}' # 顺时针旋转

# 组合运动
rostopic pub /vel_cmd std_msgs/Float32MultiArray '{data: [1.0, 0.0, 0.5, 0.2]}'  # 前进+上升+旋转
rostopic pub /vel_cmd std_msgs/Float32MultiArray '{data: [0.0, 0.0, 0.0, 0.0]}'  # 悬停（零速度）

# 高级机动
rostopic pub /vel_cmd std_msgs/Float32MultiArray '{data: [2.0, 1.0, 0.0, 0.0]}'  # 斜向前进
rostopic pub /vel_cmd std_msgs/Float32MultiArray '{data: [1.0, 0.0, 0.0, 1.0]}'  # 前进时旋转
```

#### 使用 Twist 格式
```bash
# 基本Twist命令模板
rostopic pub /vel_cmd_twist geometry_msgs/Twist "
linear:
  x: 1.0    # 前进速度 (m/s)
  y: 0.0    # 左移速度 (m/s)  
  z: 0.5    # 上升速度 (m/s)
angular:
  x: 0.0    # 横滚角速度 (rad/s) - 通常为0
  y: 0.0    # 俯仰角速度 (rad/s) - 通常为0
  z: 0.2    # 偏航角速度 (rad/s)
"

# 一键命令示例
rostopic pub /vel_cmd_twist geometry_msgs/Twist '{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

### 5. 控制命令示例
```bash
# 自动化命令
rostopic pub /drone_cmd std_msgs/String "AUTO_TAKEOFF"  # 自动起飞（推荐）
rostopic pub /drone_cmd std_msgs/String "AUTO_LAND"    # 自动降落

# 基本控制命令
rostopic pub /drone_cmd std_msgs/String "ARM"          # 解锁
rostopic pub /drone_cmd std_msgs/String "DISARM"      # 上锁
rostopic pub /drone_cmd std_msgs/String "OFFBOARD"    # OFFBOARD模式
rostopic pub /drone_cmd std_msgs/String "HOVER"       # 悬停模式

# 飞行模式
rostopic pub /drone_cmd std_msgs/String "AUTO.TAKEOFF" # 标准起飞
rostopic pub /drone_cmd std_msgs/String "AUTO.LAND"    # 标准降落
rostopic pub /drone_cmd std_msgs/String "AUTO.RTL"     # 返航

# 紧急控制
rostopic pub /drone_cmd std_msgs/String "EMERGENCY"    # 紧急停止
```

### 6. ROS话题接口说明

#### 订阅话题
| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/vel_cmd` | `std_msgs/Float32MultiArray` | 速度命令 [x,y,z,ω] |
| `/vel_cmd_twist` | `geometry_msgs/Twist` | Twist格式速度命令 |
| `/drone_cmd` | `std_msgs/String` | 控制命令 |

#### 发布话题
| 话题名 | 消息类型 | 说明 |
|--------|----------|------|
| `/xtdrone/{type}_{id}/cmd_vel_flu` | `geometry_msgs/Twist` | 发送给无人机的速度命令 |
| `/xtdrone/{type}_{id}/cmd` | `std_msgs/String` | 发送给无人机的控制命令 |
| `/drone_controller/status` | `std_msgs/String` | 控制器状态信息 |

## 测试脚本详解

### 1. 自动测试序列
```bash
# 运行完整自动测试（需要先启动drone_control.py）
python3 test_drone_control.py test
```

**测试序列包括：**
1. 自动起飞流程（10秒）
2. 前进运动测试（3秒）
3. 左移运动测试（3秒）
4. 旋转运动测试（3秒）
5. 悬停测试（3秒）
6. 自动降落流程（15秒）

### 2. 手动控制界面
```bash
# 启动交互式手动控制
python3 test_drone_control.py
```

**可用控制键：**
- `1` - 自动起飞
- `2/3` - 前进/后退
- `4/5` - 左移/右移
- `6/7` - 上升/下降
- `8/9` - 左转/右转
- `0` - 悬停
- `h` - 返航
- `l` - 自动降落
- `e` - 紧急停止
- `q` - 退出

### 3. 自定义测试脚本
您可以基于test_drone_control.py创建自己的测试序列：

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Float32MultiArray

def custom_flight_test():
    rospy.init_node('custom_test')
    
    vel_pub = rospy.Publisher('/vel_cmd', Float32MultiArray, queue_size=1)
    cmd_pub = rospy.Publisher('/drone_cmd', String, queue_size=1)
    
    rospy.sleep(2.0)  # 等待连接
    
    # 自动起飞
    cmd_pub.publish(String(data="AUTO_TAKEOFF"))
    rospy.sleep(10.0)
    
    # 你的飞行序列
    # ...
    
    # 自动降落
    cmd_pub.publish(String(data="AUTO_LAND"))

if __name__ == '__main__':
    custom_flight_test()
```

## 完整工作流程

### 方法一：快速启动（推荐用于测试）
```bash
# 1. 启动仿真环境（一键启动所有必要服务）
./launch_physical.sh

# 2. 新开终端，启动控制脚本（自动起飞模式）
python3 drone_control.py iris 0 vel auto_takeoff

# 3. 新开终端，运行测试脚本
python3 test_drone_control.py test

# 4. 停止所有服务
./stop_all_physical.sh
```

### 方法二：分步启动（适合开发调试）
```bash
# 1. 启动仿真环境
./launch_physical.sh

# 2. 等待环境完全启动（约30秒）

# 3. 启动控制脚本（手动模式）
python3 drone_control.py iris 0 vel

# 4. 手动发送起飞命令
rostopic pub /drone_cmd std_msgs/String "AUTO_TAKEOFF"

# 5. 发送速度命令进行测试
rostopic pub /vel_cmd std_msgs/Float32MultiArray '{data: [1.0, 0.0, 0.0, 0.0]}'

# 6. 降落
rostopic pub /drone_cmd std_msgs/String "AUTO_LAND"
```

### 方法三：多机协同（高级用法）
```bash
# 1. 启动仿真环境
./launch_physical.sh

# 2. 启动多个控制脚本
python3 drone_control.py iris 0 vel &        # 后台运行iris_0
python3 drone_control.py standard_vtol 0 vel & # 后台运行vtol_0

# 3. 分别控制不同无人机
rostopic pub /vel_cmd std_msgs/Float32MultiArray '{data: [1.0, 0.0, 0.0, 0.0]}'  # 控制当前焦点无人机

# 4. 或者直接发送到特定无人机
rostopic pub /xtdrone/iris_0/cmd_vel_flu geometry_msgs/Twist '{linear: {x: 1.0}}'
rostopic pub /xtdrone/standard_vtol_0/cmd_vel_flu geometry_msgs/Twist '{linear: {x: 1.0}}'
```

## 监控和调试

### 1. 实时状态监控
```bash
# 查看控制器状态（推荐）
rostopic echo /drone_controller/status

# 查看所有活跃话题
rostopic list

# 监控速度命令输入
rostopic echo /vel_cmd

# 监控发送给无人机的实际命令
rostopic echo /xtdrone/iris_0/cmd_vel_flu
rostopic echo /xtdrone/iris_0/cmd

# 监控无人机状态
rostopic echo /iris_0/mavros/state
rostopic echo /iris_0/mavros/local_position/pose
```

### 2. 性能监控
```bash
# 检查话题发布频率
rostopic hz /vel_cmd
rostopic hz /xtdrone/iris_0/cmd_vel_flu

# 查看话题详细信息
rostopic info /vel_cmd
rostopic info /drone_controller/status

# 监控ROS节点状态
rosnode list
rosnode info /iris_0_velocity_controller
```

### 3. 调试工具
```bash
# 使用rqt图形界面监控
rqt &

# 绘制话题数据图表
rqt_plot /drone_controller/status &

# 3D可视化（如果可用）
rviz &

# 网络图可视化
rqt_graph &
```

### 4. 日志分析
```bash
# 查看ROS日志
cat ~/.ros/log/latest/rosout.log | grep velocity_controller

# 实时查看日志
tail -f ~/.ros/log/latest/rosout.log

# 使用roslaunch的日志
# 日志文件位置：$HOME/simulation_logs/YYYYMMDD_HHMMSS/
ls -la $HOME/simulation_logs/
tail -f $HOME/simulation_logs/*/drone_control.log
```

### 2. 日志查看
脚本会输出详细的ROS日志信息，包括：
- 控制器初始化状态
- 接收到的速度命令
- 发送的控制命令
- 自动起飞/降落流程状态

## 扩展功能和高级用法

### 1. 自定义飞行任务
脚本设计为可扩展的，可以轻松添加自定义飞行任务：

```python
class CustomMissionController(DroneController):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        
    def square_pattern_flight(self, side_length=5.0, speed=1.0):
        """执行正方形飞行模式"""
        rospy.loginfo("开始正方形飞行模式...")
        
        # 前进
        self.current_twist.linear.x = speed
        rospy.sleep(side_length / speed)
        
        # 左移
        self.current_twist.linear.x = 0.0
        self.current_twist.linear.y = speed
        rospy.sleep(side_length / speed)
        
        # 后退
        self.current_twist.linear.y = 0.0
        self.current_twist.linear.x = -speed
        rospy.sleep(side_length / speed)
        
        # 右移
        self.current_twist.linear.x = 0.0
        self.current_twist.linear.y = -speed
        rospy.sleep(side_length / speed)
        
        # 悬停
        self.current_twist = Twist()
        rospy.loginfo("正方形飞行完成")
```

### 2. 路径跟踪功能
```python
def waypoint_navigation(self, waypoints, speed=1.0):
    """航点导航功能"""
    for i, waypoint in enumerate(waypoints):
        rospy.loginfo(f"前往航点 {i+1}: {waypoint}")
        
        # 计算方向向量
        direction = self.calculate_direction_to_waypoint(waypoint)
        
        # 设置速度
        self.current_twist.linear.x = direction[0] * speed
        self.current_twist.linear.y = direction[1] * speed
        self.current_twist.linear.z = direction[2] * speed
        
        # 等待到达
        self.wait_until_reached(waypoint, tolerance=0.5)
```

### 3. 多机协调控制
```python
class SwarmController:
    def __init__(self, drone_ids):
        self.controllers = {}
        for drone_id in drone_ids:
            self.controllers[drone_id] = DroneController("iris", drone_id, "vel")
    
    def formation_flight(self, formation_type="line"):
        """编队飞行"""
        if formation_type == "line":
            for i, controller in enumerate(self.controllers.values()):
                controller.current_twist.linear.x = 1.0
                controller.current_twist.linear.y = i * 2.0  # 横向间距2米
        
    def synchronized_takeoff(self):
        """同步起飞"""
        for controller in self.controllers.values():
            threading.Thread(target=controller.auto_takeoff_sequence).start()
```

### 4. 安全增强功能
```python
def add_safety_features(self):
    """添加安全功能"""
    
    # 地理围栏
    self.geofence_limits = {
        'x_min': -50, 'x_max': 50,
        'y_min': -50, 'y_max': 50,
        'z_min': 0,   'z_max': 20
    }
    
    # 低电量保护
    self.battery_threshold = 20  # 百分比
    
    # 失控保护
    self.max_tilt_angle = 30  # 度
```

## 性能调优建议

### 1. 系统优化
```bash
# 增加系统资源限制
ulimit -n 4096

# 优化ROS网络配置
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1

# 减少不必要的日志输出
export ROSCONSOLE_CONFIG_FILE=$HOME/rosconsole.conf
```

### 2. 代码优化
- 使用合适的发布频率（推荐10-50Hz）
- 避免在回调函数中执行耗时操作
- 使用多线程处理复杂计算
- 合理设置消息队列大小

### 3. 硬件建议
- **CPU**: 至少4核，推荐8核以上
- **内存**: 最少8GB，推荐16GB以上
- **存储**: SSD硬盘，提高I/O性能
- **网络**: 千兆以太网（多机协同时）

## 版本历史和更新

### v1.0.0 (当前版本)
- ✅ 基本速度控制功能
- ✅ 自动起飞/降落流程
- ✅ 多格式消息支持
- ✅ 安全限制和错误处理
- ✅ 状态监控和日志记录

### 计划中的功能 (v1.1.0)
- 🔄 路径规划和导航
- 🔄 障碍物检测和避障
- 🔄 多机编队飞行
- 🔄 参数动态配置
- 🔄 GUI控制界面

## 技术支持和贡献

### 报告问题
如果遇到问题，请提供以下信息：
1. 操作系统版本
2. ROS版本
3. 错误日志
4. 重现步骤

### 贡献代码
欢迎提交Pull Request，请确保：
1. 代码符合PEP8规范
2. 添加必要的注释和文档
3. 测试功能正常工作
4. 更新相应的文档

## 许可证
本项目基于MIT许可证开源，详见LICENSE文件。

---
**最后更新**: 2025年7月21日  
**作者**: sbzhihang  
**联系方式**: sbzhihang@example.com
