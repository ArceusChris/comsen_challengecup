# VTOL Control Launch Files

此目录包含用于启动VTOL演示节点的launch文件。

## Launch文件说明

### 1. vtol_demo_simple.launch
最简单的launch文件，只启动VTOL演示节点。
- **用途**: 当MAVROS和其他依赖已经在其他地方启动时使用
- **启动命令**: 
  ```bash
  roslaunch vtol_control vtol_demo_simple.launch
  ```
- **参数**:
  - `vehicle_type`: 飞机类型 (默认: standard_vtol)
  - `vehicle_id`: 飞机ID (默认: 0)

### 2. vtol_demo.launch
包含MAVROS节点的完整launch文件。
- **用途**: 启动MAVROS和VTOL演示节点
- **启动命令**: 
  ```bash
  roslaunch vtol_control vtol_demo.launch
  ```
- **参数**:
  - `vehicle_type`: 飞机类型 (默认: standard_vtol)
  - `vehicle_id`: 飞机ID (默认: 0)
  - `fcu_url`: 飞控连接URL (默认: udp://:14540@localhost:14557)
  - `tgt_system`: 目标系统ID (默认: 1)
  - `tgt_component`: 目标组件ID (默认: 1)

### 3. vtol_demo_sitl.launch
完整的仿真环境launch文件，包含Gazebo、PX4 SITL和MAVROS。
- **用途**: 完整的仿真环境测试
- **启动命令**: 
  ```bash
  roslaunch vtol_control vtol_demo_sitl.launch
  ```
- **参数**:
  - `vehicle_type`: 飞机类型 (默认: standard_vtol)
  - `vehicle_id`: 飞机ID (默认: 0)
  - `world`: Gazebo世界文件
  - `gui`: 是否启用Gazebo GUI (默认: true)
  - `headless`: 是否启用无头模式 (默认: false)

## 使用流程

### 方式一: 使用现有仿真环境
如果您已经运行了PX4 SITL和Gazebo仿真，可以直接使用：
```bash
roslaunch vtol_control vtol_demo_simple.launch
```

### 方式二: 启动MAVROS和演示节点
如果您需要连接到真实飞控或已有的PX4实例：
```bash
roslaunch vtol_control vtol_demo.launch fcu_url:="your_fcu_url"
```

### 方式三: 完整仿真环境
从零开始启动完整的仿真环境：
```bash
# 首先确保PX4固件已编译
cd /root/PX4_Firmware
make px4_sitl_default gazebo

# 然后启动完整仿真
roslaunch vtol_control vtol_demo_sitl.launch
```

## 前置条件

确保以下软件包已安装并配置：
- ROS (Melodic/Noetic)
- MAVROS
- PX4 Firmware
- Gazebo (仅用于仿真)

## 故障排除

1. **节点无法启动**: 检查Python脚本是否有执行权限
   ```bash
   chmod +x /root/workspace/src/vtol_control/scripts/vtol_demo.py
   ```

2. **MAVROS连接失败**: 检查FCU URL是否正确，确保PX4实例正在运行

3. **仿真环境启动失败**: 确保Gazebo模型路径正确，PX4固件已正确编译

## 自定义参数

您可以通过命令行参数自定义启动配置：
```bash
# 启动特定车辆类型
roslaunch vtol_control vtol_demo.launch vehicle_type:="iris_2d_lidar" vehicle_id:="1"

# 无GUI仿真
roslaunch vtol_control vtol_demo_sitl.launch gui:="false"

# 连接到特定FCU
roslaunch vtol_control vtol_demo.launch fcu_url:="tcp://192.168.1.100:5760"
```
