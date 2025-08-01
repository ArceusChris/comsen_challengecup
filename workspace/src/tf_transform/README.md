# TF Transform 坐标变换模块

这个目录包含了多无人机系统中相机坐标系变换的完整解决方案。

## 文件说明

### 核心脚本
- `mavros_pose_to_tf.py` - 从MAVROS位姿话题动态发布TF变换的Python脚本
- `camera_tf_publishers.launch` - 启动动态TF发布器的ROS启动文件
- `static_tf_publishers.launch` - 静态TF变换的备用启动文件

### 工具脚本
- `check_camera_frames.sh` - 检查相机坐标系配置的诊断脚本
- `camera_config_summary.sh` - 显示配置总结和使用说明

## 坐标系结构

### iris_0 无人机
```
map
└── iris_0/base_link (动态，来自 /iris_0/mavros/local_position/pose)
    ├── iris_0/stereo_camera_frame (前方0.1m，标准相机坐标系)
    └── iris_0/camera_link (下方0.03m，标准相机坐标系，z朝地面)
```

### standard_vtol_0 无人机
```
map
└── standard_vtol_0/base_link (动态，来自 /standard_vtol_0/mavros/local_position/pose)
    └── standard_vtol_0/camera_link (下方0.03m，标准相机坐标系，z朝地面)
```

## 相机坐标系约定

本模块使用标准的相机坐标系约定：
- **X轴**：指向右
- **Y轴**：指向下
- **Z轴**：指向相机正前方（光轴方向）

### 坐标变换说明
- **前视相机**：从机体坐标系(x前y左z上)转换到相机坐标系(x右y下z前)
- **下视相机**：从机体坐标系(x前y左z上)转换到下视相机坐标系(x右y下z朝地面)

## 使用方法

### 1. 启动动态TF发布器
```bash
# 推荐方式 - 实时跟踪无人机位姿
roslaunch tf_transform camera_tf_publishers.launch
```

### 2. 启动静态TF发布器（备用）
```bash
# 备用方式 - 使用固定的spawn位置
roslaunch tf_transform static_tf_publishers.launch
```

### 3. 直接运行Python脚本
```bash
python3 /root/workspace/tf_transform/mavros_pose_to_tf.py
```

## 验证命令

### 检查TF变换
```bash
# 检查无人机位姿
rosrun tf tf_echo map iris_0/base_link
rosrun tf tf_echo map standard_vtol_0/base_link

# 检查相机坐标系
rosrun tf tf_echo iris_0/base_link iris_0/camera_link
rosrun tf tf_echo iris_0/base_link iris_0/stereo_camera_frame
rosrun tf tf_echo standard_vtol_0/base_link standard_vtol_0/camera_link
```

### 检查相机图像
```bash
rostopic hz /iris_0/stereo_camera/left/image_raw
rostopic hz /iris_0/camera/image_raw
rostopic hz /standard_vtol_0/camera/image_raw
```

### 运行诊断脚本
```bash
cd /root/workspace/tf_transform
./check_camera_frames.sh
./camera_config_summary.sh
```

## 相机配置详情

### iris_0 相机
1. **RealSense 立体相机**
   - Frame ID: `iris_0/stereo_camera_frame`
   - 位置：机体前方0.1m
   - 坐标系：标准相机坐标系（x右，y下，z前）
   - 朝向：水平向前
   - 话题：`/iris_0/stereo_camera/left/image_raw`, `/iris_0/stereo_camera/right/image_raw`

2. **下视单目相机**
   - Frame ID: `iris_0/camera_link`
   - 位置：机体下方0.03m
   - 坐标系：标准相机坐标系（x右，y下，z朝地面）
   - 朝向：垂直向下
   - 话题：`/iris_0/camera/image_raw`

3. **深度相机**
   - 话题：`/iris_0/realsense/depth_camera/depth/image_raw`

### standard_vtol_0 相机
1. **下视单目相机**
   - Frame ID: `standard_vtol_0/camera_link`
   - 位置：机体下方0.03m
   - 坐标系：标准相机坐标系（x右，y下，z朝地面）
   - 朝向：垂直向下
   - 话题：`/standard_vtol_0/camera/image_raw`

## 技术特点

- **标准相机坐标系**：z轴指向相机正前方，符合计算机视觉约定
- **动态更新**：实时从MAVROS位姿话题获取无人机位置
- **多机支持**：同时处理两架无人机的坐标变换
- **自动发布**：相机坐标系自动跟随无人机移动
- **容错机制**：提供静态变换作为备用方案
- **易于调试**：包含完整的诊断和验证工具

## 依赖项

- ROS Noetic
- tf2_ros
- geometry_msgs
- MAVROS (用于位姿话题)

## 注意事项

1. 确保MAVROS正在运行并发布位姿话题
2. 相机坐标系使用标准的计算机视觉约定（x右，y下，z前）
3. 下视相机的z轴指向地面方向
4. 前视相机的z轴指向飞行方向
5. 如果动态TF有问题，可以使用静态TF作为备用

# TF Transform Package

这个包提供了将MAVROS位姿消息转换为TF变换的功能，支持多无人机系统。

## 功能特性

- 将iris_0和standard_vtol_0的MAVROS位姿转换为TF变换
- 发布相机坐标系变换（前视相机和下视相机）
- 支持多无人机协同仿真

## 启动文件

### 1. 基础启动文件
```bash
roslaunch tf_transform mavros_pose_to_tf.launch
```

### 2. 调试模式启动文件
```bash
roslaunch tf_transform mavros_pose_to_tf_debug.launch debug:=true
```

### 3. 多无人机系统启动文件
```bash
roslaunch tf_transform multi_uav_tf.launch
```

## 参数说明

- `debug`: 是否启用调试模式（默认: false）
- `iris_namespace`: iris无人机的命名空间（默认: iris_0）
- `vtol_namespace`: vtol无人机的命名空间（默认: standard_vtol_0）
- `output`: 节点输出模式（默认: screen）

## 发布的TF变换

### iris_0无人机
- `map` → `iris_0/base_link`
- `iris_0/base_link` → `iris_0/stereo_camera_frame` (前视相机)
- `iris_0/base_link` → `iris_0/camera_link` (下视相机)

### standard_vtol_0无人机
- `map` → `standard_vtol_0/base_link`
- `standard_vtol_0/base_link` → `standard_vtol_0/camera_link` (下视相机)

## 订阅的话题

- `/iris_0/mavros/local_position/pose`
- `/standard_vtol_0/mavros/local_position/pose`

## 使用示例

1. 启动PX4 SITL仿真
2. 启动MAVROS
3. 启动本包的TF转换节点：
```bash
roslaunch tf_transform mavros_pose_to_tf.launch
```

4. 查看TF树：
```bash
rosrun tf2_tools view_frames.py
evince frames.pdf
```

或使用调试模式：
```bash
roslaunch tf_transform mavros_pose_to_tf_debug.launch debug:=true
```
