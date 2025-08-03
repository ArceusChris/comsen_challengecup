# ROS Workspace for Drone Autonomous Search and Rescue Mission

这是一个用于无人机双机协同自主搜救任务的ROS工作空间。

## 工作空间结构

```
workspace/
├── build/                     # 编译输出目录
├── devel/                     # 开发环境设置文件
├── src/                       # 源代码目录
│   ├── CMakeLists.txt         # Catkin workspace 顶层CMakeLists
│   ├── iris_control/          # Iris旋翼无人机控制包
│   ├── landing_detection/     # 着陆检测包
│   ├── perception/            # 感知包（YOLO目标检测）
│   ├── precision_landing/     # 精确着陆包
│   ├── tf_transform/          # 坐标变换包
│   ├── vtol_control/          # VTOL垂起无人机控制包
│   └── workspace_utils/       # 工作空间工具包
└── README.md                  # 本文件
```

## 各包功能说明

### 1. iris_control
- **功能**: Iris旋翼无人机控制
- **主要脚本**:
  - `multirotor_control.py`: 旋翼无人机控制主程序
  - `drone_control.py`: 无人机控制接口
  - `landing.py`: 着陆控制
  - `test_visual_landing.py`: 视觉着陆测试

### 2. landing_detection
- **功能**: 着陆平台检测
- **主要脚本**:
  - `landing_detection.py`: 着陆检测主程序
  - `simple_landing_detector.py`: 简单着陆检测器
- **Launch文件**:
  - `landing_detection.launch`: 着陆检测启动文件

### 3. perception
- **功能**: 目标感知和检测
- **主要脚本**:
  - `yolo11_inference.py`: YOLO11推理脚本
  - `simple_test.py`: 简单测试
  - `test_yolo_functionality.py`: YOLO功能测试

### 4. precision_landing
- **功能**: 精确着陆控制
- **主要脚本**:
  - `precision_landing_controller.py`: 精确着陆控制器
  - `landing_state_machine.py`: 着陆状态机
  - `visual_servo_controller.py`: 视觉伺服控制器

### 5. tf_transform
- **功能**: 坐标系变换
- **主要脚本**:
  - `mavros_pose_to_tf.py`: MAVROS位姿到TF变换
- **Launch文件**:
  - `camera_tf_publishers.launch`: 相机TF发布器
  - `static_tf_publishers.launch`: 静态TF发布器

### 6. vtol_control
- **功能**: VTOL垂起无人机控制
- **主要脚本**:
  - `vtol_demo.py`: VTOL演示程序
  - `vtol_fly.py`: VTOL飞行控制
  - `vtol_ros.py`: VTOL ROS接口
  - `vtol_Astar.py`: A*路径规划

### 7. workspace_utils
- **功能**: 工作空间通用工具
- **主要脚本**:
  - `start.sh`: 系统启动脚本
  - `stop.sh`: 系统停止脚本
  - `get_person_positions.py`: 获取人员位置
  - `image_saver.py`: 图像保存工具
  - `virtual_rc.py`: 虚拟遥控器
- **数据集**:
  - `dataset/`: 数据集目录
  - `yolo_dataset/`: YOLO数据集
  - `yolo_train/`: YOLO训练数据

## 编译和使用

### 1. 编译工作空间
```bash
cd /root/workspace
catkin_make
```

### 2. 设置环境
```bash
source devel/setup.bash
```

### 3. 启动系统
```bash
# 使用工具包中的启动脚本
rosrun workspace_utils start.sh

# 或者直接运行
./src/workspace_utils/scripts/start.sh
```

### 4. 运行单个包
```bash
# 启动着陆检测
roslaunch landing_detection landing_detection.launch

# 运行YOLO检测
rosrun perception yolo11_inference.py

# 运行无人机控制
rosrun iris_control multirotor_control.py iris 0 vel
```

## 依赖项

### ROS包依赖
- rospy
- std_msgs
- geometry_msgs
- sensor_msgs
- mavros_msgs
- nav_msgs
- cv_bridge
- image_transport
- tf2_ros
- tf2_geometry_msgs

### Python依赖
- python3-opencv
- python3-numpy
- python3-torch (用于YOLO)

## 注意事项

1. 在使用前确保已正确安装PX4、Gazebo和相关依赖
2. 确保MAVROS已正确配置并连接到PX4
3. 运行前请先source工作空间的setup.bash文件
4. 大部分Python脚本需要可执行权限，编译后会自动设置

## 维护

- 添加新的ROS包时，请放在`src/`目录下
- 修改代码后需要重新运行`catkin_make`编译
- 添加新依赖时请同时更新相应包的`package.xml`文件
