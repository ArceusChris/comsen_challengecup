# YOLO11 无人机目标检测与位置估计系统

本系统基于YOLO11实现无人机视觉目标检测，并通过相机内参和无人机位姿信息计算目标物体在世界坐标系中的位置。

## 功能特性

- 基于YOLO11的实时目标检测
- 支持三种目标类型：red, yellow, white
- 通过相机内参和无人机位姿计算物体世界坐标
- 自动维护每种目标的历史位置点集
- 可视化检测结果和统计信息
- 支持数据导出和分析

## 文件说明

### 核心文件
- `yolo11_inference.py` - YOLO11推理节点主程序
- `launch_yolo_inference.py` - 节点启动脚本
- `point_set_viewer.py` - 点集数据查看和分析工具
- `camera_transform_config.py` - 相机变换参数配置工具
- `test_downward_camera.py` - 正下方相机配置验证工具

### 依赖要求
```bash
# ROS依赖
sensor_msgs
geometry_msgs
cv_bridge
tf2_ros
tf2_geometry_msgs

# Python依赖
ultralytics
opencv-python
numpy
```

## 使用方法

### 1. 启动仿真环境
首先启动PX4 SITL仿真和Gazebo：
```bash
cd /root
./start_px4_gazebo.sh
```

### 2. 启动QGroundControl（可选）
```bash
cd /root
./QGroundControl.AppImage
```

### 3. 配置相机变换参数（可选）
当前系统已为正下方相机进行了优化配置。如需其他配置：
```bash
cd /root/workspace/perception
python3 camera_transform_config.py
```

### 4. 设置节点参数
```bash
cd /root/workspace/perception
python3 launch_yolo_inference.py
```

### 5. 启动YOLO推理节点
在新终端中：
```bash
cd /root/workspace/perception
python3 yolo11_inference.py
```

### 6. 查看检测结果
可以通过以下话题查看结果：
```bash
# 查看检测后的图像
rostopic echo /yolo11/detection_image

# 查看原始图像
rostopic echo /iris_0/camera/image_raw

# 查看无人机位姿
rostopic echo /iris_0/mavros/local_position/pose

# 查看相机内参
rostopic echo /iris_0/camera/camera_info
```

### 7. 数据分析
使用点集查看器分析收集的位置数据：
```bash
cd /root/workspace/perception
python3 point_set_viewer.py
```

### 8. 验证相机配置（推荐）
验证正下方相机配置是否正确：
```bash
cd /root/workspace/perception
python3 test_downward_camera.py
```
该工具会测试坐标转换的准确性，确保配置正确。

## 节点参数配置

可以通过ROS参数服务器配置节点行为：

```yaml
yolo11_inference_node:
  input_topic: "/iris_0/camera/image_raw"           # 输入图像话题
  output_topic: "/yolo11/detection_image"          # 输出图像话题
  pose_topic: "/iris_0/mavros/local_position/pose" # 无人机位姿话题
  camera_info_topic: "/iris_0/camera/camera_info"  # 相机内参话题
  model_path: "yolo11n.pt"                         # YOLO模型路径
  confidence_threshold: 0.5                        # 置信度阈值
  iou_threshold: 0.45                              # IOU阈值
  
  # 相机变换参数（相机朝向正下方配置）
  camera_rotation_matrix: [                        # 3x3旋转矩阵（简化配置）
    [1, 0, 0],    # 相机X轴 -> 无人机X轴（前向）
    [0, 1, 0],    # 相机Y轴 -> 无人机Y轴（右向）  
    [0, 0, -1]    # 相机Z轴 -> 无人机-Z轴（向下）
  ]
  camera_translation: [0.0, 0.0, -0.03]           # 相机位于无人机正下方0.03m处
```

## 相机变换参数配置

相机变换参数配置工具提供了多种预设配置：

1. **downward_camera**: 相机朝向正下方（推荐配置 - 无人机下方0.03m）
2. **forward_down**: 相机朝前向下（常用于地面目标检测）
3. **forward_level**: 相机朝前水平（用于前方目标检测）
4. **down_only**: 相机正下方（云台相机）

当前系统已针对正下方相机配置进行了优化，变换关系简化：
- X、Y轴保持不变（相机与无人机坐标系对齐）
- Z轴翻转（相机向下看）
- 简单的垂直位移（-0.03m）

使用配置工具：
```bash
cd /root/workspace/perception
python3 camera_transform_config.py
```

工具功能：
- 查看和选择预设配置
- 创建自定义变换参数
- 从欧拉角或旋转矩阵生成配置
- 自动生成ROS参数文件

## 坐标系转换说明

系统假设：
1. 目标物体都位于地面上（Z=0）
2. 相机朝向无人机正下方，位于无人机下方0.03m处
3. 相机坐标系遵循ROS标准（Z轴朝前，X轴朝右，Y轴朝下）
4. 使用简化的坐标变换（相机坐标系与无人机坐标系基本对齐）

转换流程：
1. 检测框中心点 → 像素坐标
2. 像素坐标 → 归一化相机坐标
3. 相机坐标系 → 无人机坐标系（简单的Z轴翻转和位移）
4. 无人机坐标系 → 世界坐标系（通过无人机位姿）
5. 计算射线与地面交点得到物体世界坐标

相机配置参数：
- 旋转矩阵：单位矩阵（Z轴翻转）
- 平移向量：[0.0, 0.0, -0.03] （正下方0.03m）

## 数据格式

点集数据结构：
```json
{
  "red": [
    {
      "position": [x, y, z],
      "timestamp": 1234567890.123,
      "pixel_coords": [pixel_x, pixel_y]
    }
  ],
  "yellow": [...],
  "white": [...]
}
```

## 故障排除

### 常见问题

1. **模型文件不存在**
   - 确保YOLO模型文件路径正确
   - 如果使用自定义模型，请确保模型支持red, yellow, white类别

2. **缺少位姿或相机内参信息**
   - 检查仿真环境是否正常启动
   - 确认话题名称是否正确
   - 使用`rostopic list`查看可用话题

3. **检测精度问题**
   - 调整置信度阈值和IOU阈值
   - 检查目标物体是否在相机视野内
   - 确认光照条件是否适合检测

4. **坐标转换异常**
   - 检查相机内参是否有效
   - 确认无人机位姿数据是否正常
   - 验证相机变换参数是否正确
   - 使用相机配置工具调整变换矩阵
   - 验证目标是否在地面上

### 调试命令

```bash
# 检查话题数据
rostopic echo /iris_0/mavros/local_position/pose
rostopic echo /iris_0/camera/camera_info

# 查看节点日志
rosnode info /yolo11_inference_node

# 检查图像数据
rqt_image_view  # 图形界面查看图像
```

## 扩展功能

可以考虑添加的功能：
1. 支持更多目标类型
2. 3D目标检测和位置估计
3. 目标跟踪和轨迹预测
4. 实时数据可视化界面
5. 数据库存储和历史查询
6. 多无人机协同检测

## 开发说明

如需修改或扩展功能，请注意：
1. 保持线程安全（使用data_lock）
2. 处理异常情况和边界条件
3. 添加适当的日志输出
4. 保持代码可读性和文档完整性
