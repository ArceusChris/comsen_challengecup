# YOLO11推理节点 - 双模式目标检测

本节点支持两种目标检测方法：YOLO深度学习检测和基于颜色阈值的传统检测。

## 功能特性

- **双检测模式**：支持YOLO模型检测和HSV颜色阈值检测
- **目标跟踪**：检测红色、黄色、白色三种目标
- **3D定位**：将2D检测结果转换为3D世界坐标
- **点云发布**：为每种目标类型发布点云数据
- **实时切换**：可在运行时配置检测方法

## 安装依赖

```bash
# ROS依赖已通过系统安装
# Python依赖
pip3 install ultralytics opencv-python numpy
```

## 使用方法

### 1. YOLO检测模式

```bash
# 启动YOLO检测
rosrun perception yolo11_inference.py _detection_method:=yolo

# 或使用launch文件
roslaunch perception yolo_detection.launch
```

### 2. 颜色阈值检测模式

```bash
# 启动颜色检测
rosrun perception yolo11_inference.py _detection_method:=color

# 或使用launch文件中的颜色检测节点
roslaunch perception yolo_color_detection.launch
```

### 3. 参数配置

#### YOLO模式参数：
- `model_path`: YOLO模型文件路径
- `confidence_threshold`: 置信度阈值 (默认: 0.5)
- `iou_threshold`: IOU阈值 (默认: 0.45)

#### 颜色检测模式参数：
- `min_contour_area`: 最小轮廓面积 (默认: 500)
- `morphology_kernel_size`: 形态学操作核大小 (默认: 5)
- `gaussian_blur_size`: 高斯模糊核大小 (默认: 5)

### 4. 颜色阈值调整

编辑配置文件 `config/color_detection_config.yaml` 或在launch文件中设置参数：

```yaml
# HSV颜色范围示例
color_ranges:
  red:
    lower1: [0, 50, 50]      # 红色范围1
    upper1: [10, 255, 255]
    lower2: [170, 50, 50]    # 红色范围2
    upper2: [180, 255, 255]
  yellow:
    lower: [20, 50, 50]      # 黄色范围
    upper: [30, 255, 255]
  white:
    lower: [0, 0, 200]       # 白色范围
    upper: [180, 30, 255]
```

## 话题接口

### 订阅话题：
- `/camera/image_raw` (sensor_msgs/Image): 输入图像
- `/iris_0/mavros/local_position/pose` (geometry_msgs/PoseStamped): 无人机位姿
- `/iris_0/camera/camera_info` (sensor_msgs/CameraInfo): 相机内参

### 发布话题：
- `/yolo11/detection_image` (sensor_msgs/Image): 标注后的检测图像
- `/yolo11/pointcloud/red` (sensor_msgs/PointCloud2): 红色目标点云
- `/yolo11/pointcloud/yellow` (sensor_msgs/PointCloud2): 黄色目标点云
- `/yolo11/pointcloud/white` (sensor_msgs/PointCloud2): 白色目标点云

## 检测方法对比

| 特性 | YOLO检测 | 颜色阈值检测 |
|------|----------|-------------|
| 精度 | 高 | 中等 |
| 速度 | 中等 | 快 |
| 环境适应性 | 强 | 较弱 |
| 光照敏感性 | 低 | 高 |
| 计算资源需求 | 高 | 低 |
| 配置复杂度 | 低 | 中等 |

## 调试和优化

### 颜色检测调试：
1. 使用 `rqt_image_view` 查看检测结果
2. 调整HSV阈值范围
3. 修改形态学操作参数
4. 调整最小轮廓面积

### YOLO检测调试：
1. 检查模型文件路径
2. 调整置信度和IOU阈值
3. 确保目标在训练类别中

## 故障排除

1. **检测不到目标**：
   - 颜色模式：调整HSV阈值范围
   - YOLO模式：降低置信度阈值

2. **误检过多**：
   - 颜色模式：缩小HSV范围，增大最小轮廓面积
   - YOLO模式：提高置信度阈值

3. **点云数据异常**：
   - 检查相机内参是否正确
   - 确认无人机位姿数据正常
   - 验证相机-无人机变换参数

## 示例用法

```bash
# 启动仿真环境
roslaunch gazebo_simulation drone_simulation.launch

# 启动相机节点
roslaunch camera_driver camera.launch

# 启动检测节点（颜色检测模式）
rosrun perception yolo11_inference.py _detection_method:=color _min_contour_area:=300

# 查看检测结果
rqt_image_view /yolo11/detection_image

# 查看点云数据
rviz # 添加PointCloud2显示，话题选择/yolo11/pointcloud/red等
```
