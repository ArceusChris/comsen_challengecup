# YOLO11推理节点Launch文件使用说明

## 文件说明

本目录包含以下launch文件：

### 1. `yolo11_inference.launch` - 主启动文件
这是主要的launch文件，支持完整的参数配置。

**基本用法：**
```bash
# 启动iris机型
roslaunch perception yolo11_inference.launch aircraft_type:=iris

# 启动standard_vtol机型  
roslaunch perception yolo11_inference.launch aircraft_type:=standard_vtol
```

**高级用法（自定义参数）：**
```bash
# 自定义模型和阈值
roslaunch perception yolo11_inference.launch \
  aircraft_type:=iris \
  model_path:=models/custom_yolo11.pt \
  confidence_threshold:=0.7 \
  iou_threshold:=0.5

# 禁用点云发布
roslaunch perception yolo11_inference.launch \
  aircraft_type:=standard_vtol \
  publish_pointcloud:=false

# 自定义话题
roslaunch perception yolo11_inference.launch \
  aircraft_type:=iris \
  input_topic:=/custom/camera/image_raw \
  output_topic:=/custom/yolo11/detection_image
```

### 2. `yolo11_iris.launch` - Iris机型快速启动
针对iris机型的简化启动文件。

```bash
roslaunch perception yolo11_iris.launch
```

### 3. `yolo11_vtol.launch` - VTOL机型快速启动
针对standard_vtol机型的简化启动文件。

```bash
roslaunch perception yolo11_vtol.launch
```

### 4. `yolo11_debug.launch` - 调试启动文件
包含调试功能的启动文件，支持RViz可视化和详细日志。

```bash
# 基本调试模式
roslaunch perception yolo11_debug.launch aircraft_type:=iris

# 启用完整调试模式（包括RViz和图像查看器）
roslaunch perception yolo11_debug.launch aircraft_type:=iris debug:=true

# 设置详细日志级别
roslaunch perception yolo11_debug.launch aircraft_type:=standard_vtol log_level:=debug
```

## 参数说明

### 机型参数
- `aircraft_type`: 机型选择
  - `iris`: 使用iris机型配置
  - `standard_vtol`: 使用standard_vtol机型配置

### 模型参数
- `model_path`: YOLO11模型文件路径（默认：`models/yolo11n_original.pt`）
- `confidence_threshold`: 检测置信度阈值（默认：0.6）
- `iou_threshold`: 非极大值抑制IOU阈值（默认：0.45）

### 话题参数
- `input_topic`: 输入图像话题（空值时自动根据机型配置）
- `output_topic`: 输出检测图像话题（空值时自动根据机型配置）
- `pose_topic`: 位姿话题（空值时自动根据机型配置）
- `camera_info_topic`: 相机内参话题（空值时自动根据机型配置）

### 点云参数
- `publish_pointcloud`: 是否发布点云数据（默认：true）
- `pointcloud_frame_id`: 点云坐标系ID（默认：map）
- `max_points_per_cloud`: 每个点云最大点数（默认：1000）

### 相机变换参数
- `camera_rotation_matrix`: 相机旋转矩阵（3x3）
- `camera_translation`: 相机平移向量[x, y, z]

## 默认话题配置

### Iris机型话题：
- 输入图像：`/iris_0/camera/image_raw`
- 位姿：`/iris_0/mavros/local_position/pose`
- 相机内参：`/iris_0/camera/camera_info`
- 检测结果：`/iris_0/yolo11/detection_image`

### Standard VTOL机型话题：
- 输入图像：`/standard_vtol_0/camera/image_raw`
- 位姿：`/standard_vtol_0/mavros/local_position/pose`
- 相机内参：`/standard_vtol_0/camera/camera_info`
- 检测结果：`/standard_vtol_0/yolo11/detection_image`
- VTOL标志：`/zhihang2025/vtol_land_sub/done`

## 输出话题

节点会发布以下话题：

### 实时位置话题：
- `/yolo11/position/red`
- `/yolo11/position/yellow`
- `/yolo11/position/white`

### 像素坐标话题：
- `/yolo11/pixel_position/red`
- `/yolo11/pixel_position/yellow`
- `/yolo11/pixel_position/white`

### 位姿估计话题（平均位置）：
- `/yolo11/pose_estimation/red`
- `/yolo11/pose_estimation/yellow`
- `/yolo11/pose_estimation/white`

### 点云话题（如果启用）：
- `/yolo11/pointcloud/red`
- `/yolo11/pointcloud/yellow`
- `/yolo11/pointcloud/white`

### VTOL目标位姿话题（仅standard_vtol机型）：
- `/zhihang2025/first_man/pose` (red目标)
- `/zhihang2025/second_man/pose` (yellow目标)
- `/zhihang2025/third_man/pose` (white目标)

## 故障排除

### 1. 模型文件不存在
确保模型文件路径正确，或者让节点使用默认的yolo11n.pt：
```bash
roslaunch perception yolo11_inference.launch aircraft_type:=iris model_path:=yolo11n.pt
```

### 2. TF变换错误
确保相关的TF发布者正在运行，特别是从相机坐标系到地图坐标系的变换。

### 3. 话题连接问题
检查话题是否存在：
```bash
rostopic list | grep camera
rostopic list | grep mavros
```

### 4. 性能问题
如果处理速度较慢，可以：
- 降低置信度阈值
- 减少点云最大点数
- 禁用点云发布

## 监控和调试

### 查看节点状态：
```bash
rosnode info /yolo11_inference/yolo11_inference_node
```

### 查看话题发布频率：
```bash
rostopic hz /iris_0/yolo11/detection_image
```

### 查看日志：
```bash
roslog | grep yolo11
```

### 使用RViz可视化：
```bash
rosrun rviz rviz
```
然后添加PointCloud2话题来查看检测到的点云数据。
