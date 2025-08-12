# Perception YOLOv12 包

这个 ROS 包提供了使用 YOLOv12 进行目标检测并计算目标位置的功能，是原有 perception 包（使用 YOLOv11）的升级版。

## 功能介绍

- 使用 YOLOv12 算法进行目标检测
- 支持 Iris 和 Standard VTOL 两种无人机平台
- 通过 ZeroMQ 与外部推理脚本通信，提高性能与稳定性
- 基于检测结果计算目标的 3D 位置
- 支持点云发布与位置估计

## 依赖项

- ROS Melodic/Noetic
- Python 3
- OpenCV
- NumPy
- Ultralytics (YOLOv12)
- ZeroMQ
- TensorFlow/PyTorch (取决于 YOLOv12 实现)

## 安装说明

1. 确保您已安装必要的 Python 依赖项：

```bash
pip3 install ultralytics opencv-python numpy pyzmq
```

2. 克隆此存储库到您的 catkin 工作空间的 src 目录：

```bash
cd ~/catkin_ws/src
git clone https://github.com/your_username/perception_yolov12.git
```

3. 编译工作空间：

```bash
cd ~/catkin_ws
catkin_make
```

4. 确保您已下载 YOLOv12 预训练模型，并将其放在适当的目录中。

## 使用方法

### 使用启动文件

选择适合您的无人机类型的启动文件：

- 对于 Iris 无人机：

```bash
roslaunch perception_yolov12 yolov12_iris.launch
```

- 对于 Standard VTOL 无人机：

```bash
roslaunch perception_yolov12 yolov12_vtol.launch
```

### 启动参数

可以通过启动文件传递以下参数：

- `model_path`：YOLOv12 模型路径，默认为 `yolov12n.pt`
- `device`：运行推理的设备，可选 `cpu` 或 `cuda`，默认为 `cpu`
- `conf_threshold`：置信度阈值，默认为 `0.45`
- `iou_threshold`：IOU 阈值，默认为 `0.45`

例如：

```bash
roslaunch perception_yolov12 yolov12_iris.launch model_path:=/path/to/your/model.pt device:=cuda conf_threshold:=0.5 iou_threshold:=0.4
```

### 手动启动节点

您也可以单独启动桥接节点和推理脚本：

1. 首先启动桥接节点：

```bash
rosrun perception_yolov12 yolo12_bridge_node.py _aircraft_type:=iris
```

2. 然后在另一个终端中启动推理脚本：

```bash
rosrun perception_yolov12 yolo12_inference.py --model yolov12n.pt --device cpu --conf 0.45 --iou 0.45
```

## 多无人机设置

如果需要同时运行多个无人机的目标检测（例如同时使用 Iris 和 Standard VTOL），您可以使用多无人机启动文件：

```bash
roslaunch perception_yolov12 multi_uav_yolov12.launch
```

该启动文件会为每个无人机启动单独的桥接节点和推理脚本，使用不同的端口进行通信：
- Iris: 发送端口=5555，接收端口=5556
- VTOL: 发送端口=5557，接收端口=5558

### 自定义端口

如果需要使用自定义端口，可以在启动节点时指定：

```bash
# 桥接节点
rosrun perception_yolov12 yolo12_bridge_node.py _sender_port:=5559 _receiver_port:=5560

# 推理脚本
rosrun perception_yolov12 yolo12_inference.py --sender-port 5559 --receiver-port 5560
```

注意：确保桥接节点的 `sender_port` 与推理脚本的 `sender-port` 相匹配，同理 `receiver_port` 也需要与 `receiver-port` 相匹配。

## 话题说明

根据所选的无人机类型，节点将订阅和发布以下话题：

### 输入话题

- `/iris_0/camera/image_raw` 或 `/standard_vtol_0/camera/image_raw`：相机图像
- `/iris_0/mavros/local_position/pose` 或 `/standard_vtol_0/mavros/local_position/pose`：无人机位置
- `/iris_0/camera/camera_info` 或 `/standard_vtol_0/camera/camera_info`：相机参数
- `/zhihang2025/vtol_land_sub/done`：VTOL 着陆标志（仅用于 VTOL）

### 输出话题

- `/iris_0/yolo12/annotated_image` 或 `/standard_vtol_0/yolo12/annotated_image`：标注后的图像
- `/iris_0/yolo12/detections` 或 `/standard_vtol_0/yolo12/detections`：检测结果（JSON 字符串）
- `/yolo12/position/{red|yellow|white}`：实时目标位置
- `/yolo12/pose_estimation/{red|yellow|white}`：目标位置估计（平均位置）
- `/yolo12/pixel_position/{red|yellow|white}`：像素位置
- `/zhihang2025/{first|second|third}_man/pose`：VTOL 目标位姿（仅用于 VTOL）
- `/yolo12/pointcloud/{red|yellow|white}`：目标点云（可选）

## 许可证

MIT 许可证
