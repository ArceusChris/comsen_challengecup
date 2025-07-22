# YOLO11 ROS1 推理节点

这个包提供了一个ROS1节点，用于订阅图像话题，运行YOLO11目标检测推理，并发布带有检测框的标注图像。

## 功能特性

- 从ROS话题订阅图像
- 使用YOLO11模型进行实时目标检测
- 在图像上绘制检测框、标签和置信度
- 发布标注后的图像到新的ROS话题
- 支持自定义模型路径和推理参数
- 多线程处理，确保实时性能

## 依赖安装

### 1. ROS1依赖
确保已安装以下ROS包：
```bash
sudo apt-get install ros-noetic-cv-bridge ros-noetic-image-view
```

### 2. Python依赖
```bash
cd /home/xylin/comsen_challengecup/workspace/perception
pip install -r requirements.txt
```

### 3. 赋予执行权限
```bash
chmod +x yolo11_inference.py
```

## 使用方法

### 方法1：使用launch文件（推荐）
```bash
# 启动推理节点和图像查看器
roslaunch perception yolo11_inference.launch

# 只启动推理节点
roslaunch perception yolo11_inference.launch detection_viewer:=false
```

### 方法2：直接运行节点
```bash
# 使用默认参数
rosrun perception yolo11_inference.py

# 使用自定义参数
rosrun perception yolo11_inference.py _input_topic:=/your_camera/image_raw _model_path:=/path/to/your/model.pt
```

## 参数配置

| 参数名 | 类型 | 默认值 | 描述 |
|--------|------|--------|------|
| `input_topic` | string | `/camera/image_raw` | 输入图像话题名称 |
| `output_topic` | string | `/yolo11/detection_image` | 输出图像话题名称 |
| `model_path` | string | `yolo11n.pt` | YOLO11模型文件路径 |
| `confidence_threshold` | float | `0.5` | 检测置信度阈值 |
| `iou_threshold` | float | `0.45` | 非极大值抑制IoU阈值 |

## 话题说明

### 订阅话题
- `/camera/image_raw` (sensor_msgs/Image): 输入的原始图像

### 发布话题
- `/yolo11/detection_image` (sensor_msgs/Image): 带有检测框的标注图像

## 模型使用

### 默认模型
如果不指定模型路径，节点会自动下载并使用YOLOv11n模型。

### 自定义模型
1. 将训练好的YOLO11模型文件（.pt格式）放在合适的位置
2. 在launch文件中修改`model_path`参数，或通过命令行参数指定

### 支持的模型格式
- PyTorch (.pt)
- ONNX (.onnx)
- TensorRT (.engine)

## 性能优化建议

1. **GPU加速**: 确保安装了CUDA和PyTorch GPU版本
2. **模型优化**: 使用较小的模型（如yolo11n.pt）以提高速度
3. **图像尺寸**: 适当调整输入图像分辨率
4. **多线程**: 节点内部已使用异步处理

## 故障排除

### 常见问题

1. **模型下载失败**
   ```bash
   # 手动下载模型
   wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolo11n.pt
   ```

2. **依赖包缺失**
   ```bash
   pip install --upgrade ultralytics opencv-python
   ```

3. **CUDA相关错误**
   ```bash
   # 检查CUDA安装
   nvidia-smi
   python -c "import torch; print(torch.cuda.is_available())"
   ```

4. **话题连接问题**
   ```bash
   # 检查可用话题
   rostopic list
   # 检查话题信息
   rostopic info /camera/image_raw
   ```

## 示例使用场景

### 1. 与摄像头配合使用
```bash
# 启动USB摄像头
roslaunch usb_cam usb_cam-test.launch
# 启动YOLO11推理
roslaunch perception yolo11_inference.launch input_topic:=/usb_cam/image_raw
```

### 2. 与bag文件配合使用
```bash
# 播放bag文件
rosbag play your_data.bag
# 启动推理节点
roslaunch perception yolo11_inference.launch
```

### 3. 自定义检测类别
如果使用自训练模型，模型的类别名称会自动从模型文件中读取。

## 许可证

本项目遵循相应的开源许可证。请确保遵守YOLO11和相关依赖包的许可证要求。
