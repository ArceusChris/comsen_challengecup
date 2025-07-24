# RViz 图像可视化指南

本文档介绍如何在RViz中查看无人机相机和YOLO检测结果的图像。

## 1. 启动RViz

```bash
# 确保ROS环境已sourced
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash  # 如果有自定义工作空间

# 启动RViz
rviz
```

## 2. 查看原始相机图像

### 2.1 添加Image显示组件

1. 在RViz左侧面板点击 **"Add"** 按钮
2. 在弹出的对话框中选择 **"Image"**
3. 点击 **"OK"**

### 2.2 配置图像话题

1. 在左侧面板找到新添加的 **"Image"** 组件
2. 展开其属性设置
3. 在 **"Image Topic"** 下拉菜单中选择: `/iris_0/camera/image_raw`
4. 图像应该开始显示在RViz的显示窗口中

## 3. 查看YOLO检测结果

### 3.1 检查YOLO话题

首先确认YOLO节点正在发布检测结果：

```bash
# 查看所有图像话题
rostopic list | grep image

# 检查YOLO检测结果话题（可能的话题名称）
rostopic echo /yolo11/detection_image
# 或者
rostopic echo /yolo_detection/image
# 或者
rostopic echo /detection_result/image
```

### 3.2 在RViz中显示YOLO结果

1. 重复步骤2.1添加另一个Image组件
2. 在 **"Image Topic"** 中选择YOLO检测结果话题，例如：
   - `/yolo11/detection_image`
   - `/yolo_detection/image`
   - `/detection_result/image`

## 4. 替代方案：使用rqt_image_view

如果RViz显示有问题，可以使用专门的图像查看工具：

```bash
# 查看原始相机图像
rqt_image_view /iris_0/camera/image_raw

# 查看YOLO检测结果
rqt_image_view /yolo11/detection_image
```

## 5. 多窗口显示

可以同时打开多个rqt_image_view窗口来比较原始图像和检测结果：

```bash
# 终端1：显示原始图像
rqt_image_view /iris_0/camera/image_raw &

# 终端2：显示检测结果
rqt_image_view /yolo11/detection_image &
```

## 6. 保存RViz配置

为了方便下次使用，可以保存RViz配置：

1. 配置好所有显示组件后
2. 点击菜单 **File → Save Config As**
3. 保存为 `drone_vision.rviz`
4. 下次启动时使用：
   ```bash
   rviz -d drone_vision.rviz
   ```

## 7. 故障排查

### 7.1 图像不显示

```bash
# 检查话题是否存在
rostopic list | grep camera

# 检查话题是否有数据
rostopic hz /iris_0/camera/image_raw

# 检查图像消息内容
rostopic echo /iris_0/camera/image_raw --noarr
```

### 7.2 YOLO检测结果不显示

```bash
# 检查YOLO节点是否运行
rosnode list | grep yolo

# 检查YOLO相关话题
rostopic list | grep yolo
rostopic list | grep detection

# 检查YOLO节点日志
rosnode info /yolo11_node  # 替换为实际节点名称
```

### 7.3 性能优化

如果图像显示卡顿：

1. 在RViz Image组件中降低 **"Queue Size"**
2. 调整 **"Unreliable"** 选项
3. 减少图像分辨率（如果可能）

## 8. 命令行快速启动脚本

创建一个快速启动脚本来同时查看多个图像流：

```bash
#!/bin/bash
# view_images.sh

echo "启动图像查看器..."

# 启动RViz（后台）
rviz -d drone_vision.rviz &

# 等待一秒
sleep 1

# 启动原始图像查看器
rqt_image_view /iris_0/camera/image_raw &

# 启动检测结果查看器
rqt_image_view /yolo11/detection_image &

echo "图像查看器已启动"
echo "按Ctrl+C停止所有查看器"

# 等待用户中断
wait
```

使用方法：
```bash
chmod +x view_images.sh
./view_images.sh
```

## 9. 高级技巧

### 9.1 图像话题重映射

如果话题名称不匹配，可以使用重映射：

```bash
# 重映射话题名称
rosrun image_view image_view image:=/iris_0/camera/image_raw
```

### 9.2 图像录制

录制图像话题到bag文件：

```bash
# 录制相机图像
rosbag record /iris_0/camera/image_raw -O camera_data.bag

# 录制所有图像话题
rosbag record -a -O all_data.bag
```

### 9.3 图像格式转换

如果图像格式不兼容：

```bash
# 使用image_transport转换
rosrun image_transport republish compressed in:=/iris_0/camera/image_raw raw out:=/camera/image_converted
```
