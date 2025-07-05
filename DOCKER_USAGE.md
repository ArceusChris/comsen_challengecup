# 无人机仿真环境使用说明

## 启动方式

### 方式一：使用 Docker Compose（推荐）

#### 1. 完整启动脚本

```bash
# 给脚本执行权限
chmod +x docker_start.sh

# 运行启动脚本
./docker_start.sh
```

启动脚本提供以下选项：

- **选项1**：完整GUI模式（需要X11支持）
- **选项2**：无头模式（适合开发调试）
- **选项3**：仅构建不启动
- **选项4**：清理并重新构建
- **选项5**：停止并清理容器

#### 2. 快速启动

```bash
# 给脚本执行权限
chmod +x quick_start.sh

# 快速启动（GUI模式）
./quick_start.sh
```

#### 3. 手动启动

```bash
# 构建镜像
docker-compose build

# 启动容器（GUI模式）
docker-compose up -d

# 进入容器
docker-compose exec drone_sim /bin/bash
```

### 方式二：直接使用 Docker 命令

#### GUI模式

```bash
# 构建镜像
docker build -t drone-sim .

# 启动容器
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v $(pwd)/workspace:/root/workspace \
  -v $(pwd)/models:/root/.gazebo/models \
  --device /dev/dri:/dev/dri \
  --network host \
  --privileged \
  drone-sim
```

#### 无头模式

```bash
docker run -it --rm \
  -v $(pwd)/workspace:/root/workspace \
  -v $(pwd)/models:/root/.gazebo/models \
  --network host \
  drone-sim
```

## Docker内部设置

### 1.准备模型文件

首先启动docker
将model.zip解压以后在宿主机运行如下命令

```bash
docker cp /path/to/models drone_challenge_sim:/root/.gazebo
```

### 2.安装地面站

下载QGroundControl.AppImage，在宿主机运行

```bash
docker cp /path/to/QGroundControl.AppImage drone_challenge_sim:/root/.gazebo
```

docker内运行

```bash
sudo apt update && sudo apt install -y fuse libfuse2
cd ~/
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage
```

出现QGC窗口即为配置成功

### 3.编译gazebo

```bash
cd ~/PX4_Firmware
make clean
make px4_sitl_default gazebo
```

出现一架无人机即为配置成功

## 仿真环境验证

```bash
cd ~/PX4_Firmware
roslaunch px4 indoor1.launch
```

再开一个终端打开通信程序：

```bash
cd ~/XTDrone/communication/
python3 multirotor_communication.py iris 0
```

再开一个终端运行：

```bash
cd ~/XTDrone/control/keyboard
python3 multirotor_keyboard_control.py iris 1 vel
```

如果室内场景能够成功加载、四旋翼可以通过键盘控制起飞，证明环境已经成功配置完成。

## 容器内仿真启动

进入容器后，可以使用以下脚本启动仿真：

### 1. 启动PX4+Gazebo仿真

```bash
# GUI模式
./start_px4_gazebo.sh

# 无头模式
./start_px4_headless.sh
```

### 2. 手动启动
```bash
# 进入PX4目录
cd /root/PX4_Firmware

# 设置环境变量
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
source Tools/setup_gazebo.bash . build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:.
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:./Tools/sitl_gazebo

# 启动仿真
make px4_sitl_default gazebo  # GUI模式
HEADLESS=1 make px4_sitl_default gazebo  # 无头模式
```

## 比赛特定启动流程

如果有比赛文件，请按照以下步骤：

### 1. 启动仿真程序
```bash
roslaunch px4 zhihang2025.launch
```

### 2. 在新终端中运行通信脚本
```bash
# 垂起无人机
cd ~/XTDrone/communication/
python3 vtol_communication.py standard_vtol 0

# 旋翼无人机
cd ~/XTDrone/communication/
python3 multirotor_communication.py iris 0
```

### 3. 开启gazebo位姿真值
```bash
# 垂起无人机
cd ~/XTDrone/sensing/pose_ground_truth/
python3 get_local_pose.py standard_vtol 1

# 旋翼无人机
cd ~/XTDrone/sensing/pose_ground_truth/
python3 get_local_pose.py iris 1
```

### 4. 发布GPS引导点
```bash
cd ~/XTDrone/zhihang2025
python3 Pub_first_point.py
```

### 5. 发布居民区中心坐标
```bash
cd ~/XTDrone/zhihang2025
python3 Pub_downtown.py
```

### 6. 开启待救援目标移动
```bash
cd ~/XTDrone/zhihang2025
python3 zhihang_control_targets.py
```

### 7. 记录数据
```bash
cd ~/XTDrone/zhihang2025
rosbag record -O score1 /standard_vtol_0/mavros/state /iris_0/mavros/state /gazebo/model_states /xtdrone/standard_vtol_0/cmd /xtdrone/iris_0/cmd /zhihang/first_point /zhihang2025/first_man/pose /zhihang2025/second_man/pose /zhihang2025/third_man/pose /zhihang2025/iris_healthy_man/pose /zhihang2025/iris_bad_man/pose /zhihang/downtown
```

## 常见问题

### Q1: 无法显示Gazebo界面
A1: 请确保：
- 已设置DISPLAY环境变量
- 已允许X11连接：`xhost +local:docker`
- 使用GUI模式启动

### Q2: 容器无法启动
A2: 请检查：
- Docker是否正在运行
- 是否有足够的磁盘空间
- 是否有权限访问Docker

### Q3: 仿真运行缓慢
A3: 建议：
- 使用无头模式减少GPU负载
- 关闭不必要的应用程序
- 检查系统资源占用

### Q4: 比赛文件缺失
A4: 请从比赛组织方获取以下文件：
- 模型文件：standard_vtol, iris_zhihang, target_green, landing2
- 人员模型：person_standing
- Launch文件：zhihang2025.launch
- World文件：zhihang2025.world
- 位姿真值文件：get_local_pose.py
- 其他文件：zhihang2025文件夹

## 文件结构

```
.
├── docker-compose.yml      # Docker Compose配置
├── Dockerfile             # Docker镜像构建文件
├── docker_start.sh        # Docker启动脚本
├── quick_start.sh         # 快速启动脚本
├── start_simulation.sh    # 仿真启动脚本（容器内）
├── workspace/             # 工作空间目录
├── models/                # 模型文件目录
└── DOCKER_USAGE.md       # 本说明文档
```

## 停止和清理

### 停止容器
```bash
docker-compose down
```

### 清理镜像
```bash
docker-compose down --rmi all --volumes
```

### 清理所有Docker资源
```bash
docker system prune -a
```
