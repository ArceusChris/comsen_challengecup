# 无人机双机协同自主搜救任务 - Docker环境

本项目为"挑战杯"无人机双机协同自主搜救任务比赛提供了Docker化的仿真环境。

## 环境要求

- Ubuntu 20.04 (推荐)
- Docker
- Docker Compose
- X11 (用于Gazebo GUI显示)

## 快速开始

### 1. 环境准备

首先运行环境准备脚本：

```bash
./setup.sh
```

这个脚本会：
- 安装Docker和Docker Compose（如果未安装）
- 创建必要的目录
- 设置X11转发权限

### 2. 构建Docker镜像

```bash
docker-compose build
```

### 3. 启动容器

```bash
docker-compose up -d
```

### 4. 进入容器

```bash
docker-compose exec drone_sim /bin/bash
```

### 5. 放置比赛文件

在运行仿真之前，您需要从比赛组织方获取以下文件并放置在正确位置：

1. **模型文件**：`standard_vtol`, `iris_zhihang`, `target_green`, `landing2`
   - 放置位置：`~/PX4_Firmware/Tools/sitl_gazebo/models/`

2. **人员模型文件**：`person_standing`
   - 放置位置：`~/.gazebo/models/`

3. **Launch文件**：`zhihang2025.launch`
   - 放置位置：`~/PX4_Firmware/launch/`

4. **World文件**：`zhihang2025.world`
   - 放置位置：`~/PX4_Firmware/Tools/sitl_gazebo/worlds/`

5. **位姿真值文件**：`get_local_pose.py`
   - 放置位置：`~/XTDrone/sensing/pose_ground_truth/`

6. **其他文件**：`zhihang2025文件夹`
   - 放置位置：`~/XTDrone/`

### 6. 启动仿真

在容器内运行以下命令序列：

#### 终端1：启动仿真程序
```bash
roslaunch px4 zhihang2025.launch
```

#### 终端2：垂起无人机通信
```bash
cd ~/XTDrone/communication/
python3 vtol_communication.py standard_vtol 0
```

#### 终端3：旋翼无人机通信
```bash
cd ~/XTDrone/communication/
python3 multirotor_communication.py iris 0
```

#### 终端4：垂起无人机位姿真值
```bash
cd ~/XTDrone/sensing/pose_ground_truth/
python3 get_local_pose.py standard_vtol 1
```

#### 终端5：旋翼无人机位姿真值
```bash
cd ~/XTDrone/sensing/pose_ground_truth/
python3 get_local_pose.py iris 1
```

#### 终端6：发布GPS引导点
```bash
cd ~/XTDrone/zhihang2025
python3 Pub_first_point.py
```

#### 终端7：发布居民区中心坐标
```bash
cd ~/XTDrone/zhihang2025
python3 Pub_downtown.py
```

#### 终端8：开启待救援目标移动
```bash
cd ~/XTDrone/zhihang2025
python3 zhihang_control_targets.py
```

#### 终端9：记录数据
```bash
cd ~/XTDrone/zhihang2025
rosbag record -O score1 /standard_vtol_0/mavros/state /iris_0/mavros/state /gazebo/model_states /xtdrone/standard_vtol_0/cmd /xtdrone/iris_0/cmd /zhihang/first_point /zhihang2025/first_man/pose /zhihang2025/second_man/pose /zhihang2025/third_man/pose /zhihang2025/iris_healthy_man/pose /zhihang2025/iris_bad_man/pose /zhihang/downtown
```

## 任务说明

### 阶段一：垂起无人机搜索任务
1. 垂起无人机从起飞平台起飞至20米高度
2. 切换为固定翼模式飞行
3. 避开两个禁飞区（居民区中心半径200m）
4. 飞行至GPS引导点
5. 搜索并识别3个模拟人员：
   - 危重人员（红色衣服）
   - 轻伤人员（黄色衣服）
   - 健康人员（白色迷彩）
6. 发布人员对应降落平台坐标
7. 返回旋翼模式区域降落

### 阶段二：四旋翼无人机救援任务
1. 等待垂起无人机降落后起飞
2. 飞往健康人员位置，在≤0.7m高度发布坐标
3. 飞往危重人员位置，在≤0.5m高度发布坐标
4. 返回旋翼模式区域降落

## 重要话题

- 危重人员：`/zhihang2025/first_man/pose`
- 轻伤人员：`/zhihang2025/second_man/pose`
- 健康人员：`/zhihang2025/third_man/pose`
- GPS引导点：`/zhihang/first_point`
- 居民区中心：`/zhihang/downtown`
- 阶段二健康人员：`/zhihang2025/iris_healthy_man/pose`
- 阶段二危重人员：`/zhihang2025/iris_bad_man/pose`

## 评分规则

- 作品符合性：20分
- 作品完整性：10分
- 系统得分：70分
  - 阶段一：30分
  - 阶段二：40分

## 注意事项

1. 必须按照指定步骤启动程序
2. 不能随意关闭或重新开启程序
3. 必须严格按照要求生成`score1.bag`文件
4. 禁飞区违规会导致任务失败
5. 垂起无人机必须在固定翼模式下执行搜索任务
6. 四旋翼无人机必须在指定高度发布坐标

## 故障排除

如果遇到问题，请检查：
1. 所有必需的比赛文件是否已正确放置
2. Docker容器是否有足够的权限
3. X11转发是否正常工作
4. 网络连接是否稳定