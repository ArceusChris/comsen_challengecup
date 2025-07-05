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

## 联系方式

如有问题，请联系比赛组织方邮箱：18210263093@163.com

**关键绩效指标：**
- 阶段一坐标误差 ≤ 10米（30分）
- 阶段二坐标误差 ≤ 3米（40分）
- 通信延迟 < 200ms

### 2. 控制与仿真组（2人,后续视情况1人转算法）
**核心职责：**
- 无人机控制逻辑实现（模式切换、精准降落）
- Gazebo仿真环境配置与调试
- 禁飞区避障实现（200m半径圆形区域）
- 真值数据获取与验证（运行 `get_local_pose.py`）
- 日志录制管理（确保 `score1.bag` 完整）

**关键绩效指标：**
- 100% 避免禁飞区违规
- 垂起机降落精度误差 ≤ 1米
- 仿真环境启动成功率 100%

### 3. 系统集成与测试组（1人）
**核心职责：**
- ROS系统集成与节点调度
- 全流程测试与计时优化（T1计时）
- 异常场景测试与修复（坠毁、通信失败）
- 性能指标监控与报告

**关键绩效指标：**
- 任务完成时间 T1 最小化
- 全流程测试覆盖率 ≥ 90%
- BAG文件完整率 100%

### 4. 文档与材料组（1人）
**核心职责：**
- 技术文档编写（设计思路、技术路线）
- 创新点提炼与表达
- 录屏视频制作与编辑
- 代码注释规范与部署文档
- 最终材料打包提交

**关键绩效指标：**
- 文档评分 ≥ 18/20
- 代码可复现性 100%
- 材料提交零差错

---

## 二、各组操作手册

### 算法开发组操作手册

#### 算法开发组工作流程

##### 开发环境配置
```bash
git clone https://your-repo/drone-rescue.git
cd drone-rescue
git checkout -b feat/perception  # 创建功能分支
conda create -n rescue python=3.8
conda activate rescue
pip install -r requirements.txt
```

##### 每日工作流程
1. **同步最新代码**：
   ```bash
   git fetch origin
   git rebase origin/dev
   ```

2. **算法开发**：
   - 目标识别模块：`src/perception/target_detection.py`
   - 路径规划模块：`src/planning/path_planner.py`
   - 通信模块：`src/communication/message_broker.py`

3. **本地测试**：
   ```bash
   # 启动简化仿真环境
   roslaunch px4 simplified.launch
   # 运行算法测试
   pytest tests/perception/test_detection.py
   ```

4. **提交代码**：
   ```bash
   git add src/perception/target_detection.py
   git commit -m "[perception][feat] 优化红色目标识别精度"
   git push origin feat/perception
   ```

#### 关键注意事项
⚠️ **目标识别要求**：
- 红色目标（危重）：置信度 > 0.9
- 黄色目标（轻伤）：IOU阈值 > 0.7,需处理动态位置变化
- 白色目标（健康）：需处理动态位置变化

⚠️ **路径规划约束**：
```python
# 禁飞区参数（config/drone_params/no_fly_zones.yaml）
zone1: 
  center: [x1, y1]  # 居民区1中心
  radius: 200        # 单位：米
zone2:
  center: [x2, y2]  # 居民区2中心
  radius: 200
```

#### 协作要求
- 每日19:00前提交PR至dev分支
- 与测试组确认接口规范：
  ```yaml
  # 目标坐标话题规范
  topic: /zhihang2025/first_man/pose
  msg_type: geometry_msgs/Pose
  fields:
    position.x: float
    position.y: float
  ```


### 控制与仿真组操作手册


#### 控制与仿真组工作流程

##### 仿真环境配置
```bash
# 恢复标准仿真环境
cd ~/PX4_Firmware
git checkout -- Tools/sitl_gazebo/models/standard_vtol
git checkout -- launch/zhihang2025.launch
```

##### 每日工作流程
1. **启动完整仿真**：
   ```bash
   roslaunch px4 zhihang2025.launch
   ```

2. **控制逻辑测试**：
   ```python
   # 模式切换测试（control/mode_switcher.py）
   def switch_to_fixed_wing(height=20):
       if current_altitude >= height:
           set_mode("FW_MODE")
   ```

3. **避障算法验证**：
   ```bash
   # 实时监控禁飞区距离
   rostopic echo /standard_vtol_0/distance_to_zone
   ```

4. **日志录制检查**：
   ```bash
   rosbag info score1.bag | grep -E "topic_count|duration"
   ```

##### 关键控制参数
| 参数 | 值 | 说明 |
|------|----|------|
| `FW_L1_PERIOD` | 12s | 固定翼转弯半径控制 |
| `MPC_Z_VEL_MAX_DN` | 1.0m/s | 最大下降速度 |
| `NAV_RCL_ACT` | 0 | 禁飞区规避模式 |

##### 紧急情况处理
**场景：无人机进入禁飞区**
1. 立即停止当前测试
2. 记录进入前的最后有效位置
3. 检查控制逻辑中的安全边界：
   ```python
   safety_margin = 15  # 增加15米安全边界
   if distance < (radius + safety_margin):
       trigger_avoidance()
   ```

#### 协作要求
- 每日提供仿真环境状态报告
- 算法更新后立即进行避障测试
- 发现控制问题创建issue并标记`urgent`


### 系统集成与测试组操作手册

#### 系统集成与测试组工作流程

##### 每日集成测试
```bash
# 启动全流程测试
./scripts/full_test_suite.sh

# 监控关键指标
rostopic echo /system_monitor
```

##### 测试用例管理
| 测试场景 | 触发条件 | 预期结果 |
|----------|----------|----------|
| 垂起机禁飞区接近 | 距离<220m | 自动规避 |
| 目标位置突变 | 位置变化>5m/s | 持续追踪 |
| 通信中断 | 丢包率>20% | 重传机制 |
| 模式切换失败 | 高度>25m未切换 | 安全降落 |

##### 性能优化流程
1. 基准测试：记录当前T1时间
2. 分析瓶颈：`rosrun rqt_graph rqt_graph`
3. 优化方案：
   - 算法组：减少识别延迟
   - 控制组：优化路径平滑度
4. 验证结果：对比优化前后T1

##### 日志管理规范
1. 每次测试必须录制BAG：
   ```bash
   rosbag record -O test_$(date +%Y%m%d_%H%M) \
   /standard_vtol_0/* /iris_0/* /zhihang/*
   ```
2. 提交版本BAG命名：
   `score1_v${version}_${date}.bag`

##### 持续集成配置
`.github/workflows/ci.yaml`:
```yaml
name: Competition CI
on: [push]
jobs:
  integration-test:
    runs-on: ubuntu-20.04
    container: 
      image: rescue-sim:1.0
    steps:
      - name: Run full test
        run: |
          source /opt/ros/noetic/setup.bash
          ./launch_test.sh
      - name: Check results
        run: |
          python check_score.py output.log
```

#### 协作要求
- 每日19:00发布测试报告
- 关键问题即时通报（Slack #urgent）
- 提交前72小时冻结代码


### 文档与材料组操作手册


#### 文档与材料组工作流程

##### 文档结构规划

docs/
├── design_docs/
│   ├── architecture.md  # 系统架构
│   └── algorithm.md     # 算法设计
├── test_reports/
│   ├── performance/     # 性能测试
│   └── compliance/      # 规则符合性
└── submission/
    ├── final_report.pdf # 总结报告
    └── demo_video.mp4   # 录屏视频


##### 材料制作流程
1. **报告编写**：
   - 创新点提取（技术路线可行性5分）
   - 逻辑链条构建（研究思路合理性10分）

2. **视频录制**：
   ```bash
   # 使用Gazebo内置录制
   gazebo --record -o competition_demo
   # 后期编辑
   ffmpeg -i raw.ogv -vf "drawtext=text='Team Name':x=10:y=10" final.mp4
   ```

3. **代码注释规范**：
   ```python
   def detect_target(image):
       """
       目标检测核心函数
       :param image: 输入图像 (RGB格式)
       :return: (x, y, class_id, confidence)
       :rule: 需满足SCORE1评分误差<10m
       """
   ```

#### 提交材料清单
1. 必交材料：
   - `summary_report.pdf`（含设计思路、创新点）
   - `demo_video.mp4`（1920x1080, ≤3分钟）
   - `score1.bag`（符合官方要求）
   - `source_code.zip`（完整可运行代码）

2. 材料验证清单：
   - [ ] 报告页码编号完整
   - [ ] 视频清晰展示关键流程
   - [ ] BAG文件包含所有要求话题
   - [ ] 代码无外部依赖

#### 原创性保障措施
1. 代码相似度检查：
   ```bash
   flake8 --plagiarism-check src/
   ```
2. 文档查重：
   - 使用Turnitin检查报告文本
   - 视频关键帧反向搜索

## 协作要求
- 每周五收集各组技术亮点
- 关键算法变更后更新设计文档
- 提交前48小时完成材料预审




 **重要提示**：所有文档和代码必须每日备份至云端存储，使用命令：
```bash
rclone copy ~/drone-rescue remote:backup/rescue_$(date +%u)
```
每周五进行全量备份：`./scripts/full_backup.sh`
