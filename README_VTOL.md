# VTOL无人机自动飞行系统

## 项目简介

本项目实现了一套完整的VTOL（垂直起降）固定翼无人机自动飞行系统，具备智能路径规划、安全区域检测、自动模式切换等功能。系统可以根据预设的目标点自动执行起飞、航行、降落任务，并能智能避开禁飞区域。

### 主要特性

- 🛩️ **智能模式切换**：根据位置自动在旋翼模式和固定翼模式间切换
- 🗺️ **智能路径规划**：集成A*算法，自动规划避开居民区的安全航线
- 📍 **精确导航控制**：闭环控制系统，实现高精度位置到达
- 🚫 **安全区域检测**：实时监控飞行区域，禁止在居民区飞行
- 📊 **任务可视化**：生成飞行路径预览图和实时状态监控
- 🎯 **多目标点飞行**：支持从YAML文件加载多个目标点顺序执行

## 系统架构

```
vtol_control/
├── vtol_demo.py          # 主飞行控制脚本
├── vtol_map.py           # 地图区域定义和管理
├── vtol_Astar.py         # A*路径规划算法
├── vtol_target.yaml      # 目标点配置文件
└── README.md             # 本文档
```

## 新特性：灵活的旋翼模式切换规则

### 更新内容 (v2.0)

🆕 **新的旋翼模式切换规则**：
- **旧规则**：只有在严格的旋翼区内才能切换到旋翼模式
- **新规则**：只要距离原点(0,0)在半径100米范围内，无论地图区域类型（包括out_of_bounds），都可以切换到旋翼模式

这一改进使得无人机在边界区域也能正常执行降落和精确操作。

## 安装要求

### 系统要求
- Ubuntu 18.04/20.04
- ROS Melodic/Noetic
- Python 3.6+

### 依赖包
```bash
# ROS依赖
sudo apt install ros-$ROS_DISTRO-mavros ros-$ROS_DISTRO-mavros-extras

# Python依赖
pip3 install numpy matplotlib pyyaml scipy
```

### 仿真环境
- PX4固件
- XTDrone仿真平台
- Gazebo仿真器

## 配置文件

### 目标点配置 (vtol_target.yaml)

```yaml
targets:
  - name: "takeoff_point"
    description: "起飞点"
    position: [0, 0, 0]
    
  - name: "target_north"
    description: "北侧目标点"
    position: [1600, 200, 20]
    
  - name: "target_south"
    description: "南侧目标点"
    position: [1600, -200, 20]
    
  - name: "landing_point"
    description: "降落点"
    position: [0, 0, 0]
```

### 地图区域定义

系统预定义了以下区域类型：
- **旋翼区** (MULTIROTOR)：中心(0,0)，半径150米，可使用旋翼模式
- **自由空间** (FREE_SPACE)：开放飞行区域，必须使用固定翼模式
- **居民区** (RESIDENTIAL)：禁飞区域，无人机不得进入

## 使用方法

### 1. 启动仿真环境

```bash
# 启动ROS核心
roscore

# 启动XTDrone仿真（另一个终端）
cd ~/XTDrone
python3 coordination/multi_vehicle_spawn.py

# 启动PX4仿真（另一个终端）
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

### 2. 运行飞行任务

```bash
# 进入项目目录
cd /home/yzy/comsen_challengecup

# 运行主飞行脚本
python3 workspace/vtol_control/vtol_demo.py
```

### 3. 监控飞行状态

脚本会输出详细的飞行状态信息：
- 起飞序列执行情况
- A*路径规划结果
- 实时位置和距离信息
- 模式切换状态
- 安全检查结果

## 核心功能说明

### 1. 自动起飞序列

```python
def takeoff_sequence(self):
    """起飞序列 - 新规则：距离(0,0)在100米内可起飞"""
    # 位置检查和修正
    # 模式切换验证
    # 分阶段起飞控制
    # 高度验证
```

### 2. 智能路径规划

```python
def find_safe_waypoints(self, start_x, start_y, end_x, end_y):
    """使用A*算法规划安全航点，避开居民区"""
    # A*算法路径规划
    # 障碍物避让
    # 路径优化
    # 安全性验证
```

### 3. 精确导航控制

```python
def precise_fly_to_position(self, target_x, target_y, target_z, description):
    """精确飞向位置 - 使用闭环控制"""
    # 分阶段接近策略
    # 实时位置反馈
    # 动态容忍度调整
    # 稳定性检测
```

### 4. 模式切换规则

```python
def can_switch_to_multirotor(self, x=None, y=None):
    """检查是否可以切换到旋翼模式
    新规则：只要距离(0,0)在半径100米范围内，无论zone类型都允许切换
    """
    distance_to_origin = math.sqrt(x**2 + y**2)
    return distance_to_origin <= 100.0
```

## 飞行流程

1. **系统初始化**
   - ROS节点启动
   - 地图和路径规划器初始化
   - 目标点加载

2. **起飞准备**
   - 位置验证和修正
   - 安全检查
   - 模式切换到旋翼模式

3. **任务执行**
   - A*路径规划
   - 分段航行控制
   - 实时安全监控
   - 精确到达验证

4. **降落程序**
   - 模式切换判断
   - 分阶段下降
   - 自动降落执行

## 安全特性

### 区域安全检查
- 实时监控飞行区域类型
- 禁止进入居民区
- 自动规避障碍物

### 模式切换安全
- 基于距离的智能判断
- 防止在不当区域切换模式
- 紧急情况强制保持安全模式

### 路径规划安全
- A*算法自动避障
- 多重路径验证
- 紧急脱离程序

## 性能参数

- **控制频率**：50Hz
- **导航精度**：10-15米
- **模式切换时间**：3秒
- **A*网格大小**：20米
- **安全飞行高度**：15米以上

## 可视化输出

系统会自动生成以下可视化内容：
- 任务路径预览图 (`vtol_mission_astar_preview.png`)
- 实时飞行状态显示
- A*路径规划结果图
- 区域安全分析图

## 故障排除

### 常见问题

1. **无法获取位置信息**
   ```bash
   # 检查MAVLink连接
   rostopic echo /standard_vtol_0/mavros/local_position/pose
   ```

2. **模式切换失败**
   - 确认无人机在100米半径内
   - 检查xtdrone命令发布

3. **路径规划失败**
   - 验证目标点是否在居民区内
   - 检查A*算法参数设置

### 调试模式

启用详细日志输出：
```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

## 开发团队

- 主要开发者：挑战杯团队
- 项目时间：2025年7月
- 版本：v2.0

## 更新日志

### v2.0 (2025-07-27)
- ✅ 新增灵活的旋翼模式切换规则（100米半径）
- ✅ 优化边界区域处理逻辑
- ✅ 改进降落程序安全性
- ✅ 增强位置修正算法

### v1.0 (2025-07-26)
- ✅ 基础VTOL飞行控制
- ✅ A*路径规划集成
- ✅ 闭环控制系统
- ✅ 安全区域检测

## 许可证

本项目采用MIT许可证，详见LICENSE文件。

## 联系方式

如有问题或建议，请联系开发团队。

---

**注意**：使用前请确保仿真环境正确配置，并在安全的测试环境中运行。
