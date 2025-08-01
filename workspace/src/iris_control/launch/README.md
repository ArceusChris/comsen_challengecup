# Iris Control Launch Files

该目录包含了多个launch文件，用于启动iris_control包中的多旋翼控制节点。

## Launch文件说明

### 1. multirotor_control.launch
**主要的多旋翼控制启动文件**

**功能：**
- 启动单个多旋翼控制节点
- 支持多机场景（通过命名空间隔离）
- 包含完整的话题重映射
- 可选的状态监控节点

**使用方法：**
```bash
# 基础启动
roslaunch iris_control multirotor_control.launch

# 指定参数启动
roslaunch iris_control multirotor_control.launch multirotor_type:=iris multirotor_id:=0 control_type:=vel

# 启用演示模式
roslaunch iris_control multirotor_control.launch demo_mode:=true visual_target_type:=landing_target_red
```

**参数：**
- `multirotor_type`: 多旋翼类型（默认：iris）
- `multirotor_id`: 多旋翼ID（默认：0）
- `control_type`: 控制类型，vel/pos/att（默认：vel）
- `demo_mode`: 是否启用演示模式（默认：false）
- `visual_target_type`: 视觉降落目标类型（默认：landing_target_camo）

### 2. visual_landing_demo.launch
**视觉降落演示启动文件**

**功能：**
- 专门用于演示视觉降落功能
- 自动启动相关的视觉检测节点
- 简化的参数配置

**使用方法：**
```bash
# 使用迷彩目标进行视觉降落演示
roslaunch iris_control visual_landing_demo.launch

# 使用红色目标进行视觉降落演示
roslaunch iris_control visual_landing_demo.launch visual_target_type:=landing_target_red

# 使用自定义目标
roslaunch iris_control visual_landing_demo.launch visual_target_type:=landing_target_custom
```

**参数：**
- `multirotor_type`: 多旋翼类型（默认：iris）
- `multirotor_id`: 多旋翼ID（默认：0）
- `control_type`: 控制类型（默认：vel）
- `visual_target_type`: 视觉目标类型（默认：landing_target_camo）

### 3. multi_drone.launch
**多机控制启动文件**

**功能：**
- 支持同时控制多架无人机（最多4架）
- 每架无人机使用独立的命名空间
- 可选的RViz可视化监控

**使用方法：**
```bash
# 启动2架无人机
roslaunch iris_control multi_drone.launch

# 启动3架无人机
roslaunch iris_control multi_drone.launch num_drones:=3

# 启动4架无人机
roslaunch iris_control multi_drone.launch num_drones:=4
```

**参数：**
- `num_drones`: 无人机数量，1-4（默认：2）
- `multirotor_type`: 多旋翼类型（默认：iris）
- `control_type`: 控制类型（默认：vel）

### 4. zhihang_competition.launch
**智航杯2025比赛专用启动文件**

**功能：**
- 专门为智航杯比赛设计
- 集成人员检测、视觉降落、固定翼通信等功能
- 支持仿真和实物模式
- 包含完整的比赛任务流程

**使用方法：**
```bash
# 比赛模式启动
roslaunch iris_control zhihang_competition.launch

# 仿真模式启动
roslaunch iris_control zhihang_competition.launch simulation:=true

# 指定无人机ID
roslaunch iris_control zhihang_competition.launch multirotor_id:=1
```

**参数：**
- `multirotor_type`: 多旋翼类型（默认：iris）
- `multirotor_id`: 多旋翼ID（默认：0）
- `control_type`: 控制类型（默认：vel）
- `simulation`: 是否启动仿真环境（默认：false）

## 节点功能说明

### 主要节点
- **multirotor_control**: 主控制节点，执行飞行任务
- **status_monitor**: 状态监控节点（可选）
- **static_tf_publisher**: 静态TF发布器

### 话题说明
- **MAVROS相关话题**: 与飞控通信的标准话题
- **视觉降落话题**: `/landing_target_*` 系列话题
- **智航杯专用话题**: `/zhihang2025/*` 系列话题

## 依赖包
确保以下包已正确安装：
- `rospy`
- `std_msgs`
- `geometry_msgs`
- `mavros_msgs`
- `sensor_msgs`
- `precision_landing`（视觉降落功能）
- `perception`（人员检测功能）
- `vtol_control`（固定翼通信功能）

## 使用建议

1. **首次使用**: 建议先使用 `multirotor_control.launch` 进行基础测试
2. **视觉降落测试**: 使用 `visual_landing_demo.launch` 进行视觉降落功能验证
3. **多机协同**: 使用 `multi_drone.launch` 进行多机场景测试
4. **比赛环境**: 使用 `zhihang_competition.launch` 进行完整的比赛任务

## 故障排除

1. **节点启动失败**: 检查依赖包是否正确安装
2. **话题通信异常**: 检查话题重映射是否正确
3. **参数错误**: 检查launch文件中的参数设置
4. **权限问题**: 确保脚本文件具有执行权限

```bash
# 给脚本添加执行权限
chmod +x /path/to/iris_control/scripts/*.py
```
