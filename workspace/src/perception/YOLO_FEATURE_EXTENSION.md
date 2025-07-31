# YOLO11推理节点功能扩展说明

## 新增功能概述

为YOLO11推理节点增加了基于VTOL着陆状态的条件点云发布功能。

## 功能详情

### 1. 机型支持
- **iris机型**: 始终发布点云数据（保持原有行为）
- **standard_vtol机型**: 根据VTOL着陆标志控制点云发布

### 2. VTOL着陆标志控制逻辑
- **订阅话题**: `/zhihang2025/vtol_land_sub/done` (类型: `std_msgs/Int32`)
- **控制逻辑**:
  - 当标志值 = 2 时：发布点云数据
  - 当标志值 ≠ 2 时：仅发布实时位置数据，不发布点云

### 3. 发布话题

#### 实时位置话题（新增）
所有机型在检测到目标时都会发布实时位置：
- `/yolo11/position/red` (类型: `geometry_msgs/Point`)
- `/yolo11/position/yellow` (类型: `geometry_msgs/Point`) 
- `/yolo11/position/white` (类型: `geometry_msgs/Point`)

#### 点云话题（条件发布）
根据机型和VTOL标志状态决定是否发布：
- `/yolo11/pointcloud/red` (类型: `sensor_msgs/PointCloud2`)
- `/yolo11/pointcloud/yellow` (类型: `sensor_msgs/PointCloud2`)
- `/yolo11/pointcloud/white` (类型: `sensor_msgs/PointCloud2`)

### 4. 状态显示
在检测图像上添加了以下信息显示：
- VTOL标志当前值
- 点云发布状态（启用/禁用）

## 使用方法

### 启动命令
```bash
# iris机型（保持原有行为）
rosrun your_package yolo11_inference.py _aircraft_type:=iris

# standard_vtol机型（新增条件控制）
rosrun your_package yolo11_inference.py _aircraft_type:=standard_vtol
```

### 监控话题
```bash
# 监控VTOL着陆标志
rostopic echo /zhihang2025/vtol_land_sub/done

# 监控实时位置
rostopic echo /yolo11/position/red
rostopic echo /yolo11/position/yellow  
rostopic echo /yolo11/position/white

# 监控点云数据
rostopic echo /yolo11/pointcloud/red
rostopic echo /yolo11/pointcloud/yellow
rostopic echo /yolo11/pointcloud/white
```

### VTOL着陆标志发布示例
```bash
# 设置标志为0（不发布点云）
rostopic pub /zhihang2025/vtol_land_sub/done std_msgs/Int32 "data: 0"

# 设置标志为1（不发布点云）
rostopic pub /zhihang2025/vtol_land_sub/done std_msgs/Int32 "data: 1"

# 设置标志为2（发布点云）
rostopic pub /zhihang2025/vtol_land_sub/done std_msgs/Int32 "data: 2"
```

## 行为矩阵

| 机型 | VTOL标志 | 实时位置发布 | 点云发布 |
|------|----------|-------------|----------|
| iris | 任意值   | ✓           | ✓        |
| standard_vtol | 0 | ✓           | ✗        |
| standard_vtol | 1 | ✓           | ✗        |
| standard_vtol | 2 | ✓           | ✓        |

## 代码修改总结

1. **新增导入**: `std_msgs.msg.Int32`, `geometry_msgs.msg.Point`
2. **新增变量**: `vtol_land_flag`, `vtol_flag_lock`, `position_pubs`
3. **新增订阅者**: `/zhihang2025/vtol_land_sub/done`
4. **新增发布者**: 三个目标的实时位置发布者
5. **新增方法**: 
   - `vtol_flag_callback()`: VTOL标志回调
   - `publish_realtime_position()`: 发布实时位置
   - `should_publish_pointcloud()`: 判断是否发布点云
   - `conditional_publish_pointclouds()`: 条件点云发布
6. **修改方法**: 
   - `process_detections_and_update_points()`: 添加实时位置发布
   - `draw_detections()`: 添加VTOL状态显示
   - 所有调用`publish_pointclouds()`的地方改为`conditional_publish_pointclouds()`

## 注意事项

1. 只有当机型为`standard_vtol`时，才会订阅VTOL着陆标志话题
2. 实时位置数据会持续发布，不受VTOL标志影响
3. 点云数据累积的逻辑保持不变，只是发布受到条件控制
4. 图像显示中会显示当前的VTOL标志状态和点云发布状态
