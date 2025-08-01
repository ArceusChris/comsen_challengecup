# VTOL Condition定时器功能修改说明

## 修改概述
将原有的手动发布condition状态功能修改为使用ROS定时器持续发布，实现实时状态监控。

## 主要修改

### 1. VTOLDemoFlight类新增成员变量
```python
# 定时器相关
self.condition_timer = None
self.condition_timer_running = False  # 跟踪定时器状态
self.condition_publish_rate = 2.0  # 2Hz发布频率
self.current_condition = 0xAA  # 当前状态
```

### 2. 新增定时器管理方法

#### start_condition_timer()
- 启动ROS定时器，以2Hz频率持续发布condition状态
- 包含异常处理机制

#### stop_condition_timer()
- 安全停止定时器
- 更新状态标志

#### timer_condition_publish_callback()
- 定时器回调函数
- 持续发布当前condition状态

#### get_condition_status()
- 获取定时器和状态信息
- 修复了Timer对象没有`is_shutdown()`方法的问题

### 3. 修改现有方法

#### init_ros()
- 在ROS通信初始化后自动启动定时器

#### shutdown()
- 在系统关闭时停止定时器

#### publish_condition()
- 立即发布并更新当前状态
- 定时器会继续持续发布

#### update_mission_condition()
- 只更新当前状态，让定时器负责发布

### 4. 任务流程中的状态管理

#### 自动状态切换
- 起飞时：0x01
- 到达目标点：从YAML文件读取的condition值
- 返航时：0x04
- 着陆完成：0x05
- 用户中断：0xFF
- 任务异常：0xEE

### 5. VTOLROSCommunicator修改

#### publish_condition()方法优化
- 支持持续发布模式
- 只在状态变化时打印日志，避免定时器产生过多输出
- 始终发布消息，确保实时性

## 功能特性

### 实时状态发布
- 2Hz频率持续发布当前condition状态
- 确保系统始终知道无人机的当前任务状态

### 状态自动管理
- 任务流程中自动切换状态
- 支持异常情况的状态处理

### 可靠性增强
- 包含完整的异常处理机制
- 定时器启动/停止的安全管理
- 状态跟踪和监控

## 使用方法

### 基本使用
```python
# 创建VTOL演示对象
demo = VTOLDemoFlight()

# 初始化（会自动启动定时器）
demo.init_ros()

# 状态会自动在任务执行过程中更新
# 定时器会持续发布当前状态

# 清理（会自动停止定时器）
demo.shutdown()
```

### 手动状态更新
```python
# 更新状态（定时器会持续发布新状态）
demo.current_condition = 0x02

# 或使用方法更新并立即发布一次
demo.publish_condition(0x02)
```

### 状态监控
```python
# 获取当前状态信息
status = demo.get_condition_status()
print(f"当前状态: 0x{status['current_condition']:02X}")
print(f"定时器状态: {status['timer_status']}")
print(f"发布频率: {status['publish_rate']}Hz")
```

## 测试

创建了测试脚本 `test_condition_timer.py` 用于验证定时器功能。

## 优势

1. **实时性**：持续发布确保系统状态的实时更新
2. **可靠性**：定时器机制比手动发布更可靠
3. **自动化**：状态在任务流程中自动管理
4. **监控性**：可以实时监控定时器和状态信息
5. **容错性**：包含完整的异常处理机制

## 注意事项

1. 定时器在ROS通信初始化时自动启动
2. 系统关闭时会自动停止定时器
3. 发布频率设置为2Hz，平衡了实时性和系统负载
4. 修复了ROS Timer对象API的兼容性问题
