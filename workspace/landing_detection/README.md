# 降落平台检测包

这个ROS包提供了用于检测白色圆圈+十字降落平台的计算机视觉算法。

## 功能特性

- 检测带有白色十字的白色圆形降落平台（基础版本）
- **新增：检测黑白灰迷彩图案+白色圆圈+白色十字的降落平台**
- **新增：检测红色图案+白色圆圈+白色十字的降落平台**
- 实时图像处理和平台定位
- 提供像素坐标系下的目标中心位置
- 多种检测算法适应不同场景
- 包含调试可视化功能

## 支持的降落平台类型

### 1. 基础白色平台
- 纯白色圆形，中心带有白色十字
- 适用于简单背景环境

### 2. 黑白灰迷彩平台 ⭐
- **外围**：黑白灰迷彩图案的圆形
- **中心**：白色圆圈（与迷彩圆形同圆心）
- **标记**：白色十字在圆圈中心
- 适用于复杂背景环境，抗干扰能力强

### 3. 红色平台 ⭐
- **外围**：纯红色圆形
- **中心**：白色圆圈（与红色圆形同圆心）
- **标记**：白色十字在圆圈中心
- 适用于绿色或蓝色背景环境，视觉对比度高

## 文件说明

### 脚本文件 (scripts/)

1. **landing_detection.py** - 完整版检测器（基础白色平台）
   - 使用霍夫圆变换检测圆形
   - 验证圆心十字图案
   - 提供详细的置信度评估

2. **simple_landing_detector.py** - 简化版检测器（基础白色平台）
   - 基于轮廓检测的快速算法
   - 圆形度验证
   - 轻量级十字检测

3. **camouflage_pattern_detector.py** - 迷彩图案检测器 ⭐
   - 基于纹理特征分析的迷彩识别
   - 边缘检测和圆形检测相结合
   - 局部二值模式(LBP)纹理分析
   - 内部白色圆圈和十字验证

4. **red_pattern_detector.py** - 红色图案检测器 ⭐
   - HSV色彩空间红色检测
   - 处理红色在色彩空间的特殊性
   - 颜色过滤和几何形状结合
   - 内部白色圆圈和十字验证

5. **test_landing_detection.py** - 基础测试客户端
   - 订阅基础检测结果并显示
   - 实时状态监控
   - 调试图像显示

6. **test_multi_pattern.py** - 多模式测试客户端 ⭐
   - 同时监控多种图案检测结果
   - 智能选择最佳检测结果
   - 多窗口调试图像显示

### 启动文件 (launch/)

- **landing_detection.launch** - 启动基础检测系统
- **camouflage_detection.launch** - 启动迷彩图案检测器 ⭐
- **red_detection.launch** - 启动红色图案检测器 ⭐
- **multi_pattern_detection.launch** - 启动多模式检测系统 ⭐

## 话题接口

### 输入话题
- `/iris_0/camera/image_raw` (sensor_msgs/Image) - 下视摄像头图像

### 输出话题
- `/landing_platform/center` (geometry_msgs/Point) - 完整版检测器输出
- `/landing_target` (geometry_msgs/PointStamped) - 简化版检测器输出
- `/landing_target_camo` (geometry_msgs/PointStamped) - 迷彩图案检测器输出 ⭐
- `/landing_target_red` (geometry_msgs/PointStamped) - 红色图案检测器输出 ⭐
- `/landing_platform/debug_image` (sensor_msgs/Image) - 完整版调试图像
- `/landing_debug` (sensor_msgs/Image) - 简化版调试图像
- `/landing_debug_camo` (sensor_msgs/Image) - 迷彩图案调试图像 ⭐
- `/landing_debug_red` (sensor_msgs/Image) - 红色图案调试图像 ⭐

## 使用方法

### 1. 编译包
```bash
cd ~/catkin_ws
catkin build landing_detection
source devel/setup.bash
```

### 2. 启动检测器

#### 基础检测器（白色圆圈+十字）
```bash
# 启动完整版检测器
roslaunch landing_detection landing_detection.launch

# 或者单独运行简化版检测器
rosrun landing_detection simple_landing_detector.py
```

#### 迷彩图案检测器
```bash
# 启动迷彩图案检测器
roslaunch landing_detection camouflage_detection.launch

# 或者单独运行
rosrun landing_detection camouflage_pattern_detector.py
```

#### 红色图案检测器
```bash
# 启动红色图案检测器
roslaunch landing_detection red_detection.launch

# 或者单独运行
rosrun landing_detection red_pattern_detector.py
```

#### 多模式检测器（推荐）
```bash
# 同时启动迷彩和红色检测器
roslaunch landing_detection multi_pattern_detection.launch

# 只启动迷彩检测器
roslaunch landing_detection multi_pattern_detection.launch mode:=camouflage

# 只启动红色检测器
roslaunch landing_detection multi_pattern_detection.launch mode:=red
```

### 3. 测试检测效果
```bash
# 运行基础测试客户端
rosrun landing_detection test_landing_detection.py

# 运行多模式测试客户端（推荐）
rosrun landing_detection test_multi_pattern.py
```

### 4. 查看检测结果
```bash
# 基础检测器结果
rostopic echo /landing_target
rostopic echo /landing_platform/center

# 迷彩图案检测结果
rostopic echo /landing_target_camo

# 红色图案检测结果
rostopic echo /landing_target_red
```

### 5. 查看调试图像
```bash
# 使用rqt_image_view查看调试图像
rqt_image_view

# 可选择以下话题：
# /landing_debug_camo - 迷彩图案调试图像
# /landing_debug_red - 红色图案调试图像
# /landing_platform/debug_image - 基础检测器调试图像
# /landing_debug - 简化检测器调试图像
```

## 检测参数调优

### 完整版检测器参数
- `min_circle_radius`: 最小圆圈半径 (默认: 30)
- `max_circle_radius`: 最大圆圈半径 (默认: 200)
- `white_threshold`: 白色像素阈值 (默认: 200)
- `cross_length_ratio`: 十字相对圆圈的长度比例 (默认: 0.3)

### 简化版检测器参数
- `min_contour_area`: 最小轮廓面积 (默认: 1000)
- `circularity_threshold`: 圆形度阈值 (默认: 0.6)

### 迷彩图案检测器参数 ⭐
- `white_threshold`: 白色像素阈值 (默认: 200)
- `min_circle_radius`: 最小圆圈半径 (默认: 30)
- `max_circle_radius`: 最大圆圈半径 (默认: 200)
- `texture_window_size`: 纹理分析窗口大小 (默认: 15)
- `edge_threshold`: 边缘检测阈值 (默认: 50)
- `circularity_threshold`: 圆形度阈值 (默认: 0.6)

### 红色图案检测器参数 ⭐
- `red_lower1/upper1`: 红色HSV范围1 (默认: [0,100,100] - [10,255,255])
- `red_lower2/upper2`: 红色HSV范围2 (默认: [170,100,100] - [180,255,255])
- `white_lower/upper`: 白色HSV范围 (默认: [0,0,200] - [180,30,255])
- `min_circle_radius`: 最小圆圈半径 (默认: 30)
- `max_circle_radius`: 最大圆圈半径 (默认: 200)
- `circularity_threshold`: 圆形度阈值 (默认: 0.65)
- `red_area_threshold`: 红色区域面积比例阈值 (默认: 0.6)

## 算法原理

### 基础检测流程
1. **图像预处理**: 转换为灰度图，高斯滤波去噪
2. **白色区域提取**: 基于阈值的二值化处理
3. **圆形检测**: 
   - 完整版: 使用霍夫圆变换
   - 简化版: 基于轮廓的圆形度检测
4. **十字验证**: 在圆心区域检测水平和垂直白线
5. **置信度评估**: 综合圆形度和十字检测结果

### 迷彩图案检测算法 ⭐
1. **图像增强**: 自适应直方图均衡化增强对比度
2. **边缘检测**: Canny边缘检测器识别迷彩边界
3. **纹理分析**: 
   - 灰度方差分析识别迷彩纹理
   - 边缘密度计算
   - 局部二值模式(LBP)纹理特征
4. **圆形检测**: 基于边缘的霍夫圆变换
5. **内部结构验证**: 
   - 白色圆圈检测和圆形度验证
   - 十字标记检测
6. **综合评分**: 纹理特征40% + 白色圆圈30% + 十字标记30%

### 红色图案检测算法 ⭐
1. **色彩空间转换**: BGR转HSV，更适合颜色检测
2. **红色提取**: 
   - 处理红色跨越0度的特殊性
   - 双范围掩码合并
3. **形态学处理**: 去除噪声，连接断裂区域
4. **几何验证**: 
   - 轮廓圆形度计算
   - 最小外接圆拟合
5. **内部结构检测**: 
   - 白色圆圈识别和定位
   - 十字标记验证
6. **综合评分**: 红色比例30% + 白色圆圈35% + 十字标记25% + 圆形度10%

### 坐标系说明
- 输出坐标为图像像素坐标系
- 原点在图像左上角
- X轴向右，Y轴向下
- 需要根据相机参数转换为真实世界坐标

## 依赖项

- ROS Noetic
- OpenCV (python3-opencv)
- NumPy (python3-numpy)
- cv_bridge
- sensor_msgs
- geometry_msgs

## 注意事项

### 通用注意事项
1. 确保摄像头图像质量良好，光照条件适中
2. 降落平台应与背景有明显对比
3. 十字标记应清晰可见，线条粗细适中
4. 调试图像可以帮助调优检测参数
5. 根据实际应用场景调整检测阈值

### 迷彩图案特殊注意事项 ⭐
1. **纹理对比度**: 迷彩图案应有足够的灰度变化
2. **边缘清晰度**: 确保迷彩边界与背景有明显区分
3. **内部圆圈**: 白色圆圈应占外圆面积的30-70%
4. **环境适应**: 适合复杂背景，但需要足够的光照
5. **纹理复杂度**: 避免过于简单或过于复杂的迷彩图案

### 红色图案特殊注意事项 ⭐
1. **颜色纯度**: 使用饱和度较高的红色，避免偏橙或偏紫
2. **光照条件**: 红色在不同光照下可能有色偏，需要调整HSV范围
3. **背景颜色**: 避免红色背景，绿色或蓝色背景效果最佳
4. **反光处理**: 避免过强反光影响颜色检测
5. **相机设置**: 确保相机白平衡设置正确

## 故障排除

### 基础检测问题
1. **无法检测到目标**:
   - 检查白色阈值设置
   - 确认降落平台在视野范围内
   - 调整最小/最大圆圈半径参数

2. **误检测较多**:
   - 增加圆形度阈值
   - 提高十字检测的严格程度
   - 增加最小轮廓面积

3. **检测不稳定**:
   - 优化图像预处理参数
   - 增加时间滤波
   - 考虑使用卡尔曼滤波跟踪

### 迷彩图案检测问题 ⭐
1. **纹理识别失败**:
   - 调整`edge_threshold`参数
   - 检查迷彩图案对比度
   - 增加`texture_window_size`

2. **边缘检测效果差**:
   - 优化光照条件
   - 调整Canny边缘检测参数
   - 检查图像清晰度

3. **内部圆圈检测失败**:
   - 确认白色圆圈与迷彩的对比度
   - 调整白色检测阈值
   - 检查圆圈大小比例

### 红色图案检测问题 ⭐
1. **红色检测失败**:
   - 检查HSV颜色范围设置
   - 确认光照条件影响
   - 调整饱和度和亮度阈值

2. **颜色误检**:
   - 缩小HSV检测范围
   - 增加形态学操作强度
   - 检查背景颜色干扰

3. **在不同光照下不稳定**:
   - 使用自适应颜色范围
   - 增加光照补偿算法
   - 考虑使用多个HSV范围

### 通用性能优化
1. **提高检测速度**:
   - 降低图像分辨率
   - 减少检测区域
   - 优化算法参数

2. **提高检测精度**:
   - 增加多帧融合
   - 使用卡尔曼滤波
   - 实现多算法融合
