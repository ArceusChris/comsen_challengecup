#!/bin/bash

# 精准降落系统使用示例脚本

echo "=========================================="
echo "    XTDrone 精准降落系统使用指南"
echo "=========================================="
echo

# 检查依赖
echo "1. 检查系统依赖..."
source /root/catkin_ws/devel/setup.bash

if [ ! -f "/root/catkin_ws/devel/setup.bash" ]; then
    echo "错误: 请先编译工作空间"
    echo "运行: cd /root/catkin_ws && catkin build"
    exit 1
fi

echo "✓ ROS环境已配置"

# 显示使用步骤
echo
echo "=========================================="
echo "使用步骤:"
echo "=========================================="
echo
echo "1. 启动Gazebo仿真环境:"
echo "   cd /root/XTDrone"
echo "   python3 sitl_config/sitl_gazebo_iris.py"
echo
echo "2. 启动精准降落系统 (新终端):"
echo "   source /root/catkin_ws/devel/setup.bash"
echo "   roslaunch precision_landing precision_landing.launch"
echo
echo "3. 启动测试客户端 (新终端):"
echo "   source /root/catkin_ws/devel/setup.bash"
echo "   rosrun precision_landing test_precision_landing.py"
echo
echo "4. 测试客户端控制键:"
echo "   s - 开始降落序列"
echo "   a - 中止降落"
echo "   e - 启用精准控制器"
echo "   d - 禁用精准控制器"
echo "   r - 重置到IDLE状态"
echo "   q - 退出"
echo

echo "=========================================="
echo "不同检测器启动方式:"
echo "=========================================="
echo
echo "• 迷彩图案检测:"
echo "  roslaunch precision_landing precision_landing.launch detector_type:=camouflage"
echo
echo "• 红色图案检测:"
echo "  roslaunch precision_landing precision_landing.launch detector_type:=red"
echo
echo "• 多模式检测 (推荐):"
echo "  roslaunch precision_landing precision_landing.launch detector_type:=multi"
echo
echo "• 启用视觉伺服:"
echo "  roslaunch precision_landing precision_landing.launch enable_visual_servo:=true"
echo

echo "=========================================="
echo "监控话题:"
echo "=========================================="
echo
echo "• 降落状态: rostopic echo /precision_landing/state"
echo "• 降落进度: rostopic echo /precision_landing/progress"
echo "• 目标检测: rostopic echo /landing_target*"
echo "• 位置误差: rostopic echo /precision_landing/error"
echo

echo "=========================================="
echo "调试工具:"
echo "=========================================="
echo
echo "• 图像查看: rqt_image_view"
echo "• 话题监控: rqt_topic"
echo "• 绘图工具: rqt_plot"
echo "• 日志查看: rqt_console"
echo

echo "准备开始测试? (y/n)"
read -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "启动测试环境..."
    roslaunch precision_landing test_precision_landing.launch
else
    echo "使用指南结束。请按照上述步骤手动启动系统。"
fi
