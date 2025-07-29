#!/bin/bash

# 视觉降落控制器启动脚本

echo "视觉降落控制器启动脚本"
echo "=========================="

# 检查参数
if [ $# -eq 0 ]; then
    echo "使用默认目标话题: landing_target_camo"
    TARGET_TOPIC="landing_target_camo"
elif [ $# -eq 1 ]; then
    TARGET_TOPIC=$1
    echo "使用指定目标话题: $TARGET_TOPIC"
else
    echo "用法: $0 [target_topic]"
    echo "支持的目标话题:"
    echo "  - landing_target_camo (默认)"
    echo "  - landing_target_red"
    exit 1
fi

# 检查ROS环境
if [ -z "$ROS_MASTER_URI" ]; then
    echo "错误: ROS环境未设置，请先source setup.bash"
    exit 1
fi

# 检查roscore是否运行
if ! rostopic list > /dev/null 2>&1; then
    echo "错误: 无法连接到ROS master，请确保roscore正在运行"
    exit 1
fi

echo "启动视觉降落控制器..."
echo "目标话题: /$TARGET_TOPIC"
echo "按 Ctrl+C 停止"
echo ""

# 切换到脚本目录
cd "$(dirname "$0")"

# 启动控制器
python3 landing.py $TARGET_TOPIC
