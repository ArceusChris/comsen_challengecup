#!/bin/bash

# 无人机仿真环境 Docker 启动脚本
# 作者：XYLin
# 日期：2025-07-05

echo "========================================"
echo "  无人机双机协同自主搜救任务仿真启动脚本"
echo "========================================"

# 检查Docker是否运行
if ! docker info > /dev/null 2>&1; then
    echo "错误：Docker未运行或无权限访问Docker"
    echo "请确保Docker已启动并且当前用户在docker组中"
    exit 1
fi

# 检查docker-compose是否存在
if ! command -v docker-compose &> /dev/null; then
    echo "错误：docker-compose未安装"
    echo "请先安装docker-compose"
    exit 1
fi

# 检查是否有X11显示
if [ -z "$DISPLAY" ]; then
    echo "警告：未检测到X11显示环境"
    echo "如果需要GUI，请确保已设置DISPLAY变量"
fi

# 检查GPU支持
if command -v nvidia-smi &> /dev/null; then
    echo "检测到NVIDIA GPU支持"
    GPU_SUPPORT=true
else
    echo "未检测到NVIDIA GPU或驱动"
    GPU_SUPPORT=false
fi

# 选择启动模式
echo ""
echo "请选择启动模式："
echo "1. 完整GUI模式（需要X11支持）"
echo "2. 无头模式（适合开发调试）"
echo "3. 仅构建不启动"
echo "4. 清理并重新构建"
echo "5. 停止并清理容器"
read -p "请输入选择 (1-5): " choice

case $choice in
    1)
        echo "启动完整GUI模式..."
        # 允许X11连接
        xhost +local:docker 2>/dev/null || echo "无法设置X11权限，但继续尝试启动"
        docker-compose up -d
        echo "容器已启动，正在进入容器..."
        echo "注意：容器内已有启动脚本 /root/start_px4_gazebo.sh 可用于启动仿真"
        docker-compose exec drone_sim /bin/bash
        ;;
    2)
        echo "启动无头模式..."
        # 临时修改环境变量以支持无头模式
        DISPLAY="" docker-compose up -d
        echo "容器已启动，正在进入容器..."
        echo "注意：容器内已有启动脚本 /root/start_px4_headless.sh 可用于启动无头仿真"
        docker-compose exec drone_sim /bin/bash
        ;;
    3)
        echo "构建镜像..."
        docker-compose build
        echo "构建完成"
        ;;
    4)
        echo "清理并重新构建..."
        docker-compose down
        docker-compose build --no-cache
        echo "重新构建完成"
        ;;
    5)
        echo "停止并清理容器..."
        docker-compose down
        echo "容器已停止并清理"
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac
