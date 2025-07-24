#!/bin/bash

# 无人机双机协同自主搜救任务 - 物理机启动脚本
# 此脚本将在物理机上启动所有必要的进程和服务

echo "=============================================="
echo "无人机双机协同自主搜救任务 - 物理机启动脚本"
echo "=============================================="

# 检查是否为root用户（某些操作可能需要）
if [ "$EUID" -eq 0 ]; then
    echo "警告: 检测到以root用户运行"
fi

# 设置环境变量 - 物理机版本
echo "设置环境变量..."

# 检查ROS环境
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    echo "找到ROS Noetic环境"
    source /opt/ros/noetic/setup.bash
elif [ -f "/opt/ros/melodic/setup.bash" ]; then
    echo "找到ROS Melodic环境"
    source /opt/ros/melodic/setup.bash
else
    echo "错误: 未找到ROS环境，请确保已安装ROS"
    exit 1
fi

# 检查catkin工作空间
CATKIN_WS=""
if [ -f "$HOME/catkin_ws/devel/setup.bash" ]; then
    CATKIN_WS="$HOME/catkin_ws"
    echo "找到catkin工作空间: $CATKIN_WS"
    source $CATKIN_WS/devel/setup.bash
elif [ -f "$(pwd)/catkin_ws/devel/setup.bash" ]; then
    CATKIN_WS="$(pwd)/catkin_ws"
    echo "找到本地catkin工作空间: $CATKIN_WS"
    source $CATKIN_WS/devel/setup.bash
else
    echo "警告: 未找到catkin工作空间，某些功能可能无法正常工作"
fi

# 检查PX4固件
PX4_PATH=""
if [ -d "$HOME/PX4-Autopilot" ]; then
    PX4_PATH="$HOME/PX4-Autopilot"
elif [ -d "$HOME/PX4_Firmware" ]; then
    PX4_PATH="$HOME/PX4_Firmware"
elif [ -d "$(pwd)/PX4-Autopilot" ]; then
    PX4_PATH="$(pwd)/PX4-Autopilot"
elif [ -d "$(pwd)/PX4_Firmware" ]; then
    PX4_PATH="$(pwd)/PX4_Firmware"
else
    echo "警告: 未找到PX4固件路径"
    echo "请确保PX4-Autopilot或PX4_Firmware在以下位置之一："
    echo "  - $HOME/PX4-Autopilot"
    echo "  - $HOME/PX4_Firmware"
    echo "  - $(pwd)/PX4-Autopilot"
    echo "  - $(pwd)/PX4_Firmware"
fi

if [ -n "$PX4_PATH" ]; then
    echo "找到PX4路径: $PX4_PATH"
    if [ -f "$PX4_PATH/Tools/setup_gazebo.bash" ]; then
        source $PX4_PATH/Tools/setup_gazebo.bash $PX4_PATH $PX4_PATH/build/px4_sitl_default
        export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_PATH
        export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_PATH/Tools/sitl_gazebo
    fi
fi

# 检查XTDrone路径
XTDRONE_PATH=""
if [ -d "$HOME/XTDrone" ]; then
    XTDRONE_PATH="$HOME/XTDrone"
elif [ -d "$(pwd)/XTDrone" ]; then
    XTDRONE_PATH="$(pwd)/XTDrone"
else
    echo "错误: 未找到XTDrone路径"
    echo "请确保XTDrone在以下位置之一："
    echo "  - $HOME/XTDrone"
    echo "  - $(pwd)/XTDrone"
    exit 1
fi

echo "找到XTDrone路径: $XTDRONE_PATH"

# 创建日志目录
LOG_DIR="$HOME/simulation_logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p $LOG_DIR
echo "日志保存在: $LOG_DIR"

# 清理之前的进程
echo "清理之前的进程..."
pkill -f "gazebo" > /dev/null 2>&1
pkill -f "roslaunch" > /dev/null 2>&1
pkill -f "roscore" > /dev/null 2>&1
pkill -f "px4" > /dev/null 2>&1
pkill -f "vtol_communication.py" > /dev/null 2>&1
pkill -f "multirotor_communication.py" > /dev/null 2>&1
pkill -f "get_local_pose.py" > /dev/null 2>&1
pkill -f "Pub_first_point.py" > /dev/null 2>&1
pkill -f "Pub_downtown.py" > /dev/null 2>&1
pkill -f "zhihang_control_targets.py" > /dev/null 2>&1
sleep 3

# 等待函数
wait_for_service() {
    local service=$1
    local timeout=${2:-30}
    echo "等待服务 $service 启动..."
    
    for i in $(seq 1 $timeout); do
        if rosservice list 2>/dev/null | grep -q "$service"; then
            echo "服务 $service 已启动"
            return 0
        fi
        sleep 1
    done
    
    echo "警告: 服务 $service 在 $timeout 秒内未启动"
    return 1
}

# 检查ROS话题是否存在
wait_for_topic() {
    local topic=$1
    local timeout=${2:-30}
    echo "等待话题 $topic 发布..."
    
    for i in $(seq 1 $timeout); do
        if rostopic list 2>/dev/null | grep -q "$topic"; then
            echo "话题 $topic 已发布"
            return 0
        fi
        sleep 1
    done
    
    echo "警告: 话题 $topic 在 $timeout 秒内未发布"
    return 1
}

echo ""
echo "步骤 1: 启动仿真程序..."
echo "========================================"

# 检查launch文件是否存在
if [ ! -f "$XTDRONE_PATH/../zhihang2025.launch" ] && [ ! -f "$PX4_PATH/launch/zhihang2025.launch" ]; then
    echo "错误: 未找到zhihang2025.launch文件"
    echo "请确保文件在以下位置之一："
    echo "  - $XTDRONE_PATH/../zhihang2025.launch"
    echo "  - $PX4_PATH/launch/zhihang2025.launch"
    exit 1
fi

nohup roslaunch px4 zhihang2025.launch > $LOG_DIR/simulation.log 2>&1 &
SIMULATION_PID=$!
echo "仿真程序已启动 (PID: $SIMULATION_PID)"

# 等待仿真环境启动
echo "等待仿真环境初始化..."
sleep 15

# 等待必要的服务启动
wait_for_service "/gazebo/get_model_state" 60

echo ""
echo "步骤 2: 运行通信脚本..."
echo "========================================"

# 启动垂起无人机通信
echo "启动垂起无人机通信..."
cd $XTDRONE_PATH/communication/
nohup python3 vtol_communication.py standard_vtol 0 > $LOG_DIR/vtol_comm.log 2>&1 &
VTOL_COMM_PID=$!
echo "垂起无人机通信已启动 (PID: $VTOL_COMM_PID)"

# 启动旋翼无人机通信
echo "启动旋翼无人机通信..."
nohup python3 multirotor_communication.py iris 0 > $LOG_DIR/iris_comm.log 2>&1 &
IRIS_COMM_PID=$!
echo "旋翼无人机通信已启动 (PID: $IRIS_COMM_PID)"

# 等待通信建立
sleep 5

echo ""
echo "步骤 3: 开启gazebo位姿真值..."
echo "========================================"

# 启动垂起无人机位姿真值
echo "启动垂起无人机位姿真值..."
cd $XTDRONE_PATH/sensing/pose_ground_truth/
nohup python3 get_local_pose.py standard_vtol 1 > $LOG_DIR/vtol_pose.log 2>&1 &
VTOL_POSE_PID=$!
echo "垂起无人机位姿真值已启动 (PID: $VTOL_POSE_PID)"

# 启动旋翼无人机位姿真值
echo "启动旋翼无人机位姿真值..."
nohup python3 get_local_pose.py iris 1 > $LOG_DIR/iris_pose.log 2>&1 &
IRIS_POSE_PID=$!
echo "旋翼无人机位姿真值已启动 (PID: $IRIS_POSE_PID)"

# 等待位姿数据开始发布
sleep 3

echo ""
echo "步骤 4: 发布两居民区间GPS引导点..."
echo "========================================"
cd $XTDRONE_PATH/zhihang2025
nohup python3 Pub_first_point.py > $LOG_DIR/first_point.log 2>&1 &
FIRST_POINT_PID=$!
echo "GPS引导点发布已启动 (PID: $FIRST_POINT_PID)"
echo "提示: 可通过订阅 /zhihang/first_point 话题获取GPS点信息"

sleep 2

echo ""
echo "步骤 5: 发布两个居民区中心坐标..."
echo "========================================"
nohup python3 Pub_downtown.py > $LOG_DIR/downtown.log 2>&1 &
DOWNTOWN_PID=$!
echo "居民区中心坐标发布已启动 (PID: $DOWNTOWN_PID)"
echo "提示: 可通过订阅 /zhihang/downtown 话题获取居民区中心点信息"

sleep 2

echo ""
echo "步骤 6: 开启待救援目标移动代码..."
echo "========================================"
nohup python3 zhihang_control_targets.py > $LOG_DIR/targets_control.log 2>&1 &
TARGETS_PID=$!
echo "待救援目标移动控制已启动 (PID: $TARGETS_PID)"

sleep 3

echo ""
echo "=============================================="
echo "所有服务启动完成！"
echo "=============================================="
echo ""
echo "进程状态:"
echo "  - 仿真程序 PID: $SIMULATION_PID"
echo "  - 垂起无人机通信 PID: $VTOL_COMM_PID"
echo "  - 旋翼无人机通信 PID: $IRIS_COMM_PID"
echo "  - 垂起无人机位姿 PID: $VTOL_POSE_PID"
echo "  - 旋翼无人机位姿 PID: $IRIS_POSE_PID"
echo "  - GPS引导点发布 PID: $FIRST_POINT_PID"
echo "  - 居民区坐标发布 PID: $DOWNTOWN_PID"
echo "  - 目标移动控制 PID: $TARGETS_PID"
echo ""
echo "日志文件位置: $LOG_DIR"
echo ""
echo "重要话题:"
echo "  - GPS引导点: /zhihang/first_point"
echo "  - 居民区中心: /zhihang/downtown"
echo ""
echo "常用命令:"
echo "  - 查看话题列表: rostopic list"
echo "  - 查看话题信息: rostopic echo /话题名"
echo "  - 停止所有进程: ./stop_all_physical.sh"
echo ""

# 保存进程ID到文件，方便后续清理
echo $SIMULATION_PID > $LOG_DIR/pids.txt
echo $VTOL_COMM_PID >> $LOG_DIR/pids.txt
echo $IRIS_COMM_PID >> $LOG_DIR/pids.txt
echo $VTOL_POSE_PID >> $LOG_DIR/pids.txt
echo $IRIS_POSE_PID >> $LOG_DIR/pids.txt
echo $FIRST_POINT_PID >> $LOG_DIR/pids.txt
echo $DOWNTOWN_PID >> $LOG_DIR/pids.txt
echo $TARGETS_PID >> $LOG_DIR/pids.txt

echo "进程ID已保存到: $LOG_DIR/pids.txt"
echo ""
echo "仿真环境已完全启动，可以开始您的任务！"
echo "按 Ctrl+C 退出监控，进程将继续在后台运行"
echo ""

# 监控进程状态
monitor_processes() {
    while true; do
        sleep 10
        echo "$(date): 检查进程状态..."
        
        # 检查关键进程是否还在运行
        if ! kill -0 $SIMULATION_PID 2>/dev/null; then
            echo "警告: 仿真程序已停止！"
        fi
        
        if ! kill -0 $VTOL_COMM_PID 2>/dev/null; then
            echo "警告: 垂起无人机通信已停止！"
        fi
        
        if ! kill -0 $IRIS_COMM_PID 2>/dev/null; then
            echo "警告: 旋翼无人机通信已停止！"
        fi
    done
}

# 设置清理函数
cleanup() {
    echo ""
    echo "接收到退出信号，正在清理..."
    
    # 读取并终止所有进程
    if [ -f "$LOG_DIR/pids.txt" ]; then
        while read pid; do
            if kill -0 $pid 2>/dev/null; then
                echo "终止进程 $pid"
                kill $pid
            fi
        done < $LOG_DIR/pids.txt
    fi
    
    echo "清理完成"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# 开始监控
monitor_processes
