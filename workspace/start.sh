#!/bin/bash

# 无人机双机协同自主搜救任务 - 全自动启动脚本
# 作者：基于赛题操作步骤自动化脚本
# 日期：2025年7月6日

# 设置错误处理
set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# 检查必要的目录和文件
check_prerequisites() {
    log_step "检查系统环境和必要文件..."
    
    # 检查ROS环境
    if ! command -v roscore &> /dev/null; then
        log_error "ROS未安装或未正确配置"
        exit 1
    fi
    
    # 检查必要目录
    directories=(
        "$HOME/PX4_Firmware"
        "$HOME/XTDrone"
        "$HOME/XTDrone/communication"
        "$HOME/XTDrone/sensing/pose_ground_truth"
        "$HOME/XTDrone/zhihang2025"
    )
    
    for dir in "${directories[@]}"; do
        if [ ! -d "$dir" ]; then
            log_error "目录不存在: $dir"
            exit 1
        fi
    done
    
    log_info "系统环境检查完成"
}

# 清理之前的进程
cleanup_processes() {
    log_step "清理之前的进程..."
    
    # 杀死可能存在的相关进程
    pkill -f "roscore" || true
    pkill -f "gazebo" || true
    pkill -f "px4" || true
    pkill -f "vtol_communication" || true
    pkill -f "multirotor_communication" || true
    pkill -f "get_local_pose" || true
    pkill -f "Pub_first_point" || true
    pkill -f "Pub_downtown" || true
    pkill -f "zhihang_control_targets" || true
    pkill -f "rosbag" || true
    
    sleep 2
    log_info "进程清理完成"
}

# 设置ROS环境
setup_ros_environment() {
    log_step "设置ROS环境..."
    
    # 设置ROS环境变量
    export ROS_MASTER_URI=http://localhost:11311
    export ROS_HOSTNAME=localhost
    
    # 加载bashrc配置
    source ~/.bashrc
    
    log_info "ROS环境设置完成"
}

# 启动仿真程序
start_simulation() {
    log_step "启动仿真程序..."
    
    cd ~/PX4_Firmware
    
    # 在新终端中启动仿真
    gnome-terminal --tab --title="仿真程序" -- bash -c "
        source ~/.bashrc
        roslaunch px4 zhihang2025.launch
        exec bash
    " &
    
    # 等待仿真程序启动
    log_info "等待仿真程序启动..."
    sleep 15
    
    # 检查仿真是否启动成功
    if ! pgrep -f "gazebo" > /dev/null; then
        log_error "仿真程序启动失败"
        exit 1
    fi
    
    log_info "仿真程序启动成功"
}

# 启动通信脚本
start_communication() {
    log_step "启动通信脚本..."
    
    # 启动垂起无人机通信
    gnome-terminal --tab --title="垂起无人机通信" -- bash -c "
        cd ~/XTDrone/communication/
        source ~/.bashrc
        python3 vtol_communication.py standard_vtol 0
        exec bash
    " &
    
    sleep 3
    
    # 启动旋翼无人机通信
    gnome-terminal --tab --title="旋翼无人机通信" -- bash -c "
        cd ~/XTDrone/communication/
        source ~/.bashrc
        python3 multirotor_communication.py iris 0
        exec bash
    " &
    
    sleep 3
    log_info "通信脚本启动完成"
}

# 启动位姿真值服务
start_pose_ground_truth() {
    log_step "启动位姿真值服务..."
    
    # 启动垂起无人机位姿真值
    gnome-terminal --tab --title="垂起无人机位姿" -- bash -c "
        cd ~/XTDrone/sensing/pose_ground_truth/
        source ~/.bashrc
        python3 get_local_pose.py standard_vtol 1
        exec bash
    " &
    
    sleep 2
    
    # 启动旋翼无人机位姿真值
    gnome-terminal --tab --title="旋翼无人机位姿" -- bash -c "
        cd ~/XTDrone/sensing/pose_ground_truth/
        source ~/.bashrc
        python3 get_local_pose.py iris 1
        exec bash
    " &
    
    sleep 2
    log_info "位姿真值服务启动完成"
}

# 启动GPS引导点发布
start_gps_publisher() {
    log_step "启动GPS引导点发布..."
    
    gnome-terminal --tab --title="GPS引导点" -- bash -c "
        cd ~/XTDrone/zhihang2025
        source ~/.bashrc
        python3 Pub_first_point.py
        exec bash
    " &
    
    sleep 2
    log_info "GPS引导点发布启动完成"
}

# 启动居民区中心坐标发布
start_downtown_publisher() {
    log_step "启动居民区中心坐标发布..."
    
    gnome-terminal --tab --title="居民区坐标" -- bash -c "
        cd ~/XTDrone/zhihang2025
        source ~/.bashrc
        python3 Pub_downtown.py
        exec bash
    " &
    
    sleep 2
    log_info "居民区中心坐标发布启动完成"
}

# 启动目标移动控制
start_target_control() {
    log_step "启动目标移动控制..."
    
    gnome-terminal --tab --title="目标移动控制" -- bash -c "
        cd ~/XTDrone/zhihang2025
        source ~/.bashrc
        python3 zhihang_control_targets.py
        exec bash
    " &
    
    sleep 2
    log_info "目标移动控制启动完成"
}

# 启动数据记录
start_data_recording() {
    log_step "启动数据记录..."
    
    cd ~/XTDrone/zhihang2025
    
    # 创建日志目录
    mkdir -p logs
    
    # 生成带时间戳的文件名
    timestamp=$(date +"%Y%m%d_%H%M%S")
    
    gnome-terminal --tab --title="数据记录" -- bash -c "
        cd ~/XTDrone/zhihang2025
        source ~/.bashrc
        rosbag record -O score1_${timestamp} \
            /standard_vtol_0/mavros/state \
            /iris_0/mavros/state \
            /gazebo/model_states \
            /xtdrone/standard_vtol_0/cmd \
            /xtdrone/iris_0/cmd \
            /zhihang/first_point \
            /zhihang2025/first_man/pose \
            /zhihang2025/second_man/pose \
            /zhihang2025/third_man/pose \
            /zhihang2025/iris_healthy_man/pose \
            /zhihang2025/iris_bad_man/pose \
            /zhihang/downtown
        exec bash
    " &
    
    sleep 2
    log_info "数据记录启动完成"
}

# 检查服务状态
check_services_status() {
    log_step "检查服务状态..."
    
    # 检查ROS Master
    if ! rostopic list &> /dev/null; then
        log_error "ROS Master未运行"
        return 1
    fi
    
    # 检查Gazebo
    if ! pgrep -f "gazebo" > /dev/null; then
        log_error "Gazebo未运行"
        return 1
    fi
    
    log_info "所有服务运行正常"
    return 0
}

# 显示使用说明
show_usage() {
    echo "使用说明："
    echo "1. 确保所有必要的文件已按照赛题要求放置在正确位置"
    echo "2. 运行此脚本: ./start_all_services.sh"
    echo "3. 等待所有服务启动完成"
    echo "4. 启动您的无人机控制程序"
    echo ""
    echo "注意事项："
    echo "- 请确保在启动无人机之前数据记录已经开始"
    echo "- 如需停止所有服务，请使用 Ctrl+C 或运行 ./stop_all_services.sh"
    echo "- 数据记录文件将保存在 ~/XTDrone/zhihang2025/ 目录下"
}

# 主函数
main() {
    log_info "=== 无人机双机协同自主搜救任务 - 全自动启动脚本 ==="
    echo ""
    
    # 检查先决条件
    check_prerequisites
    
    # 清理之前的进程
    cleanup_processes
    
    # 设置ROS环境
    setup_ros_environment
    
    # 启动各项服务
    start_simulation
    start_communication
    start_pose_ground_truth
    start_gps_publisher
    start_downtown_publisher
    start_target_control
    start_data_recording
    
    # 检查服务状态
    sleep 5
    if check_services_status; then
        log_info "=== 所有服务启动完成 ==="
        echo ""
        show_usage
        echo ""
        log_info "系统已准备就绪，请启动您的无人机控制程序"
    else
        log_error "某些服务启动失败，请检查日志"
        exit 1
    fi
}

# 信号处理
trap 'log_info "收到中断信号，正在清理..."; cleanup_processes; exit 0' INT TERM

# 运行主函数
main "$@"
