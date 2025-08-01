#!/bin/bash

# 无人机双机协同自主搜救任务 - 统一启动脚本
# 作者：参赛选手
# 日期：2025年

echo "=============================================="
echo "   无人机双机协同自主搜救任务启动脚本"
echo "=============================================="

# 设置错误处理
set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的信息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查必要的目录和文件
check_prerequisites() {
    print_info "检查运行环境..."
    
    if [ ! -d "$HOME/PX4_Firmware" ]; then
        print_error "PX4_Firmware目录不存在: $HOME/PX4_Firmware"
        exit 1
    fi
    
    if [ ! -d "$HOME/XTDrone" ]; then
        print_error "XTDrone目录不存在: $HOME/XTDrone"
        exit 1
    fi
    
    if [ ! -d "$HOME/XTDrone/zhihang2025" ]; then
        print_error "zhihang2025目录不存在: $HOME/XTDrone/zhihang2025"
        exit 1
    fi
    
    print_success "环境检查完成"
}

# 启动前准备
setup_environment() {
    print_info "设置环境变量..."
    
    # Source ROS环境
    source /opt/ros/noetic/setup.bash
    source ~/catkin_ws/devel/setup.bash
    source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware/ ~/PX4_Firmware/build/px4_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo
    
    print_success "环境变量设置完成"
}

# 清理函数 - 当脚本退出时清理所有后台进程
cleanup() {
    print_warning "正在清理后台进程..."
    
    # 杀死所有相关的后台进程
    pkill -f "roslaunch px4 zhihang2025.launch" 2>/dev/null || true
    pkill -f "vtol_communication.py" 2>/dev/null || true
    pkill -f "multirotor_communication.py" 2>/dev/null || true
    pkill -f "get_local_pose.py" 2>/dev/null || true
    pkill -f "Pub_first_point.py" 2>/dev/null || true
    pkill -f "Pub_downtown.py" 2>/dev/null || true
    pkill -f "zhihang_control_targets.py" 2>/dev/null || true
    pkill -f "rosbag record" 2>/dev/null || true
    
    # 等待进程完全退出
    sleep 2
    
    print_success "清理完成"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# 主启动函数
start_competition() {
    print_info "开始启动竞赛环境..."
    
    # 1. 启动仿真程序
    print_info "步骤1: 启动仿真程序 (zhihang2025.launch)..."
    cd ~/PX4_Firmware
    roslaunch px4 zhihang2025.launch &
    LAUNCH_PID=$!
    sleep 25  # 等待Gazebo完全启动
    print_success "仿真程序启动完成"
    
    # 2. 启动通信脚本
    print_info "步骤2: 启动通信脚本..."
    
    # 垂起无人机通信
    print_info "  启动垂起无人机通信..."
    cd ~/XTDrone/communication/
    python3 vtol_communication.py standard_vtol 0 &
    VTOL_COMM_PID=$!
    sleep 5
    
    # 旋翼无人机通信
    print_info "  启动旋翼无人机通信..."
    python3 multirotor_communication.py iris 0 &
    IRIS_COMM_PID=$!
    sleep 5
    print_success "通信脚本启动完成"
    
    # 3. 开启gazebo位姿真值
    print_info "步骤3: 开启Gazebo位姿真值..."
    
    # 垂起无人机位姿真值
    print_info "  启动垂起无人机位姿真值..."
    cd ~/XTDrone/sensing/pose_ground_truth/
    python3 get_local_pose.py standard_vtol 1 &
    VTOL_POSE_PID=$!
    sleep 5
    
    # 旋翼无人机位姿真值
    print_info "  启动旋翼无人机位姿真值..."
    python3 get_local_pose.py iris 1 &
    IRIS_POSE_PID=$!
    sleep 5
    print_success "位姿真值启动完成"
    
    # 4. 发布GPS引导点
    print_info "步骤4: 发布GPS引导点..."
    cd ~/XTDrone/zhihang2025
    python3 Pub_first_point.py &
    GPS_PID=$!
    sleep 2
    print_success "GPS引导点发布启动"
    
    # 5. 发布居民区中心坐标
    print_info "步骤5: 发布居民区中心坐标..."
    python3 Pub_downtown.py &
    DOWNTOWN_PID=$!
    sleep 2
    print_success "居民区中心坐标发布启动"
    
    # 6. 开启待救援目标移动代码
    print_info "步骤6: 开启待救援目标移动代码..."
    python3 zhihang_control_targets.py &
    TARGETS_PID=$!
    sleep 2
    print_success "目标移动代码启动"
    
    # 7. 启动数据记录
    print_info "步骤7: 启动数据记录..."
    cd ~/XTDrone/zhihang2025
    rosbag record -O score1 \
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
        /zhihang/downtown &
    ROSBAG_PID=$!
    sleep 2
    print_success "数据记录启动"
    
    print_success "所有竞赛组件启动完成！"
    print_info "=========================================="
    print_info "系统状态概览："
    print_info "  - 仿真环境: 运行中 (PID: $LAUNCH_PID)"
    print_info "  - 垂起无人机通信: 运行中 (PID: $VTOL_COMM_PID)"
    print_info "  - 旋翼无人机通信: 运行中 (PID: $IRIS_COMM_PID)"
    print_info "  - 垂起无人机位姿: 运行中 (PID: $VTOL_POSE_PID)"
    print_info "  - 旋翼无人机位姿: 运行中 (PID: $IRIS_POSE_PID)"
    print_info "  - GPS引导点: 运行中 (PID: $GPS_PID)"
    print_info "  - 居民区坐标: 运行中 (PID: $DOWNTOWN_PID)"
    print_info "  - 目标移动: 运行中 (PID: $TARGETS_PID)"
    print_info "  - 数据记录: 运行中 (PID: $ROSBAG_PID)"
    print_info "=========================================="
    print_warning "注意: 按 Ctrl+C 可以安全停止所有进程"
    
    # 等待用户中断或任何关键进程退出
    wait
}

# 显示使用说明
show_usage() {
    echo "使用方法:"
    echo "  $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -h, --help     显示此帮助信息"
    echo "  -c, --check    仅检查环境，不启动程序"
    echo ""
    echo "说明:"
    echo "  此脚本将按照比赛要求的顺序启动所有必要的程序"
    echo "  包括仿真环境、通信脚本、位姿获取、数据发布和记录等"
    echo ""
    echo "注意事项:"
    echo "  1. 确保已经正确安装并配置了PX4、XTDrone等环境"
    echo "  2. 确保zhihang2025相关文件已正确放置"
    echo "  3. 运行过程中请勿随意关闭，按Ctrl+C安全退出"
}

# 主程序入口
main() {
    # 处理命令行参数
    case "${1:-}" in
        -h|--help)
            show_usage
            exit 0
            ;;
        -c|--check)
            check_prerequisites
            print_success "环境检查通过，可以正常启动竞赛程序"
            exit 0
            ;;
        "")
            # 默认行为：启动竞赛
            ;;
        *)
            print_error "未知选项: $1"
            show_usage
            exit 1
            ;;
    esac
    
    # 执行启动流程
    check_prerequisites
    setup_environment
    start_competition
}

# 运行主程序
main "$@"
