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

# 检查PX4状态
check_px4_status() {
    log_step "检查PX4状态..."
    
    local timeout=60
    local count=0
    
    while [ $count -lt $timeout ]; do
        # 检查是否有mavros状态话题
        if rostopic list | grep -q "/mavros/state"; then
            log_info "MAVROS连接已建立"
            break
        fi
        
        sleep 1
        count=$((count + 1))
        
        if [ $count -eq $timeout ]; then
            log_error "MAVROS连接超时"
            return 1
        fi
    done
    
    # 等待PX4完成初始化
    sleep 5
    
    # 检查无人机状态
    if rostopic echo /standard_vtol_0/mavros/state -n 1 | grep -q "connected: True"; then
        log_info "垂起无人机连接正常"
    else
        log_warn "垂起无人机连接状态异常"
    fi
    
    if rostopic echo /iris_0/mavros/state -n 1 | grep -q "connected: True"; then
        log_info "旋翼无人机连接正常"
    else
        log_warn "旋翼无人机连接状态异常"
    fi
    
    log_info "PX4状态检查完成"
    return 0
}

# 等待话题可用
wait_for_topic() {
    local topic=$1
    local timeout=${2:-30}
    local count=0
    
    log_info "等待话题: $topic"
    
    while [ $count -lt $timeout ]; do
        if rostopic list | grep -q "$topic"; then
            log_info "话题 $topic 已可用"
            return 0
        fi
        
        sleep 1
        count=$((count + 1))
    done
    
    log_warn "话题 $topic 等待超时"
    return 1
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
    pkill -f "virtual_rc" || true
    
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
        export PX4_SIM_MODEL=standard_vtol
        roslaunch px4 zhihang2025.launch
        exec bash
    " &
    
    # 等待仿真程序启动
    log_info "等待仿真程序启动..."
    sleep 20
    
    # 检查仿真是否启动成功
    local timeout=60
    local count=0
    
    while [ $count -lt $timeout ]; do
        if pgrep -f "gazebo" > /dev/null; then
            log_info "Gazebo已启动"
            break
        fi
        sleep 1
        count=$((count + 1))
    done
    
    if [ $count -eq $timeout ]; then
        log_error "仿真程序启动失败"
        exit 1
    fi
    
    # 等待ROS节点完全启动
    log_info "等待ROS节点启动..."
    sleep 10
    
    log_info "仿真程序启动成功"
}

# 启动通信脚本
start_communication() {
    log_step "启动通信脚本..."
    
    # 等待必要的话题
    wait_for_topic "/gazebo/model_states" 30
    
    # 启动垂起无人机通信
    gnome-terminal --tab --title="垂起无人机通信" -- bash -c "
        cd ~/XTDrone/communication/
        source ~/.bashrc
        sleep 5
        python3 vtol_communication.py standard_vtol 0
        exec bash
    " &
    
    sleep 5
    
    # 启动旋翼无人机通信
    gnome-terminal --tab --title="旋翼无人机通信" -- bash -c "
        cd ~/XTDrone/communication/
        source ~/.bashrc
        sleep 5
        python3 multirotor_communication.py iris 0
        exec bash
    " &
    
    sleep 5
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

# 配置PX4参数以支持自主飞行
configure_px4_params() {
    log_step "配置PX4参数以支持自主飞行..."
    
    # 等待参数服务可用
    local timeout=30
    local count=0
    
    while [ $count -lt $timeout ]; do
        if rosservice list | grep -q "/standard_vtol_0/mavros/param/set" 2>/dev/null; then
            log_info "参数服务已可用"
            break
        fi
        sleep 1
        count=$((count + 1))
    done
    
    if [ $count -eq $timeout ]; then
        log_warn "参数服务等待超时，跳过参数配置"
        return 1
    fi
    
    # 禁用手动控制失效保护（对于自主飞行）
    log_info "配置失效保护参数..."
    
    # 设置参数以允许无手动控制的飞行
    rosservice call /standard_vtol_0/mavros/param/set "param_id: 'COM_RCL_EXCEPT'
value: 
  integer: 4
  real: 0.0" 2>/dev/null || log_warn "设置 COM_RCL_EXCEPT 参数失败"
    
    # 禁用RC失效保护
    rosservice call /standard_vtol_0/mavros/param/set "param_id: 'NAV_RCL_ACT'
value: 
  integer: 0
  real: 0.0" 2>/dev/null || log_warn "设置 NAV_RCL_ACT 参数失败"
    
    # 允许offboard模式无RC
    rosservice call /standard_vtol_0/mavros/param/set "param_id: 'COM_OF_LOSS_T'
value: 
  integer: 0
  real: 5.0" 2>/dev/null || log_warn "设置 COM_OF_LOSS_T 参数失败"
    
    # 对旋翼无人机进行相同配置
    if rosservice list | grep -q "/iris_0/mavros/param/set" 2>/dev/null; then
        rosservice call /iris_0/mavros/param/set "param_id: 'COM_RCL_EXCEPT'
value: 
  integer: 4
  real: 0.0" 2>/dev/null || log_warn "设置 iris COM_RCL_EXCEPT 参数失败"
        
        rosservice call /iris_0/mavros/param/set "param_id: 'NAV_RCL_ACT'
value: 
  integer: 0
  real: 0.0" 2>/dev/null || log_warn "设置 iris NAV_RCL_ACT 参数失败"
        
        rosservice call /iris_0/mavros/param/set "param_id: 'COM_OF_LOSS_T'
value: 
  integer: 0
  real: 5.0" 2>/dev/null || log_warn "设置 iris COM_OF_LOSS_T 参数失败"
    fi
    
    sleep 2
    log_info "PX4参数配置完成"
    return 0
}

# 发布虚拟手动控制信号
start_virtual_rc() {
    log_step "启动虚拟手动控制信号..."
    
    gnome-terminal --tab --title="虚拟RC" -- bash -c "
        source ~/.bashrc
        cd ~/workspace
        
        # 创建虚拟RC发布脚本
        cat > virtual_rc.py << 'EOF'
#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import RCIn
import time

def publish_virtual_rc():
    rospy.init_node('virtual_rc_publisher', anonymous=True)
    
    # 为两架无人机发布RC信号
    pub_vtol = rospy.Publisher('/standard_vtol_0/mavros/rc/in', RCIn, queue_size=10)
    pub_iris = rospy.Publisher('/iris_0/mavros/rc/in', RCIn, queue_size=10)
    
    rate = rospy.Rate(50)  # 50Hz
    
    while not rospy.is_shutdown():
        # 创建RC消息
        rc_msg = RCIn()
        rc_msg.header.stamp = rospy.Time.now()
        rc_msg.rssi = 255
        
        # 设置通道值 (1000-2000, 中位1500)
        # 通道1-4: 摇杆输入, 通道5: 模式切换
        rc_msg.channels = [1500, 1500, 1500, 1500, 2000, 1500, 1500, 1500]
        
        # 发布到两架无人机
        pub_vtol.publish(rc_msg)
        pub_iris.publish(rc_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.loginfo('Starting virtual RC publisher...')
        publish_virtual_rc()
    except rospy.ROSInterruptException:
        pass
EOF
        
        python3 virtual_rc.py
        exec bash
    " &
    
    sleep 3
    log_info "虚拟手动控制信号启动完成"
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

# 显示最终状态和指南
show_final_status() {
    log_step "显示最终状态..."
    
    echo ""
    echo "=== 系统状态检查 ==="
    
    # 检查Gazebo
    if pgrep -f "gazebo" > /dev/null; then
        log_info "✓ Gazebo 仿真环境运行中"
    else
        log_error "✗ Gazebo 未运行"
    fi
    
    # 检查MAVROS连接
    if rostopic list | grep -q "/mavros/state" 2>/dev/null; then
        log_info "✓ MAVROS 连接正常"
        
        # 检查无人机状态
        if timeout 5 rostopic echo /standard_vtol_0/mavros/state -n 1 2>/dev/null | grep -q "connected: True"; then
            log_info "✓ 垂起无人机连接正常"
        else
            log_warn "⚠ 垂起无人机连接异常"
        fi
        
        if timeout 5 rostopic echo /iris_0/mavros/state -n 1 2>/dev/null | grep -q "connected: True"; then
            log_info "✓ 旋翼无人机连接正常"
        else
            log_warn "⚠ 旋翼无人机连接异常"
        fi
    else
        log_error "✗ MAVROS 未连接"
    fi
    
    # 检查重要话题
    important_topics=(
        "/gazebo/model_states"
        "/zhihang/first_point"
        "/zhihang/downtown"
        "/standard_vtol_0/mavros/rc/in"
        "/iris_0/mavros/rc/in"
    )
    
    for topic in "${important_topics[@]}"; do
        if rostopic list | grep -q "$topic" 2>/dev/null; then
            log_info "✓ 话题 $topic 可用"
        else
            log_warn "⚠ 话题 $topic 不可用"
        fi
    done
    
    echo ""
    echo "=== 关于 PX4 警告信息 ==="
    echo "• 'Disarmed by auto preflight disarming' - 正常的初始化状态"
    echo "• 'Failsafe enabled: No manual control stick input' - 已通过虚拟RC解决"
    echo "• 这些都是正常的安全机制，不影响自主飞行"
    echo ""
    echo "=== 下一步操作指南 ==="
    echo "1. 检查上述所有服务是否正常运行"
    echo "2. 在您的控制程序中发送以下命令："
    echo "   - 首先发布控制命令（位置/速度/姿态）"
    echo "   - 设置模式：rosservice call /standard_vtol_0/mavros/set_mode \"custom_mode: 'OFFBOARD'\""
    echo "   - 武装无人机：rosservice call /standard_vtol_0/mavros/cmd/arming \"value: true\""
    echo "3. 确保在武装前已经持续发布控制命令至少2秒"
    echo ""
    echo "=== 重要提示 ==="
    echo "• 现在已配置虚拟RC信号，手动控制警告应该消失"
    echo "• 参数已设置为支持无RC的OFFBOARD飞行"
    echo "• 请确保您的控制程序持续发布控制命令"
    echo ""
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
    check_px4_status
    configure_px4_params
    start_virtual_rc
    start_communication
    start_pose_ground_truth
    start_gps_publisher
    start_downtown_publisher
    start_target_control
    start_data_recording
    configure_px4_params
    start_virtual_rc
    
    # 检查服务状态
    sleep 10
    if check_services_status; then
        log_info "=== 所有服务启动完成 ==="
        show_final_status
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
