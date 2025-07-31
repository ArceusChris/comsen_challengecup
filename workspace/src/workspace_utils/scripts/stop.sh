#!/bin/bash

# 无人机双机协同自主搜救任务 - 停止所有服务脚本
# 作者：基于赛题操作步骤自动化脚本
# 日期：2025年7月6日

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

# 停止所有相关进程
stop_all_processes() {
    log_step "停止所有相关进程..."
    
    # 定义要停止的进程列表
    processes=(
        "rosbag"
        "zhihang_control_targets"
        "Pub_downtown"
        "Pub_first_point"
        "get_local_pose"
        "multirotor_communication"
        "vtol_communication"
        "gazebo"
        "px4"
        "roscore"
        "roslaunch"
        "rosmaster"
    )
    
    # 逐个停止进程
    for process in "${processes[@]}"; do
        if pgrep -f "$process" > /dev/null; then
            log_info "停止进程: $process"
            pkill -f "$process" || log_warn "无法停止进程: $process"
            sleep 1
        fi
    done
    
    # 强制杀死残留进程
    sleep 2
    for process in "${processes[@]}"; do
        if pgrep -f "$process" > /dev/null; then
            log_warn "强制停止进程: $process"
            pkill -9 -f "$process" || true
        fi
    done
    
    log_info "所有进程已停止"
}

# 清理临时文件
cleanup_temp_files() {
    log_step "清理临时文件..."
    
    # 清理可能的临时文件
    temp_files=(
        "/tmp/gazebo*"
        "/tmp/px4*"
        "/tmp/ros*"
        "$HOME/.ros/log/*"
    )
    
    for pattern in "${temp_files[@]}"; do
        if ls $pattern &> /dev/null; then
            log_info "清理临时文件: $pattern"
            rm -rf $pattern 2>/dev/null || log_warn "无法清理: $pattern"
        fi
    done
    
    log_info "临时文件清理完成"
}

# 检查进程是否完全停止
check_processes_stopped() {
    log_step "检查进程是否完全停止..."
    
    remaining_processes=()
    
    # 检查关键进程
    key_processes=("gazebo" "px4" "roscore" "rosbag")
    
    for process in "${key_processes[@]}"; do
        if pgrep -f "$process" > /dev/null; then
            remaining_processes+=("$process")
        fi
    done
    
    if [ ${#remaining_processes[@]} -eq 0 ]; then
        log_info "所有进程已完全停止"
        return 0
    else
        log_warn "以下进程仍在运行: ${remaining_processes[*]}"
        return 1
    fi
}

# 显示停止后的状态
show_status() {
    log_step "显示系统状态..."
    
    echo ""
    echo "系统状态:"
    echo "- ROS Master: $(pgrep -f "roscore" > /dev/null && echo "运行中" || echo "已停止")"
    echo "- Gazebo: $(pgrep -f "gazebo" > /dev/null && echo "运行中" || echo "已停止")"
    echo "- PX4: $(pgrep -f "px4" > /dev/null && echo "运行中" || echo "已停止")"
    echo "- 数据记录: $(pgrep -f "rosbag" > /dev/null && echo "运行中" || echo "已停止")"
    echo ""
    
    # 检查数据记录文件
    if [ -d "$HOME/XTDrone/zhihang2025" ]; then
        bag_files=$(find "$HOME/XTDrone/zhihang2025" -name "*.bag" -mtime -1 2>/dev/null | wc -l)
        if [ $bag_files -gt 0 ]; then
            log_info "发现 $bag_files 个最近的数据记录文件"
            echo "数据记录文件位置: $HOME/XTDrone/zhihang2025/"
            find "$HOME/XTDrone/zhihang2025" -name "*.bag" -mtime -1 -exec ls -lh {} \; 2>/dev/null | tail -5
        fi
    fi
}

# 主函数
main() {
    log_info "=== 无人机双机协同自主搜救任务 - 停止所有服务 ==="
    echo ""
    
    # 停止所有进程
    stop_all_processes
    
    # 清理临时文件
    cleanup_temp_files
    
    # 检查进程状态
    if check_processes_stopped; then
        log_info "=== 所有服务已成功停止 ==="
    else
        log_warn "=== 部分进程可能仍在运行，请手动检查 ==="
    fi
    
    # 显示状态
    show_status
    
    echo ""
    log_info "系统已清理完成，可以重新启动服务"
}

# 运行主函数
main "$@"
