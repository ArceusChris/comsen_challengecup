#!/bin/bash

# 优化的无人机启动脚本示例
# 解决ROS节点阻塞问题的最佳实践

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 示例：优化的多服务启动函数
start_multiple_ros_services() {
    log_info "启动多个ROS服务（优化版本）..."
    
    gnome-terminal --tab --title="多服务集合" -- bash -c "
        source ~/.bashrc
        
        # 创建日志目录
        mkdir -p /tmp/ros_logs
        
        # 服务启动函数
        start_service() {
            local service_name=\$1
            local command=\$2
            local log_file=\$3
            
            echo \"[\$(date '+%H:%M:%S')] 启动 \$service_name...\"
            
            # 使用nohup确保进程不会因为终端关闭而停止
            nohup bash -c \"
                source ~/.bashrc
                cd ~/workspace
                \$command
            \" > \$log_file 2>&1 &
            
            local pid=\$!
            echo \"  PID: \$pid\"
            
            # 验证进程是否成功启动
            sleep 2
            if kill -0 \$pid 2>/dev/null; then
                echo \"  ✓ \$service_name 启动成功\"
                echo \$pid > /tmp/\${service_name}_pid
                return \$pid
            else
                echo \"  ✗ \$service_name 启动失败\"
                return 0
            fi
        }
        
        echo '=== 启动ROS服务集群 ==='
        echo ''
        
        # 启动服务1：通信桥接
        start_service 'communication_bridge' 'python3 ~/XTDrone/communication/vtol_communication.py standard_vtol 0' '/tmp/ros_logs/comm_bridge.log'
        COMM_PID=\$?
        
        # 启动服务2：位姿估计
        start_service 'pose_estimator' 'python3 ~/XTDrone/sensing/pose_ground_truth/get_local_pose.py standard_vtol 1' '/tmp/ros_logs/pose_est.log'
        POSE_PID=\$?
        
        # 启动服务3：目标检测
        start_service 'object_detection' 'python3 ~/workspace/perception/yolo11_inference.py _aircraft_type:=standard_vtol' '/tmp/ros_logs/yolo_det.log'
        YOLO_PID=\$?
        
        echo ''
        echo '=== 服务状态总览 ==='
        echo \"通信桥接 PID: \$COMM_PID\"
        echo \"位姿估计 PID: \$POSE_PID\"
        echo \"目标检测 PID: \$YOLO_PID\"
        echo ''
        echo '=== 日志监控 ==='
        echo '实时查看日志: tail -f /tmp/ros_logs/*.log'
        echo '检查服务状态: ps aux | grep python3'
        echo '查看ROS话题: rostopic list'
        echo ''
        
        # 服务健康检查函数
        health_check() {
            while true; do
                sleep 15
                local running=0
                local total=3
                
                echo \"[\$(date '+%H:%M:%S')] 服务健康检查...\"
                
                # 检查每个服务
                if [ \$COMM_PID -gt 0 ] && kill -0 \$COMM_PID 2>/dev/null; then
                    echo \"  ✓ 通信桥接运行正常\"
                    running=\$((running + 1))
                else
                    echo \"  ✗ 通信桥接已停止\"
                fi
                
                if [ \$POSE_PID -gt 0 ] && kill -0 \$POSE_PID 2>/dev/null; then
                    echo \"  ✓ 位姿估计运行正常\"
                    running=\$((running + 1))
                else
                    echo \"  ✗ 位姿估计已停止\"
                fi
                
                if [ \$YOLO_PID -gt 0 ] && kill -0 \$YOLO_PID 2>/dev/null; then
                    echo \"  ✓ 目标检测运行正常\"
                    running=\$((running + 1))
                else
                    echo \"  ✗ 目标检测已停止\"
                fi
                
                echo \"  运行状态: \$running/\$total\"
                
                if [ \$running -eq 0 ]; then
                    echo \"所有服务已停止\"
                    break
                fi
                
                echo ''
            done
        }
        
        # 优雅关闭函数
        graceful_shutdown() {
            echo ''
            echo '收到停止信号，正在优雅关闭服务...'
            
            # 停止所有服务
            [ \$COMM_PID -gt 0 ] && kill -TERM \$COMM_PID 2>/dev/null && echo '停止通信桥接'
            [ \$POSE_PID -gt 0 ] && kill -TERM \$POSE_PID 2>/dev/null && echo '停止位姿估计'
            [ \$YOLO_PID -gt 0 ] && kill -TERM \$YOLO_PID 2>/dev/null && echo '停止目标检测'
            
            # 等待进程优雅退出
            sleep 3
            
            # 强制杀死仍在运行的进程
            [ \$COMM_PID -gt 0 ] && kill -9 \$COMM_PID 2>/dev/null || true
            [ \$POSE_PID -gt 0 ] && kill -9 \$POSE_PID 2>/dev/null || true
            [ \$YOLO_PID -gt 0 ] && kill -9 \$YOLO_PID 2>/dev/null || true
            
            echo '所有服务已停止'
            exit 0
        }
        
        # 设置信号处理
        trap graceful_shutdown INT TERM
        
        echo '服务监控已启动，按 Ctrl+C 停止所有服务'
        echo ''
        
        # 启动健康检查
        health_check
        
        exec bash
    " &
    
    sleep 5
    log_info "多服务集合启动完成"
}

# 示例：单独启动服务的函数
start_single_service_example() {
    local service_name=$1
    local command=$2
    local work_dir=${3:-~/workspace}
    
    log_info "启动单个服务: $service_name"
    
    gnome-terminal --tab --title="$service_name" -- bash -c "
        source ~/.bashrc
        cd $work_dir
        
        echo '启动 $service_name...'
        echo '工作目录: \$(pwd)'
        echo '命令: $command'
        echo ''
        
        # 创建启动包装器
        start_wrapper() {
            local retry_count=0
            local max_retries=3
            
            while [ \$retry_count -lt \$max_retries ]; do
                echo \"[\$(date '+%H:%M:%S')] 尝试启动 $service_name (第\$((retry_count + 1))次)...\"
                
                # 执行命令
                if $command; then
                    echo \"$service_name 正常退出\"
                    break
                else
                    retry_count=\$((retry_count + 1))
                    if [ \$retry_count -lt \$max_retries ]; then
                        echo \"启动失败，5秒后重试...\"
                        sleep 5
                    else
                        echo \"达到最大重试次数，放弃启动\"
                    fi
                fi
            done
        }
        
        # 信号处理
        cleanup() {
            echo ''
            echo '正在停止 $service_name...'
            # 这里可以添加特定的清理逻辑
            exit 0
        }
        trap cleanup INT TERM
        
        # 启动服务
        start_wrapper
        
        echo ''
        echo '按任意键继续...'
        read -n 1
        exec bash
    " &
    
    sleep 2
    log_info "$service_name 启动完成"
}

# 主函数示例
main() {
    echo "=== ROS服务启动优化示例 ==="
    echo ""
    echo "选择启动方式:"
    echo "1. 多服务合并启动（推荐）"
    echo "2. 单个服务启动示例"
    echo "3. 显示最佳实践说明"
    echo ""
    
    read -p "请选择 (1-3): " choice
    
    case $choice in
        1)
            start_multiple_ros_services
            ;;
        2)
            start_single_service_example "示例服务" "echo '这是一个示例服务'; sleep 30"
            ;;
        3)
            show_best_practices
            ;;
        *)
            echo "无效选择"
            exit 1
            ;;
    esac
}

# 最佳实践说明
show_best_practices() {
    echo ""
    echo "=== ROS节点启动最佳实践 ==="
    echo ""
    echo "1. 后台进程管理:"
    echo "   - 使用 'nohup command &' 启动后台进程"
    echo "   - 使用 'disown' 让进程脱离shell会话"
    echo "   - 记录每个服务的PID用于后续管理"
    echo ""
    echo "2. 进程监控:"
    echo "   - 使用 'kill -0 \$PID' 检查进程是否存在"
    echo "   - 定期进行健康检查"
    echo "   - 记录日志到文件便于调试"
    echo ""
    echo "3. 错误处理:"
    echo "   - 设置trap处理中断信号"
    echo "   - 实现优雅关闭机制"
    echo "   - 添加重试逻辑"
    echo ""
    echo "4. 日志管理:"
    echo "   - 将输出重定向到日志文件"
    echo "   - 使用时间戳记录事件"
    echo "   - 分离不同服务的日志"
    echo ""
    echo "5. 资源清理:"
    echo "   - 确保所有子进程都能被正确清理"
    echo "   - 删除临时文件"
    echo "   - 释放端口和资源"
    echo ""
    echo "示例脚本模板:"
    echo ""
    cat << 'EOF'
# 后台启动ROS节点的模板
start_ros_node() {
    local node_name=$1
    local command=$2
    
    # 启动后台进程
    nohup bash -c "
        source ~/.bashrc
        $command
    " > /tmp/${node_name}.log 2>&1 &
    
    local pid=$!
    echo "Started $node_name with PID: $pid"
    
    # 验证启动
    sleep 2
    if kill -0 $pid 2>/dev/null; then
        echo "$node_name started successfully"
        echo $pid > /tmp/${node_name}.pid
    else
        echo "$node_name failed to start"
        return 1
    fi
}

# 使用示例
start_ros_node "my_node" "roslaunch my_package my_launch_file.launch"
EOF
    echo ""
}

# 运行主函数
if [ "${BASH_SOURCE[0]}" = "${0}" ]; then
    main "$@"
fi
