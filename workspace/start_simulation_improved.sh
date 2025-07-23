#!/bin/bash

# 改进的仿真启动脚本 - 解决spawn_model超时问题
# 基于原始zhihang2025.launch优化

# 设置错误处理
set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# 1. 环境准备
prepare_environment() {
    log_step "准备仿真环境..."
    
    # 清理旧进程
    pkill -f gazebo || true
    pkill -f px4 || true
    pkill -f roscore || true
    sleep 3
    
    # 设置环境变量
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash
    
    cd /root/PX4_Firmware
    source Tools/setup_gazebo.bash . build/px4_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:.
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:./Tools/sitl_gazebo
    
    log_info "环境准备完成"
}

# 2. 启动ROS Master
start_ros_master() {
    log_step "启动ROS Master..."
    
    roscore &
    ROS_PID=$!
    sleep 5
    
    # 检查ROS Master是否启动成功
    if ! rostopic list &>/dev/null; then
        log_error "ROS Master启动失败"
        exit 1
    fi
    
    log_info "ROS Master启动成功 (PID: $ROS_PID)"
}

# 3. 启动Gazebo仿真环境
start_gazebo() {
    log_step "启动Gazebo仿真环境..."
    
    # 设置世界文件路径
    WORLD_FILE="/root/PX4_Firmware/Tools/sitl_gazebo/worlds/zhihang2025.world"
    
    if [ ! -f "$WORLD_FILE" ]; then
        log_warning "世界文件不存在，使用默认empty世界"
        WORLD_FILE="worlds/empty.world"
    fi
    
    # 启动Gazebo
    roslaunch gazebo_ros empty_world.launch \
        world_name:="$WORLD_FILE" \
        paused:=false \
        use_sim_time:=true \
        gui:=true \
        headless:=false \
        debug:=false \
        physics:=ode \
        verbose:=false \
        respawn_gazebo:=true &
    
    GAZEBO_PID=$!
    
    # 等待Gazebo完全启动
    log_info "等待Gazebo启动..."
    sleep 15
    
    # 检查Gazebo是否启动成功
    local timeout=30
    local count=0
    while [ $count -lt $timeout ]; do
        if rosservice list | grep -q "/gazebo/spawn_sdf_model"; then
            log_info "Gazebo启动成功 (PID: $GAZEBO_PID)"
            return 0
        fi
        sleep 1
        ((count++))
    done
    
    log_error "Gazebo启动超时"
    exit 1
}

# 4. 启动PX4 SITL
start_px4_sitl() {
    local vehicle_id=$1
    local vehicle_type=$2
    local x=$3
    local y=$4
    local z=$5
    
    log_step "启动PX4 SITL - ${vehicle_type}_${vehicle_id}..."
    
    cd /root/PX4_Firmware
    
    # 启动PX4 SITL实例
    ./build/px4_sitl_default/bin/px4 \
        -d build/px4_sitl_default/etc \
        -s etc/init.d-posix/rcS \
        -i $vehicle_id \
        -w sitl_${vehicle_type}_${vehicle_id} &
    
    PX4_PID=$!
    sleep 10
    
    log_info "PX4 SITL启动成功 - ${vehicle_type}_${vehicle_id} (PID: $PX4_PID)"
}

# 5. 生成飞行器模型（带重试机制）
spawn_vehicle_with_retry() {
    local vehicle_type=$1
    local vehicle_id=$2
    local x=$3
    local y=$4
    local z=$5
    local max_retries=3
    
    log_step "生成飞行器模型 - ${vehicle_type}_${vehicle_id}..."
    
    # 准备模型参数
    local model_file="/root/PX4_Firmware/Tools/sitl_gazebo/models/${vehicle_type}/${vehicle_type}.sdf"
    
    if [ ! -f "$model_file" ]; then
        log_error "模型文件不存在: $model_file"
        return 1
    fi
    
    # 设置模型参数到ROS参数服务器
    rosparam set /${vehicle_type}_${vehicle_id}/model_description "$(cat $model_file)"
    
    # 多次尝试生成模型
    for ((retry=1; retry<=max_retries; retry++)); do
        log_info "尝试生成模型 (第${retry}次)..."
        
        # 使用超时命令防止卡死
        if timeout 60 rosrun gazebo_ros spawn_model \
            -sdf \
            -param /${vehicle_type}_${vehicle_id}/model_description \
            -model ${vehicle_type}_${vehicle_id} \
            -x $x -y $y -z $z \
            -R 0 -P 0 -Y 0; then
            
            log_info "模型生成成功 - ${vehicle_type}_${vehicle_id}"
            return 0
        else
            log_warning "模型生成失败 (第${retry}次尝试)"
            if [ $retry -lt $max_retries ]; then
                log_info "等待10秒后重试..."
                sleep 10
            fi
        fi
    done
    
    log_error "模型生成最终失败 - ${vehicle_type}_${vehicle_id}"
    return 1
}

# 6. 启动MAVROS
start_mavros() {
    local vehicle_type=$1
    local vehicle_id=$2
    local fcu_port=$3
    
    log_step "启动MAVROS - ${vehicle_type}_${vehicle_id}..."
    
    local fcu_url="udp://:$((24540 + vehicle_id))@localhost:$((34580 + vehicle_id))"
    
    ROS_NAMESPACE=${vehicle_type}_${vehicle_id} roslaunch mavros px4.launch \
        fcu_url:="$fcu_url" \
        gcs_url:="" \
        tgt_system:=$((1 + vehicle_id)) \
        tgt_component:=1 &
    
    MAVROS_PID=$!
    sleep 5
    
    log_info "MAVROS启动成功 - ${vehicle_type}_${vehicle_id} (PID: $MAVROS_PID)"
}

# 7. 主启动流程
main() {
    log_info "开始启动多无人机仿真系统..."
    
    # 环境准备
    prepare_environment
    
    # 启动基础服务
    start_ros_master
    start_gazebo
    
    # 启动第一架无人机 (iris_0)
    log_info "=== 启动第一架无人机 iris_0 ==="
    start_px4_sitl 0 "iris" 2.5 2.7 1.3
    spawn_vehicle_with_retry "iris" 0 2.5 2.7 1.3
    start_mavros "iris" 0 24540
    
    # 等待第一架无人机稳定
    sleep 10
    
    # 启动第二架无人机 (standard_vtol_0)
    log_info "=== 启动第二架无人机 standard_vtol_0 ==="
    start_px4_sitl 1 "standard_vtol" 2.3 0.4 1.3
    spawn_vehicle_with_retry "standard_vtol" 0 2.3 0.4 1.3
    start_mavros "standard_vtol" 0 24541
    
    log_info "多无人机仿真系统启动完成！"
    
    # 显示状态信息
    echo ""
    log_info "系统状态："
    echo "- ROS Master: 运行中"
    echo "- Gazebo: 运行中"
    echo "- iris_0: 位置(2.5, 2.7, 1.3)"
    echo "- standard_vtol_0: 位置(2.3, 0.4, 1.3)"
    
    echo ""
    log_info "可用的ROS话题："
    rostopic list | grep -E "(iris_0|standard_vtol_0)" | head -10
    
    echo ""
    log_info "仿真系统已准备就绪！"
    
    # 保持脚本运行
    log_info "按Ctrl+C退出..."
    wait
}

# 信号处理函数
cleanup() {
    log_info "正在关闭仿真系统..."
    pkill -f gazebo || true
    pkill -f px4 || true
    pkill -f mavros || true
    pkill -f roscore || true
    exit 0
}

trap cleanup SIGINT SIGTERM

# 执行主函数
main "$@"
