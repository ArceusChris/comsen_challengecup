#!/bin/bash

# Gazebo模型生成问题诊断和修复脚本
# 解决standard_vtol_0模型生成超时问题

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

# 1. 检查系统资源
check_system_resources() {
    log_step "检查系统资源..."
    
    # 检查内存使用
    mem_usage=$(free | grep Mem | awk '{printf("%.1f", $3/$2 * 100.0)}')
    log_info "内存使用率: ${mem_usage}%"
    
    # 检查磁盘空间
    disk_usage=$(df -h / | tail -1 | awk '{print $5}' | sed 's/%//')
    log_info "磁盘使用率: ${disk_usage}%"
    
    # 检查CPU负载
    cpu_load=$(uptime | awk -F'load average:' '{print $2}' | awk '{print $1}' | sed 's/,//')
    log_info "CPU负载: ${cpu_load}"
    
    if (( $(echo "$mem_usage > 90" | bc -l) )); then
        log_warning "内存使用率过高，可能影响仿真性能"
    fi
    
    if [ "$disk_usage" -gt 90 ]; then
        log_warning "磁盘空间不足，可能影响仿真运行"
    fi
}

# 2. 清理残留进程
cleanup_processes() {
    log_step "清理残留进程..."
    
    # 停止所有gazebo相关进程
    pkill -f gazebo || true
    pkill -f px4 || true
    pkill -f roscore || true
    pkill -f rosmaster || true
    pkill -f spawn_model || true
    
    sleep 3
    
    # 强制杀死顽固进程
    pkill -9 -f gazebo || true
    pkill -9 -f px4 || true
    
    log_info "进程清理完成"
}

# 3. 清理共享内存和临时文件
cleanup_temp_files() {
    log_step "清理临时文件和共享内存..."
    
    # 清理Gazebo临时文件
    rm -rf /tmp/.gazebo* || true
    rm -rf ~/.gazebo/log/* || true
    
    # 清理ROS日志
    rm -rf ~/.ros/log/* || true
    
    # 清理共享内存
    ipcs -m | awk '/^0x/ {print $2}' | xargs -r ipcrm -m || true
    
    log_info "临时文件清理完成"
}

# 4. 检查模型文件完整性
check_model_files() {
    log_step "检查模型文件完整性..."
    
    # 检查standard_vtol模型
    model_path="/root/PX4_Firmware/Tools/sitl_gazebo/models/standard_vtol"
    
    if [ ! -f "$model_path/standard_vtol.sdf" ]; then
        log_error "standard_vtol.sdf文件不存在"
        return 1
    fi
    
    if [ ! -f "$model_path/model.config" ]; then
        log_error "model.config文件不存在"
        return 1
    fi
    
    # 检查SDF文件格式
    if ! xmllint --noout "$model_path/standard_vtol.sdf" 2>/dev/null; then
        log_error "standard_vtol.sdf文件格式错误"
        return 1
    fi
    
    log_info "模型文件检查通过"
}

# 5. 设置环境变量
setup_environment() {
    log_step "设置环境变量..."
    
    # 设置Gazebo模型路径
    export GAZEBO_MODEL_PATH="/root/PX4_Firmware/Tools/sitl_gazebo/models:$GAZEBO_MODEL_PATH"
    export GAZEBO_PLUGIN_PATH="/root/PX4_Firmware/build/px4_sitl_default/build_gazebo:$GAZEBO_PLUGIN_PATH"
    
    # 设置ROS环境
    source /opt/ros/noetic/setup.bash
    source /root/catkin_ws/devel/setup.bash
    
    # 设置PX4环境
    cd /root/PX4_Firmware
    source Tools/setup_gazebo.bash . build/px4_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:.
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:./Tools/sitl_gazebo
    
    log_info "环境变量设置完成"
}

# 6. 修复spawn_model超时问题
fix_spawn_timeout() {
    log_step "修复spawn_model超时问题..."
    
    # 创建修复版本的launch文件
    cat > /tmp/test_spawn.launch << 'EOF'
<?xml version="1.0"?>
<launch>
    <!-- 测试spawn_model功能 -->
    <arg name="gui" default="true"/>
    <arg name="paused" default="false"/>
    
    <!-- 启动Gazebo -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="gui" value="$(arg gui)"/>
        <arg name="paused" value="$(arg paused)"/>
        <arg name="physics" value="ode"/>
        <arg name="verbose" value="true"/>
        <arg name="world_name" value="worlds/empty.world"/>
        <arg name="respawn_gazebo" value="true"/>
    </include>
    
    <!-- 生成模型参数 -->
    <param name="robot_description" 
           command="cat $(find px4)/Tools/sitl_gazebo/models/standard_vtol/standard_vtol.sdf"/>
    
    <!-- 延迟启动spawn_model -->
    <node name="spawn_vtol" 
          pkg="gazebo_ros" 
          type="spawn_model" 
          args="-sdf -param robot_description -model test_vtol -x 0 -y 0 -z 1"
          launch-prefix="bash -c 'sleep 10; $0 $@'"
          respawn="false"/>
</launch>
EOF
    
    log_info "创建测试launch文件: /tmp/test_spawn.launch"
}

# 7. 优化Gazebo配置
optimize_gazebo_config() {
    log_step "优化Gazebo配置..."
    
    # 创建优化的Gazebo配置
    mkdir -p ~/.gazebo
    cat > ~/.gazebo/gui.ini << 'EOF'
[geometry]
x=0
y=0
width=1024
height=768

[increments]
translation=1
rotation=1

[spacenav]
deadband_x=0.1
deadband_y=0.1
deadband_z=0.1
deadband_rx=0.1
deadband_ry=0.1
deadband_rz=0.1
topic=/spacenav/joy

[overlay]
anchor=top_right
height=240
material=Gazebo/LogoOverlay
width=240
x=10
y=10

[physics]
real_time_factor=1.0
max_step_size=0.001
real_time_update_rate=1000

[rendering]
shadows=false
EOF
    
    log_info "Gazebo配置优化完成"
}

# 8. 测试spawn功能
test_spawn_functionality() {
    log_step "测试spawn功能..."
    
    log_info "启动测试环境..."
    
    # 在后台启动roscore
    roscore &
    ROSCORE_PID=$!
    sleep 5
    
    # 启动基础Gazebo
    gazebo --verbose worlds/empty.world &
    GAZEBO_PID=$!
    sleep 10
    
    # 测试spawn服务
    if rosservice list | grep -q "/gazebo/spawn_sdf_model"; then
        log_info "Gazebo spawn服务可用"
        
        # 测试简单模型生成
        rosrun gazebo_ros spawn_model \
            -file /root/PX4_Firmware/Tools/sitl_gazebo/models/standard_vtol/standard_vtol.sdf \
            -sdf -model test_vtol -x 0 -y 0 -z 1 &
        
        sleep 10
        
        # 检查模型是否成功生成
        if rosservice call /gazebo/get_model_state "model_name: 'test_vtol'" 2>/dev/null; then
            log_info "模型生成测试成功"
        else
            log_warning "模型生成测试失败"
        fi
    else
        log_error "Gazebo spawn服务不可用"
    fi
    
    # 清理测试进程
    kill $GAZEBO_PID $ROSCORE_PID || true
    sleep 3
}

# 9. 主修复流程
main() {
    log_info "开始Gazebo模型生成问题修复..."
    
    check_system_resources
    cleanup_processes
    cleanup_temp_files
    check_model_files
    setup_environment
    fix_spawn_timeout
    optimize_gazebo_config
    
    log_info "修复完成！"
    
    echo ""
    log_info "建议的启动顺序："
    echo "1. 运行此脚本进行修复"
    echo "2. 等待10-15秒让系统稳定"
    echo "3. 重新启动仿真程序"
    echo "4. 如果仍有问题，可以尝试："
    echo "   - 增加spawn_model启动延迟"
    echo "   - 降低仿真复杂度"
    echo "   - 检查硬件资源是否充足"
}

# 执行主函数
main "$@"
