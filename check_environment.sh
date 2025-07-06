#!/bin/bash

# 无人机双机协同自主搜救任务 - 环境检查脚本

echo "🔍 检查无人机仿真环境..."
echo "================================"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

success_count=0
total_count=0

check_item() {
    local item_name="$1"
    local check_command="$2"
    local path="$3"
    
    total_count=$((total_count + 1))
    
    printf "%-40s" "$item_name"
    
    if eval "$check_command" &>/dev/null; then
        echo -e "${GREEN}✓ 通过${NC}"
        success_count=$((success_count + 1))
    else
        echo -e "${RED}✗ 失败${NC}"
        if [ -n "$path" ]; then
            echo "   路径: $path"
        fi
    fi
}

echo "基础环境检查："
echo "--------------------------------"
check_item "ROS环境" "command -v roscore"
check_item "Gazebo" "command -v gazebo"
check_item "Python3" "command -v python3"
check_item "终端环境" "command -v gnome-terminal"

echo ""
echo "关键目录检查："
echo "--------------------------------"
check_item "PX4_Firmware" "[ -d ~/PX4_Firmware ]" "~/PX4_Firmware"
check_item "XTDrone" "[ -d ~/XTDrone ]" "~/XTDrone"
check_item "通信脚本目录" "[ -d ~/XTDrone/communication ]" "~/XTDrone/communication"
check_item "位姿真值目录" "[ -d ~/XTDrone/sensing/pose_ground_truth ]" "~/XTDrone/sensing/pose_ground_truth"
check_item "zhihang2025目录" "[ -d ~/XTDrone/zhihang2025 ]" "~/XTDrone/zhihang2025"

echo ""
echo "关键文件检查："
echo "--------------------------------"
check_item "Launch文件" "[ -f ~/PX4_Firmware/launch/zhihang2025.launch ]" "~/PX4_Firmware/launch/zhihang2025.launch"
check_item "World文件" "[ -f ~/PX4_Firmware/Tools/sitl_gazebo/worlds/zhihang2025.world ]" "~/PX4_Firmware/Tools/sitl_gazebo/worlds/zhihang2025.world"
check_item "位姿真值脚本" "[ -f ~/XTDrone/sensing/pose_ground_truth/get_local_pose.py ]" "~/XTDrone/sensing/pose_ground_truth/get_local_pose.py"
check_item "垂起通信脚本" "[ -f ~/XTDrone/communication/vtol_communication.py ]" "~/XTDrone/communication/vtol_communication.py"
check_item "旋翼通信脚本" "[ -f ~/XTDrone/communication/multirotor_communication.py ]" "~/XTDrone/communication/multirotor_communication.py"

echo ""
echo "任务相关文件检查："
echo "--------------------------------"
check_item "GPS发布脚本" "[ -f ~/XTDrone/zhihang2025/Pub_first_point.py ]" "~/XTDrone/zhihang2025/Pub_first_point.py"
check_item "居民区发布脚本" "[ -f ~/XTDrone/zhihang2025/Pub_downtown.py ]" "~/XTDrone/zhihang2025/Pub_downtown.py"
check_item "目标控制脚本" "[ -f ~/XTDrone/zhihang2025/zhihang_control_targets.py ]" "~/XTDrone/zhihang2025/zhihang_control_targets.py"

echo ""
echo "模型文件检查："
echo "--------------------------------"
check_item "垂起无人机模型" "[ -d ~/PX4_Firmware/Tools/sitl_gazebo/models/standard_vtol ]" "~/PX4_Firmware/Tools/sitl_gazebo/models/standard_vtol"
check_item "旋翼无人机模型" "[ -d ~/PX4_Firmware/Tools/sitl_gazebo/models/iris_zhihang ]" "~/PX4_Firmware/Tools/sitl_gazebo/models/iris_zhihang"
check_item "目标模型" "[ -d ~/PX4_Firmware/Tools/sitl_gazebo/models/target_green ]" "~/PX4_Firmware/Tools/sitl_gazebo/models/target_green"
check_item "着陆模型" "[ -d ~/PX4_Firmware/Tools/sitl_gazebo/models/landing2 ]" "~/PX4_Firmware/Tools/sitl_gazebo/models/landing2"
check_item "人员模型" "[ -d ~/.gazebo/models/person_standing ]" "~/.gazebo/models/person_standing"

echo ""
echo "================================"
echo "检查结果："
echo "  通过项目: $success_count/$total_count"

if [ $success_count -eq $total_count ]; then
    echo -e "${GREEN}🎉 所有检查项目都通过了！环境配置正确。${NC}"
    echo ""
    echo "下一步："
    echo "  1. 运行 ./auto_start_mission.sh 启动所有服务"
    echo "  2. 等待服务完全启动（约30秒）"
    echo "  3. 启动你的无人机控制程序"
    echo ""
    exit 0
else
    fail_count=$((total_count - success_count))
    echo -e "${RED}❌ 有 $fail_count 个项目检查失败！${NC}"
    echo ""
    echo "请检查并修复失败的项目，然后重新运行此脚本。"
    echo ""
    echo "常见问题解决方案："
    echo "  1. 确保按照赛题要求正确放置所有文件"
    echo "  2. 检查文件路径和权限"
    echo "  3. 确保ROS和Gazebo环境正确安装"
    echo "  4. 参考 AUTO_START_README.md 文档"
    echo ""
    exit 1
fi
