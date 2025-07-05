#!/bin/bash

# 启动仿真环境的脚本

echo "正在启动无人机双机协同自主搜救任务仿真环境..."

# 设置环境变量
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
source /root/PX4_Firmware/Tools/setup_gazebo.bash /root/PX4_Firmware/ /root/PX4_Firmware/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/PX4_Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/PX4_Firmware/Tools/sitl_gazebo

# 检查是否有比赛文件
if [ ! -f "/root/XTDrone/zhihang2025.launch" ]; then
    echo "警告：比赛文件未找到，请确保已将比赛提供的文件放置在正确位置"
    echo "比赛文件应该包括："
    echo "1. 模型文件：standard_vtol, iris_zhihang, target_green, landing2"
    echo "2. 人员模型：person_standing"
    echo "3. Launch文件：zhihang2025.launch"
    echo "4. World文件：zhihang2025.world"
    echo "5. 位姿真值文件：get_local_pose.py"
    echo "6. 其他文件：zhihang2025文件夹"
    echo ""
    echo "请从比赛组织方获取这些文件后再运行仿真"
    echo ""
fi

echo "环境已准备完成！"
echo ""
echo "使用以下命令启动仿真（需要在容器内运行）："
echo "1. 启动仿真程序："
echo "   roslaunch px4 zhihang2025.launch"
echo ""
echo "2. 在新终端中运行通信脚本："
echo "   # 垂起无人机"
echo "   cd ~/XTDrone/communication/"
echo "   python3 vtol_communication.py standard_vtol 0"
echo ""
echo "   # 旋翼无人机"
echo "   cd ~/XTDrone/communication/"
echo "   python3 multirotor_communication.py iris 0"
echo ""
echo "3. 开启gazebo位姿真值："
echo "   # 垂起无人机"
echo "   cd ~/XTDrone/sensing/pose_ground_truth/"
echo "   python3 get_local_pose.py standard_vtol 1"
echo ""
echo "   # 旋翼无人机"
echo "   cd ~/XTDrone/sensing/pose_ground_truth/"
echo "   python3 get_local_pose.py iris 1"
echo ""
echo "4. 发布GPS引导点："
echo "   cd ~/XTDrone/zhihang2025"
echo "   python3 Pub_first_point.py"
echo ""
echo "5. 发布居民区中心坐标："
echo "   cd ~/XTDrone/zhihang2025"
echo "   python3 Pub_downtown.py"
echo ""
echo "6. 开启待救援目标移动："
echo "   cd ~/XTDrone/zhihang2025"
echo "   python3 zhihang_control_targets.py"
echo ""
echo "7. 记录数据："
echo "   cd ~/XTDrone/zhihang2025"
echo "   rosbag record -O score1 /standard_vtol_0/mavros/state /iris_0/mavros/state /gazebo/model_states /xtdrone/standard_vtol_0/cmd /xtdrone/iris_0/cmd /zhihang/first_point /zhihang2025/first_man/pose /zhihang2025/second_man/pose /zhihang2025/third_man/pose /zhihang2025/iris_healthy_man/pose /zhihang2025/iris_bad_man/pose /zhihang/downtown"

# 保持容器运行
exec /bin/bash
