#!/bin/bash
# step-1: install ros1-noetic
echo "[INFO] 启动后台终端，自动检测并关闭Gazebo进程..."
touch /tmp/gazebo_kill_flag
gnome-terminal -- bash -c '
while true; do
    if pgrep -x "gzserver" > /dev/null || pgrep -x "gazebo" > /dev/null; then
        echo "检测到Gazebo正在运行，正在关闭..."
        pkill -x gzserver
        pkill -x gzclient
        pkill -x gazebo
    fi
    sleep 2
done
' &


echo "[INFO] 开始安装ROS Noetic..."
echo -e "1\n1\n2\n3\n1" | wget http://fishros.com/install -O fishros && bash fishros
echo "[INFO] ROS Noetic安装完成。"

# step-2: install dependencies
echo "[INFO] 开始安装Python及系统依赖..."
pip install --upgrade setuptools
echo "[INFO] setuptools升级完成。"
python -m pip install --upgrade pip
echo "[INFO] pip升级完成。"
sudo apt install ninja-build exiftool ninja-build protobuf-compiler libeigen3-dev genromfs xmlstarlet libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev python3-pip gawk
echo "[INFO] 系统依赖安装完成。"
pip3 install pandas jinja2 pyserial cerberus pyulog==0.7.0 numpy toml pyquaternion empy pyyaml
echo "[INFO] 第一批Python依赖安装完成。"
pip3 install packaging numpy empy toml pyyaml jinja2 pyargparse kconfiglib jsonschema future
echo "[INFO] 第二批Python依赖安装完成。"
echo "[INFO] 安装特定版本importlib_metadata...（官方文档有误，需手动指定版本）"
pip3 install importlib_metadata==4.13.0
echo "[INFO] importlib_metadata安装完成。"

# step-3: install gazebo
echo "[INFO] 开始安装Gazebo及相关依赖..."
mkdir -p ~/catkin_ws/src
mkdir -p ~/catkin_ws/scripts
echo "[INFO] 初始化catkin工作空间..."
cd ~/catkin_ws && catkin init
echo "[INFO] 第一次catkin build..."
catkin build
echo "[INFO] 回到主目录..."
cd ~
echo "[INFO] 移除旧版gazebo及相关包..."
sudo apt-get remove gazebo* -y
sudo apt-get remove libgazebo* -y
sudo apt-get remove ros-noetic-gazebo* -y
echo "[INFO] 安装Gazebo相关ROS消息包..."
sudo apt-get install ros-noetic-moveit-msgs ros-noetic-object-recognition-msgs ros-noetic-octomap-msgs ros-noetic-camera-info-manager  ros-noetic-control-toolbox ros-noetic-polled-camera ros-noetic-controller-manager ros-noetic-transmission-interface ros-noetic-joint-limits-interface
echo "[INFO] 安装Gazebo主程序及开发包..."
echo "install additional gazebo packages,for the guidance is wrong"
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control gazebo11 libgazebo11-dev -y
echo "[INFO] 复制gazebo_ros_pkgs到catkin_ws/src..."
cd ~/catkin_ws
cp -r ~/XTDrone/sitl_config/gazebo_ros_pkgs src/
echo "[INFO] 清理catkin工作空间..."
catkin clean -y
echo "[INFO] 第二次catkin build..."
catkin build 
echo "[INFO] 解压模型文件到~/models..."
unzip models.zip -d ~/models
echo "[INFO] 移动模型到~/.gazebo/models..."
mv ~/models ~/.gazebo/models

# step-4: install mavros
echo "[INFO] 开始安装MAVROS..."
echo "if you have a proxy, please set it before installing mavros"
echo "export http_proxy=http://your_proxy_server:port"
echo "export https_proxy=http://your_proxy_server:port"
echo "If you don't have a proxy, please ignore this message."
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras  
echo "[INFO] 下载geographiclib数据集安装脚本..."
wget https://gitee.com/robin_shaun/XTDrone/raw/master/sitl_config/mavros/install_geographiclib_datasets.sh
echo "[INFO] 添加执行权限..."
sudo chmod a+x ./install_geographiclib_datasets.sh
echo "[INFO] 安装geographiclib数据集..."
sudo -E ./install_geographiclib_datasets.sh

# step-5: install px4
echo "[INFO] 开始安装PX4 Firmware..."
echo "move the files to the correct directory,for the guidance is wrong"
unzip PX4_Firmware.zip -d ~/PX4_Firmware
echo "[INFO] 移动PX4_Firmware_13内容到~/PX4_Firmware..."
mv ~/PX4_Firmware/PX4_Firmware_13/* ~/PX4_Firmware
rm -rf ~/PX4_Firmware/PX4_Firmware_13

echo "[INFO] 清理PX4编译缓存..."
cd ~/PX4_Firmware
make clean
echo "[INFO] 编译PX4 SITL Gazebo..."
make px4_sitl_default gazebo

echo "[INFO] 设置环境变量..."
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "source ~/PX4_Firmware/Tools/setup_gazebo.bash ~/PX4_Firmware/ ~/PX4_Firmware/build/px4_sitl_default" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4_Firmware/Tools/sitl_gazebo" >> ~/.bashrc

# step-6 :install qgroundcontrol
echo "[INFO] 安装QGroundControl..."
mv QGroundControl.AppImage ~/QGroundControl.AppImage

# step-7: install xtdrone
echo "[INFO] 克隆XTDrone仓库..."
git clone https://gitee.com/robin_shaun/XTDrone.git
echo "[INFO] 切换到XTDrone目录..."
cd ~/XTDrone
echo "[INFO] 切换到1_13_2分支..."
git checkout 1_13_2
echo "[INFO] 初始化子模块..."
git submodule update --init --recursive
echo "[INFO] 拷贝启动脚本..."
cp sitl_config/init.d-posix/* ~/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/
echo "[INFO] 拷贝launch文件..."
cp -r sitl_config/launch/* ~/PX4_Firmware/launch/
echo "[INFO] 拷贝世界文件..."
cp sitl_config/worlds/* ~/PX4_Firmware/Tools/sitl_gazebo/worlds/
echo "[INFO] 拷贝云台插件..."
cp sitl_config/gazebo_plugin/gimbal_controller/gazebo_gimbal_controller_plugin.cpp ~/PX4_Firmware/Tools/sitl_gazebo/src
cp sitl_config/gazebo_plugin/gimbal_controller/gazebo_gimbal_controller_plugin.hh ~/PX4_Firmware/Tools/sitl_gazebo/include
echo "[INFO] 拷贝风场插件..."
cp sitl_config/gazebo_plugin/wind_plugin/gazebo_ros_wind_plugin_xtdrone.cpp ~/PX4_Firmware/Tools/sitl_gazebo/src
cp sitl_config/gazebo_plugin/wind_plugin/gazebo_ros_wind_plugin_xtdrone.h ~/PX4_Firmware/Tools/sitl_gazebo/include
echo "[INFO] 替换CMakeLists.txt..."
cp sitl_config/CMakeLists.txt ~/PX4_Firmware/Tools/sitl_gazebo
echo "[INFO] 拷贝模型文件..."
cp -r sitl_config/models/* ~/PX4_Firmware/Tools/sitl_gazebo/models/ 
echo "[INFO] 删除旧模型..."
cd ~/.gazebo/models/
rm -r stereo_camera/ 3d_lidar/ 3d_gpu_lidar/ hokuyo_lidar/
echo "[INFO] 进入PX4_Firmware目录..."
cd ~/PX4_Firmware
echo "[INFO] 再次清理PX4编译缓存..."
make clean
echo "[INFO] 再次编译PX4 SITL Gazebo..."
make px4_sitl_default gazebo
echo "[INFO] 关闭后台Gazebo检测进程..."
rm -f /tmp/gazebo_kill_flag

# step-8: test the installation
echo "[INFO] 启动仿真测试终端1：PX4仿真..."
gnome-terminal -- bash -c "cd ~/PX4_Firmware && roslaunch px4 indoor1.launch"
echo "[INFO] 启动仿真测试终端2：通信节点..."
gnome-terminal -- bash -c "cd ~/XTDrone/communication/ && python3 multirotor_communication.py iris 0"
echo "[INFO] 启动仿真测试终端3：键盘控制..."
gnome-terminal -- bash -c "cd ~/XTDrone/control/keyboard && python3 multirotor_keyboard_control.py iris 1 vel"
