# 无人机双机协同自主搜救任务 Docker环境
FROM ubuntu:20.04

# 设置环境变量避免交互式安装
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai

# 设置时区
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# 预配置键盘布局以避免交互式选择
RUN echo 'keyboard-configuration keyboard-configuration/layout select English (US)' | debconf-set-selections && \
    echo 'keyboard-configuration keyboard-configuration/layoutcode select us' | debconf-set-selections && \
    echo 'keyboard-configuration keyboard-configuration/model select Generic 105-key (Intl) PC' | debconf-set-selections && \
    echo 'keyboard-configuration keyboard-configuration/modelcode select pc105' | debconf-set-selections && \
    echo 'keyboard-configuration keyboard-configuration/variant select English (US)' | debconf-set-selections && \
    echo 'keyboard-configuration keyboard-configuration/variantcode select ' | debconf-set-selections

# 更新源并安装基础工具
RUN apt-get update && apt-get install -y \
    debconf-utils \
    curl \
    wget \
    git \
    vim \
    sudo \
    software-properties-common \
    lsb-release \
    gnupg2 \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# 添加Gazebo源
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt update \ 
    && apt install wget python3-yaml -y  \
    # 预配置可能的交互式包
    && echo 'tzdata tzdata/Areas select Asia' | debconf-set-selections \
    && echo 'tzdata tzdata/Zones/Asia select Shanghai' | debconf-set-selections \
    # 安装ROS noetic
    && echo "chooses:\n" > fish_install.yaml \
    && echo "- {choose: 1, desc: '一键安装:ROS(支持ROS和ROS2,树莓派Jetson)'}\n" >> fish_install.yaml \
    && echo "- {choose: 1, desc: 更换源继续安装}\n" >> fish_install.yaml \
    && echo "- {choose: 2, desc: 清理三方源}\n" >> fish_install.yaml \
    && echo "- {choose: 3, desc: noetic(ROS1)}\n" >> fish_install.yaml \
    && echo "- {choose: 1, desc: noetic(ROS1)桌面版}\n" >> fish_install.yaml \
    && wget http://fishros.com/install  -O fishros && /bin/bash fishros \
    # 进行最后的清理
    && rm -rf /var/lib/apt/lists/*  /tmp/* /var/tmp/* \
    && apt-get clean && apt autoclean 

# 安装PX4依赖
RUN apt-get update && apt-get install -y \
    ninja-build \
    exiftool \
    protobuf-compiler \
    libeigen3-dev \
    genromfs \
    xmlstarlet \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    python3-pip \
    gawk \
    python3-dev \
    python3-setuptools \
    # 添加X11支持包（可选，用于GUI）
    x11-apps \
    xauth \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

# 安装Python依赖
RUN pip3 install --upgrade pip setuptools wheel
RUN pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple
# 修复importlib_metadata版本兼容性问题
RUN pip3 install "importlib_metadata<5.0"
RUN pip3 install \
    pandas \
    jinja2 \
    pyserial \
    cerberus \
    pyulog \
    numpy \
    toml \
    pyquaternion \
    empy \
    pyyaml \
    packaging \
    pyargparse \
    kconfiglib \
    jsonschema \
    future \
    pyros-genmsg

# 安装Gazebo 11
RUN apt-get update && apt-get install -y \
    gazebo11 \
    libgazebo11-dev \
    && rm -rf /var/lib/apt/lists/*

# 安装MAVROS
RUN apt-get update && apt-get install -y \
    ros-noetic-mavros \
    ros-noetic-mavros-extras \
    ros-noetic-moveit-msgs \
    ros-noetic-object-recognition-msgs \
    ros-noetic-octomap-msgs \
    ros-noetic-camera-info-manager \
    ros-noetic-control-toolbox \
    ros-noetic-polled-camera \
    ros-noetic-controller-manager \
    ros-noetic-transmission-interface \
    ros-noetic-joint-limits-interface \
    && rm -rf /var/lib/apt/lists/*

# 安装GeographicLib数据集
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    && chmod +x install_geographiclib_datasets.sh \
    && ./install_geographiclib_datasets.sh \
    && rm install_geographiclib_datasets.sh

# 清理apt缓存
RUN apt-get autoremove -y && apt-get autoclean && rm -rf /var/lib/apt/lists/*

# 创建工作目录
WORKDIR /root

# 创建catkin工作空间
RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin init && catkin build"

# 下载并编译PX4
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive PX4_Firmware
WORKDIR /root/PX4_Firmware
RUN git checkout v1.13.2
# 更新所有子模块到正确版本避免交互式提示
RUN git submodule sync --recursive && git submodule update --init --recursive
# 只编译PX4 SITL，不启动Gazebo GUI
RUN make px4_sitl_default

# 下载XTDrone
WORKDIR /root
RUN git clone https://gitee.com/robin_shaun/XTDrone.git
WORKDIR /root/XTDrone
RUN git checkout 1_13_2
# 确保所有子模块正确更新
RUN git submodule sync --recursive && git submodule update --init --recursive

# 配置XTDrone
RUN cp sitl_config/init.d-posix/* /root/PX4_Firmware/ROMFS/px4fmu_common/init.d-posix/ && \
    cp -r sitl_config/launch/* /root/PX4_Firmware/launch/ && \
    cp sitl_config/worlds/* /root/PX4_Firmware/Tools/sitl_gazebo/worlds/ && \
    cp sitl_config/gazebo_plugin/gimbal_controller/gazebo_gimbal_controller_plugin.cpp /root/PX4_Firmware/Tools/sitl_gazebo/src/ && \
    cp sitl_config/gazebo_plugin/gimbal_controller/gazebo_gimbal_controller_plugin.hh /root/PX4_Firmware/Tools/sitl_gazebo/include/ && \
    cp sitl_config/gazebo_plugin/wind_plugin/gazebo_ros_wind_plugin_xtdrone.cpp /root/PX4_Firmware/Tools/sitl_gazebo/src/ && \
    cp sitl_config/gazebo_plugin/wind_plugin/gazebo_ros_wind_plugin_xtdrone.h /root/PX4_Firmware/Tools/sitl_gazebo/include/ && \
    cp sitl_config/CMakeLists.txt /root/PX4_Firmware/Tools/sitl_gazebo/ && \
    cp -r sitl_config/models/* /root/PX4_Firmware/Tools/sitl_gazebo/models/

# 创建.gazebo目录并复制模型
RUN mkdir -p /root/.gazebo/models

# 重新编译PX4
WORKDIR /root/PX4_Firmware
# 再次确保子模块同步，避免配置文件更改后的版本冲突
RUN git submodule sync --recursive && git submodule update --init --recursive
# 清理并重新编译，只编译不启动Gazebo GUI
RUN make clean && make px4_sitl_default

# 设置环境变量
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "source /root/PX4_Firmware/Tools/setup_gazebo.bash /root/PX4_Firmware/ /root/PX4_Firmware/build/px4_sitl_default" >> /root/.bashrc && \
    echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/root/PX4_Firmware" >> /root/.bashrc && \
    echo "export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:/root/PX4_Firmware/Tools/sitl_gazebo" >> /root/.bashrc

# 创建启动脚本
RUN echo '#!/bin/bash' > /root/start_px4_gazebo.sh && \
    echo 'cd /root/PX4_Firmware' >> /root/start_px4_gazebo.sh && \
    echo 'source /opt/ros/noetic/setup.bash' >> /root/start_px4_gazebo.sh && \
    echo 'source /root/catkin_ws/devel/setup.bash' >> /root/start_px4_gazebo.sh && \
    echo 'source Tools/setup_gazebo.bash . build/px4_sitl_default' >> /root/start_px4_gazebo.sh && \
    echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:.' >> /root/start_px4_gazebo.sh && \
    echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:./Tools/sitl_gazebo' >> /root/start_px4_gazebo.sh && \
    echo 'make px4_sitl_default gazebo' >> /root/start_px4_gazebo.sh && \
    chmod +x /root/start_px4_gazebo.sh

# 创建无头模式启动脚本
RUN echo '#!/bin/bash' > /root/start_px4_headless.sh && \
    echo 'cd /root/PX4_Firmware' >> /root/start_px4_headless.sh && \
    echo 'source /opt/ros/noetic/setup.bash' >> /root/start_px4_headless.sh && \
    echo 'source /root/catkin_ws/devel/setup.bash' >> /root/start_px4_headless.sh && \
    echo 'source Tools/setup_gazebo.bash . build/px4_sitl_default' >> /root/start_px4_headless.sh && \
    echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:.' >> /root/start_px4_headless.sh && \
    echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:./Tools/sitl_gazebo' >> /root/start_px4_headless.sh && \
    echo 'HEADLESS=1 make px4_sitl_default gazebo' >> /root/start_px4_headless.sh && \
    chmod +x /root/start_px4_headless.sh

# 设置工作目录
WORKDIR /root

# 暴露端口
EXPOSE 11345 14556 14557

# 启动脚本
CMD ["/bin/bash"]
