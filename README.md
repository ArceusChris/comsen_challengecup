# 双无人机自主搜救系统使用说明

## 项目结构

```Markdown
comsen_challengecup
├── README.md *使用说明文件*
├── start_competition.sh *仿真环境启动脚本*
├── start_yolo12.sh *外部YOLO12推理程序启动脚本*
├── workspace 
│   ├── CMakeLists.txt
│   ├── main.launch *主启动文件*
│   ├── package.xml
│   └── src
│       ├── CMakeLists.txt
│       ├── iris_control *四旋翼控制包*
│       ├── landing_detection *降落平台检测包*
│       ├── perception *YOLO11识别包（备用）*
│       ├── perception_yolov12 *YOLO12识别包*
│       ├── tf_transform *坐标变换发布包*
│       └── vtol_control *垂起固定翼控制包*
│   
└── yolo_models *YOLO目标检测模型*
```

## 环境配置步骤

### ROS依赖配置

```bash
conda deactivate
pip install -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple scipy opencv-python 
```

### YOLOv12环境配置

首先使用```nvcc -V```  查看本机cuda版本（11或12）
接着依次运行下面的命令

```bash
cd ~/
git clone https://github.com/sunsmarterjie/yolov12.git
cd yolov12
conda create -n yolov12 python=3.11
conda activate yolov12
pip config set global.index-url https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple
wget https://github.com/Dao-AILab/flash-attention/releases/download/v2.7.3/flash_attn-2.7.3+cu11torch2.2cxx11abiFALSE-cp311-cp311-linux_x86_64.whl
#  若cuda版本为12则将链接替换为 https://github.com/Dao-AILab/flash-attention/releases/download/v2.7.3/flash_attn-2.7.3+cu12torch2.2cxx11abiFALSE-cp311-cp311-linux_x86_64.whl 同时修改requirements.txt中的文件名
pip install -r requirements.txt
pip install -e .
```

## 系统启动步骤

将```comsen_challengecup```目录放至主目录下,接着分别在三个终端中依次运行下列命令

```bash
cd ~/comsen_challengecup
chmod +x ./start_competition.sh
conda deactivate
./start_competition.sh
```

等待上述命令输出启动成功的标志后再执行接下来的命令

```bash
cd ~/comsen_challengecup
chmod +x ./start_yolo12.sh
conda activate yolov12
./start_yolo12.sh
```

```bash
cd ~/comsen_challengecup/workspace
catkin build && source devel/setup.bash
roslaunch main.launch
```
