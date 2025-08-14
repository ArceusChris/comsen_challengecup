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

### 
