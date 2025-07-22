#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
点云可视化配置生成器
为RViz生成YOLO11目标检测点云的可视化配置
"""

import json

def generate_rviz_config():
    """生成RViz配置文件"""
    
    rviz_config = {
        "Panels": [
            {
                "Class": "rviz/Displays",
                "Help Height": 78,
                "Name": "Displays",
                "Property Tree Widget": {
                    "Expanded": [
                        "/Global Options1",
                        "/Red Points1",
                        "/Yellow Points1", 
                        "/White Points1"
                    ],
                    "Splitter Ratio": 0.5
                },
                "Tree Height": 565
            },
            {
                "Class": "rviz/Tool Properties",
                "Expanded": [
                    "/2D Pose Estimate1",
                    "/2D Nav Goal1"
                ],
                "Name": "Tool Properties",
                "Splitter Ratio": 0.588679
            }
        ],
        "Toolbars": "toolbars",
        "Visualization Manager": {
            "Class": "",
            "Displays": [
                {
                    "Alpha": 0.5,
                    "Cell Size": 1,
                    "Class": "rviz/Grid",
                    "Color": "160; 160; 164",
                    "Enabled": True,
                    "Line Style": {
                        "Line Width": 0.029999999329447746,
                        "Value": "Lines"
                    },
                    "Name": "Grid",
                    "Normal Cell Count": 0,
                    "Offset": {
                        "X": 0,
                        "Y": 0,
                        "Z": 0
                    },
                    "Plane": "XY",
                    "Plane Cell Count": 50,
                    "Reference Frame": "<Fixed Frame>",
                    "Value": True
                },
                {
                    "Alpha": 1,
                    "Autocompute Intensity Bounds": True,
                    "Autocompute Value Bounds": {
                        "Max Value": 10,
                        "Min Value": -10,
                        "Value": True
                    },
                    "Axis": "Z",
                    "Channel Name": "intensity",
                    "Class": "rviz/PointCloud2",
                    "Color": "255; 0; 0",
                    "Color Transformer": "FlatColor",
                    "Decay Time": 0,
                    "Enabled": True,
                    "Invert Rainbow": False,
                    "Max Color": "255; 255; 255",
                    "Min Color": "0; 0; 0",
                    "Name": "Red Points",
                    "Position Transformer": "XYZ",
                    "Queue Size": 10,
                    "Selectable": True,
                    "Size (Pixels)": 8,
                    "Size (m)": 0.1,
                    "Style": "Spheres",
                    "Topic": "/yolo11/pointcloud/red",
                    "Unreliable": False,
                    "Use Fixed Frame": True,
                    "Use rainbow": True,
                    "Value": True
                },
                {
                    "Alpha": 1,
                    "Autocompute Intensity Bounds": True,
                    "Autocompute Value Bounds": {
                        "Max Value": 10,
                        "Min Value": -10,
                        "Value": True
                    },
                    "Axis": "Z",
                    "Channel Name": "intensity",
                    "Class": "rviz/PointCloud2",
                    "Color": "255; 255; 0",
                    "Color Transformer": "FlatColor",
                    "Decay Time": 0,
                    "Enabled": True,
                    "Invert Rainbow": False,
                    "Max Color": "255; 255; 255",
                    "Min Color": "0; 0; 0",
                    "Name": "Yellow Points",
                    "Position Transformer": "XYZ",
                    "Queue Size": 10,
                    "Selectable": True,
                    "Size (Pixels)": 8,
                    "Size (m)": 0.1,
                    "Style": "Spheres",
                    "Topic": "/yolo11/pointcloud/yellow",
                    "Unreliable": False,
                    "Use Fixed Frame": True,
                    "Use rainbow": True,
                    "Value": True
                },
                {
                    "Alpha": 1,
                    "Autocompute Intensity Bounds": True,
                    "Autocompute Value Bounds": {
                        "Max Value": 10,
                        "Min Value": -10,
                        "Value": True
                    },
                    "Axis": "Z",
                    "Channel Name": "intensity",
                    "Class": "rviz/PointCloud2",
                    "Color": "255; 255; 255",
                    "Color Transformer": "FlatColor",
                    "Decay Time": 0,
                    "Enabled": True,
                    "Invert Rainbow": False,
                    "Max Color": "255; 255; 255",
                    "Min Color": "0; 0; 0",
                    "Name": "White Points",
                    "Position Transformer": "XYZ",
                    "Queue Size": 10,
                    "Selectable": True,
                    "Size (Pixels)": 8,
                    "Size (m)": 0.1,
                    "Style": "Spheres",
                    "Topic": "/yolo11/pointcloud/white",
                    "Unreliable": False,
                    "Use Fixed Frame": True,
                    "Use rainbow": True,
                    "Value": True
                }
            ],
            "Enabled": True,
            "Global Options": {
                "Background Color": "48; 48; 48",
                "Default Light": True,
                "Fixed Frame": "map",
                "Frame Rate": 30
            },
            "Name": "root",
            "Tools": [
                {
                    "Class": "rviz/Interact",
                    "Hide Inactive Objects": True
                },
                {
                    "Class": "rviz/MoveCamera"
                },
                {
                    "Class": "rviz/Select"
                },
                {
                    "Class": "rviz/FocusCamera"
                },
                {
                    "Class": "rviz/Measure"
                },
                {
                    "Class": "rviz/SetInitialPose",
                    "Topic": "/initialpose"
                },
                {
                    "Class": "rviz/SetGoal",
                    "Topic": "/move_base_simple/goal"
                },
                {
                    "Class": "rviz/PublishPoint",
                    "Single click": False,
                    "Topic": "/clicked_point"
                }
            ],
            "Value": True,
            "Views": {
                "Current": {
                    "Class": "rviz/Orbit",
                    "Distance": 20,
                    "Enable Stereo Rendering": {
                        "Stereo Eye Separation": 0.05999999865889549,
                        "Stereo Focal Distance": 1,
                        "Swap Stereo Eyes": False,
                        "Value": False
                    },
                    "Focal Point": {
                        "X": 0,
                        "Y": 0,
                        "Z": 0
                    },
                    "Focal Shape Fixed Size": True,
                    "Focal Shape Size": 0.05000000074505806,
                    "Invert Z Axis": False,
                    "Name": "Current View",
                    "Near Clip Distance": 0.009999999776482582,
                    "Pitch": 0.785398,
                    "Target Frame": "<Fixed Frame>",
                    "Value": "Orbit (rviz)",
                    "Yaw": 0.785398
                },
                "Saved": []
            }
        }
    }
    
    return rviz_config

def save_rviz_config():
    """保存RViz配置到文件"""
    config = generate_rviz_config()
    
    # 保存为JSON格式（RViz使用YAML，但结构相似）
    config_file = "/root/workspace/perception/yolo11_pointcloud_visualization.rviz"
    
    # 转换为RViz的YAML格式
    rviz_yaml = """Panels:
  - Class: rviz/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Red Points1
        - /Yellow Points1
        - /White Points1
      Splitter Ratio: 0.5
    Tree Height: 565
  - Class: rviz/Tool Properties
    Expanded:
      - /2D Pose Estimate1
      - /2D Nav Goal1
    Name: Tool Properties
    Splitter Ratio: 0.588679
Toolbars: toolbars
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 50
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz/PointCloud2
      Color: 255; 0; 0
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: Red Points
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 8
      Size (m): 0.1
      Style: Spheres
      Topic: /yolo11/pointcloud/red
      Unreliable: false
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz/PointCloud2
      Color: 255; 255; 0
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: Yellow Points
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 8
      Size (m): 0.1
      Style: Spheres
      Topic: /yolo11/pointcloud/yellow
      Unreliable: false
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz/PointCloud2
      Color: 255; 255; 255
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Min Color: 0; 0; 0
      Name: White Points
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 8
      Size (m): 0.1
      Style: Spheres
      Topic: /yolo11/pointcloud/white
      Unreliable: false
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Default Light: true
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz/Interact
      Hide Inactive Objects: true
    - Class: rviz/MoveCamera
    - Class: rviz/Select
    - Class: rviz/FocusCamera
    - Class: rviz/Measure
    - Class: rviz/SetInitialPose
      Topic: /initialpose
    - Class: rviz/SetGoal
      Topic: /move_base_simple/goal
    - Class: rviz/PublishPoint
      Single click: false
      Topic: /clicked_point
  Value: true
  Views:
    Current:
      Class: rviz/Orbit
      Distance: 20
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.785398
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.785398
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002b0fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002b0000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500200053007400650072006500006f0200000100000000d0000000ee0000017afb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000002b0fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d000002b0000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004b00000003efc0100000002fb0000000800540069006d00650100000000000004b0000002eb00fffffffb0000000800540069006d006501000000000000045000000000000000000000023f000002b000000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1200
  X: 67
  Y: 27
"""
    
    try:
        with open(config_file, 'w', encoding='utf-8') as f:
            f.write(rviz_yaml)
        print(f"RViz配置文件已生成: {config_file}")
        print("\n使用方法:")
        print("1. 启动RViz:")
        print("   rviz")
        print("2. 在RViz中加载配置文件:")
        print("   File -> Open Config -> 选择生成的.rviz文件")
        print("3. 或者直接使用配置启动:")
        print(f"   rviz -d {config_file}")
        return True
    except Exception as e:
        print(f"生成RViz配置文件失败: {str(e)}")
        return False

def main():
    """主函数"""
    print("YOLO11点云可视化配置生成器")
    print("="*50)
    
    success = save_rviz_config()
    
    if success:
        print("\n配置说明:")
        print("- 红色球体: red目标的历史位置")
        print("- 黄色球体: yellow目标的历史位置") 
        print("- 白色球体: white目标的历史位置")
        print("- 网格: 地面参考")
        print("- 坐标系: map (世界坐标系)")
        
        print("\n点云话题:")
        print("- /yolo11/pointcloud/red")
        print("- /yolo11/pointcloud/yellow")
        print("- /yolo11/pointcloud/white")

if __name__ == '__main__':
    main()
