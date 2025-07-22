#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
相机变换参数配置工具
用于配置相机到无人机坐标系的变换参数
"""

import numpy as np
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class CameraTransformConfig:
    def __init__(self):
        """初始化相机变换配置工具"""
        self.preset_configs = {
            'downward_camera': {
                'description': '相机朝向正下方（推荐配置 - 无人机下方0.03m）',
                'rotation': [
                    [1, 0, 0],   # 相机X轴 -> 无人机X轴（前向）
                    [0, 1, 0],   # 相机Y轴 -> 无人机Y轴（右向）
                    [0, 0, -1]   # 相机Z轴 -> 无人机-Z轴（向下）
                ],
                'translation': [0.0, 0.0, -0.03]  # 无人机正下方0.03m
            },
            'forward_down': {
                'description': '相机朝前向下（常用于地面目标检测）',
                'rotation': [
                    [0, 0, 1],   # 相机Z轴 -> 无人机X轴（前向）
                    [-1, 0, 0],  # 相机X轴 -> 无人机-Y轴
                    [0, -1, 0]   # 相机Y轴 -> 无人机-Z轴（向下）
                ],
                'translation': [0.0, 0.0, 0.0]
            },
            'forward_level': {
                'description': '相机朝前水平（用于前方目标检测）',
                'rotation': [
                    [0, -1, 0],  # 相机Z轴 -> 无人机-Y轴
                    [0, 0, -1],  # 相机X轴 -> 无人机-Z轴
                    [1, 0, 0]    # 相机Y轴 -> 无人机X轴（前向）
                ],
                'translation': [0.1, 0.0, 0.0]  # 相机稍微前置
            },
            'down_only': {
                'description': '相机正下方（云台相机）',
                'rotation': [
                    [1, 0, 0],   # 相机Z轴 -> 无人机X轴
                    [0, 1, 0],   # 相机X轴 -> 无人机Y轴
                    [0, 0, 1]    # 相机Y轴 -> 无人机Z轴
                ],
                'translation': [0.0, 0.0, -0.05]  # 相机稍微下置
            }
        }
    
    def rotation_matrix_from_euler(self, roll, pitch, yaw):
        """从欧拉角创建旋转矩阵"""
        # 将角度转换为弧度
        roll_rad = np.radians(roll)
        pitch_rad = np.radians(pitch)
        yaw_rad = np.radians(yaw)
        
        # 创建旋转矩阵
        cr = np.cos(roll_rad)
        sr = np.sin(roll_rad)
        cp = np.cos(pitch_rad)
        sp = np.sin(pitch_rad)
        cy = np.cos(yaw_rad)
        sy = np.sin(yaw_rad)
        
        rotation_matrix = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
        
        return rotation_matrix
    
    def euler_from_rotation_matrix(self, rotation_matrix):
        """从旋转矩阵提取欧拉角"""
        # 提取欧拉角（ZYX顺序）
        sy = np.sqrt(rotation_matrix[0,0]**2 + rotation_matrix[1,0]**2)
        
        singular = sy < 1e-6
        
        if not singular:
            x = np.arctan2(rotation_matrix[2,1], rotation_matrix[2,2])
            y = np.arctan2(-rotation_matrix[2,0], sy)
            z = np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0])
        else:
            x = np.arctan2(-rotation_matrix[1,2], rotation_matrix[1,1])
            y = np.arctan2(-rotation_matrix[2,0], sy)
            z = 0
        
        return np.degrees([x, y, z])  # 返回角度值
    
    def display_config(self, config_name, config):
        """显示配置信息"""
        print(f"\n=== {config_name} ===")
        print(f"描述: {config['description']}")
        print(f"旋转矩阵:")
        rotation = np.array(config['rotation'])
        for row in rotation:
            print(f"  [{row[0]:6.3f}, {row[1]:6.3f}, {row[2]:6.3f}]")
        
        print(f"平移向量: {config['translation']}")
        
        # 显示等效的欧拉角
        euler_angles = self.euler_from_rotation_matrix(rotation)
        print(f"等效欧拉角 (度): Roll={euler_angles[0]:.1f}, Pitch={euler_angles[1]:.1f}, Yaw={euler_angles[2]:.1f}")
    
    def generate_ros_param_yaml(self, config, output_file=None):
        """生成ROS参数YAML文件"""
        if output_file is None:
            output_file = "/root/workspace/perception/camera_transform_params.yaml"
        
        yaml_content = f"""# 相机变换参数配置文件
# 用于YOLO11推理节点的相机到无人机坐标系变换

yolo11_inference_node:
  # 相机到无人机坐标系的旋转矩阵 (3x3)
  camera_rotation_matrix: [
    [{config['rotation'][0][0]:6.3f}, {config['rotation'][0][1]:6.3f}, {config['rotation'][0][2]:6.3f}],
    [{config['rotation'][1][0]:6.3f}, {config['rotation'][1][1]:6.3f}, {config['rotation'][1][2]:6.3f}],
    [{config['rotation'][2][0]:6.3f}, {config['rotation'][2][1]:6.3f}, {config['rotation'][2][2]:6.3f}]
  ]
  
  # 相机相对于无人机的位置偏移 [x, y, z] (米)
  camera_translation: [{config['translation'][0]:6.3f}, {config['translation'][1]:6.3f}, {config['translation'][2]:6.3f}]
  
  # 其他推理参数
  input_topic: "/iris_0/camera/image_raw"
  output_topic: "/yolo11/detection_image"
  pose_topic: "/iris_0/mavros/local_position/pose"
  camera_info_topic: "/iris_0/camera/camera_info"
  model_path: "yolo11n.pt"
  confidence_threshold: 0.5
  iou_threshold: 0.45
"""
        
        try:
            with open(output_file, 'w', encoding='utf-8') as f:
                f.write(yaml_content)
            print(f"参数文件已生成: {output_file}")
            return True
        except Exception as e:
            print(f"生成参数文件失败: {str(e)}")
            return False
    
    def create_custom_config(self):
        """创建自定义配置"""
        print("\n=== 创建自定义相机变换配置 ===")
        print("请选择输入方式:")
        print("1. 输入欧拉角 (Roll, Pitch, Yaw)")
        print("2. 直接输入旋转矩阵")
        
        choice = input("选择 (1-2): ").strip()
        
        if choice == '1':
            try:
                print("\n输入相机相对于无人机的欧拉角 (度):")
                roll = float(input("Roll (绕X轴旋转): "))
                pitch = float(input("Pitch (绕Y轴旋转): "))
                yaw = float(input("Yaw (绕Z轴旋转): "))
                
                rotation_matrix = self.rotation_matrix_from_euler(roll, pitch, yaw)
                
            except ValueError:
                print("输入格式错误，使用默认配置")
                return None
                
        elif choice == '2':
            try:
                print("\n输入3x3旋转矩阵 (按行输入，用空格分隔):")
                rotation_matrix = []
                for i in range(3):
                    row_str = input(f"第{i+1}行: ")
                    row = [float(x) for x in row_str.split()]
                    if len(row) != 3:
                        raise ValueError("每行必须有3个数值")
                    rotation_matrix.append(row)
                
                rotation_matrix = np.array(rotation_matrix)
                
            except (ValueError, IndexError):
                print("输入格式错误，使用默认配置")
                return None
        else:
            print("无效选择")
            return None
        
        # 输入平移向量
        try:
            print("\n输入相机相对于无人机的位置偏移 (米):")
            tx = float(input("X偏移: "))
            ty = float(input("Y偏移: "))
            tz = float(input("Z偏移: "))
            translation = [tx, ty, tz]
            
        except ValueError:
            print("使用默认平移向量 [0, 0, 0]")
            translation = [0.0, 0.0, 0.0]
        
        custom_config = {
            'description': '自定义配置',
            'rotation': rotation_matrix.tolist(),
            'translation': translation
        }
        
        return custom_config
    
    def interactive_menu(self):
        """交互式菜单"""
        while True:
            print("\n=== 相机变换参数配置工具 ===")
            print("1. 查看预设配置")
            print("2. 选择预设配置并生成参数文件")
            print("3. 创建自定义配置")
            print("4. 测试当前Gazebo仿真的相机配置")
            print("5. 退出")
            
            choice = input("\n请选择操作 (1-5): ").strip()
            
            if choice == '1':
                print("\n可用的预设配置:")
                for name, config in self.preset_configs.items():
                    self.display_config(name, config)
                    
            elif choice == '2':
                print("\n选择预设配置:")
                config_names = list(self.preset_configs.keys())
                for i, name in enumerate(config_names):
                    print(f"{i+1}. {name} - {self.preset_configs[name]['description']}")
                
                try:
                    idx = int(input(f"选择配置 (1-{len(config_names)}): ")) - 1
                    if 0 <= idx < len(config_names):
                        selected_name = config_names[idx]
                        selected_config = self.preset_configs[selected_name]
                        self.display_config(selected_name, selected_config)
                        
                        confirm = input("\n确认生成参数文件? (y/n): ").strip().lower()
                        if confirm == 'y':
                            self.generate_ros_param_yaml(selected_config)
                    else:
                        print("无效选择")
                except ValueError:
                    print("输入格式错误")
                    
            elif choice == '3':
                custom_config = self.create_custom_config()
                if custom_config:
                    self.display_config("自定义配置", custom_config)
                    confirm = input("\n确认生成参数文件? (y/n): ").strip().lower()
                    if confirm == 'y':
                        self.generate_ros_param_yaml(custom_config)
                        
            elif choice == '4':
                print("\n=== Gazebo仿真相机配置测试 ===")
                print("建议使用 'gazebo_iris' 预设配置")
                print("如果检测结果不准确，可能需要:")
                print("1. 检查Gazebo中相机的实际安装方向")
                print("2. 调整旋转矩阵参数")
                print("3. 在实际飞行中观察检测效果并微调")
                
            elif choice == '5':
                print("退出程序")
                break
                
            else:
                print("无效选择，请重试")

def main():
    """主函数"""
    print("相机变换参数配置工具")
    print("用于配置YOLO11推理节点的相机到无人机坐标系变换参数")
    
    config_tool = CameraTransformConfig()
    config_tool.interactive_menu()

if __name__ == '__main__':
    main()
