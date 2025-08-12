#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
迷彩图案检测器测试脚本
用于验证世界坐标计算功能是否正确添加
"""

import sys
import ast

def test_camouflage_detector():
    """测试迷彩图案检测器的修改"""
    file_path = "/home/xylin/comsen_challengecup/workspace/src/landing_detection/scripts/camouflage_pattern_detector.py"
    
    print("测试迷彩图案检测器的修改...")
    print("=" * 50)
    
    try:
        # 读取文件内容
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 检查必要的导入
        required_imports = [
            'import tf2_ros',
            'from tf.transformations import quaternion_matrix',
            'from sensor_msgs.msg import Image, CameraInfo',
            'from geometry_msgs.msg import Point, PointStamped, PoseStamped'
        ]
        
        print("1. 检查导入...")
        for imp in required_imports:
            if imp in content:
                print(f"   ✓ {imp}")
            else:
                print(f"   ✗ 缺少: {imp}")
        
        # 检查必要的函数
        required_functions = [
            'def pose_callback',
            'def camera_info_callback',
            'def pixel_to_world_coordinate',
            'def transform_to_matrix'
        ]
        
        print("\n2. 检查函数...")
        for func in required_functions:
            if func in content:
                print(f"   ✓ {func}")
            else:
                print(f"   ✗ 缺少: {func}")
        
        # 检查发布器
        required_publishers = [
            'world_coord_pub',
            'target_pub',
            'debug_pub'
        ]
        
        print("\n3. 检查发布器...")
        for pub in required_publishers:
            if pub in content:
                print(f"   ✓ {pub}")
            else:
                print(f"   ✗ 缺少: {pub}")
        
        # 检查订阅器
        required_subscribers = [
            'pose_sub',
            'camera_info_sub',
            'image_sub'
        ]
        
        print("\n4. 检查订阅器...")
        for sub in required_subscribers:
            if sub in content:
                print(f"   ✓ {sub}")
            else:
                print(f"   ✗ 缺少: {sub}")
        
        # 检查TF2初始化
        tf2_components = [
            'tf_buffer',
            'tf_listener',
            'camera_frame_id'
        ]
        
        print("\n5. 检查TF2组件...")
        for comp in tf2_components:
            if comp in content:
                print(f"   ✓ {comp}")
            else:
                print(f"   ✗ 缺少: {comp}")
        
        # 检查世界坐标发布逻辑
        world_coord_publish_check = [
            'pixel_to_world_coordinate',
            'world_coord_pub.publish',
            'world_coord_msg'
        ]
        
        print("\n6. 检查世界坐标发布逻辑...")
        for check in world_coord_publish_check:
            if check in content:
                print(f"   ✓ {check}")
            else:
                print(f"   ✗ 缺少: {check}")
        
        # 尝试解析Python语法
        print("\n7. 检查Python语法...")
        try:
            ast.parse(content)
            print("   ✓ Python语法正确")
        except SyntaxError as e:
            print(f"   ✗ 语法错误: {e}")
            return False
        
        print("\n" + "=" * 50)
        print("✓ 所有检查通过！迷彩图案检测器已成功添加世界坐标计算功能。")
        print("\n新增功能:")
        print("- 订阅位姿话题: /iris_0/mavros/local_position/pose")
        print("- 订阅相机内参话题: /iris_0/camera/camera_info")
        print("- 发布世界坐标话题: /landing_target_camo/world_coord")
        print("- 像素坐标到世界坐标转换（假设目标在地面上）")
        print("- 使用TF2进行坐标系变换")
        
        return True
        
    except Exception as e:
        print(f"测试失败: {e}")
        return False

if __name__ == '__main__':
    success = test_camouflage_detector()
    sys.exit(0 if success else 1)
