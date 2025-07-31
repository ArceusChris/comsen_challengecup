#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
视觉降落测试脚本
演示如何使用MultirotorControl类的视觉降落功能
"""

import sys
import rospy
from drone_control import DroneController
from multirotor_control import MultirotorControl

def test_visual_landing():
    """测试视觉降落功能"""
    
    # 检查命令行参数
    if len(sys.argv) < 4:
        print("用法: python3 test_visual_landing.py <multirotor_type> <multirotor_id> <control_type> [target_type]")
        print("例如: python3 test_visual_landing.py iris 0 vel landing_target_red")
        return
    
    multirotor_type = sys.argv[1]  # 例如: "iris"
    multirotor_id = int(sys.argv[2])  # 例如: 0
    control_type = sys.argv[3]  # 例如: "vel"
    target_type = sys.argv[4] if len(sys.argv) > 4 else "landing_target_camo"
    
    print(f"初始化无人机控制器...")
    print(f"  类型: {multirotor_type}")
    print(f"  ID: {multirotor_id}")
    print(f"  控制类型: {control_type}")
    print(f"  目标类型: {target_type}")
    
    try:
        # 创建控制器
        controller = DroneController(multirotor_type, multirotor_id, control_type)
        multirotor_control = MultirotorControl(controller)
        
        # 等待一段时间让系统初始化
        rospy.sleep(2.0)
        
        print("\n=== 开始视觉降落测试 ===")
        
        # 方法1：直接调用视觉降落
        print("\n方法1：直接调用视觉降落方法")
        success = multirotor_control.visual_landing(target_type)
        
        if success:
            print("✓ 视觉降落成功！")
        else:
            print("✗ 视觉降落失败")
            
        print("\n=== 测试完成 ===")
        
    except Exception as e:
        print(f"错误: {e}")
        return False
    
    return True

def test_mission_with_visual_landing():
    """测试带视觉降落的完整任务"""
    
    if len(sys.argv) < 4:
        print("用法: python3 test_visual_landing.py <multirotor_type> <multirotor_id> <control_type> [target_type]")
        return
    
    multirotor_type = sys.argv[1]
    multirotor_id = int(sys.argv[2])
    control_type = sys.argv[3]
    target_type = sys.argv[4] if len(sys.argv) > 4 else "landing_target_camo"
    
    try:
        # 创建控制器
        controller = DroneController(multirotor_type, multirotor_id, control_type)
        multirotor_control = MultirotorControl(controller)
        
        # 等待初始化
        rospy.sleep(2.0)
        
        print("\n=== 开始完整任务测试（包含视觉降落）===")
        
        # 方法2：使用execute_mission并启用视觉降落
        target_position = [0, 0]  # 可根据需要调整目标位置
        
        success = multirotor_control.execute_mission(
            altitude_high=10.0,           # 起飞到10米
            target_position=target_position,  # 移动到目标位置
            use_visual_landing=True,      # 启用视觉降落
            target_type=target_type       # 指定目标类型
        )
        
        if success:
            print("✓ 完整任务（包含视觉降落）执行成功！")
        else:
            print("✗ 任务执行失败")
            
        print("\n=== 任务完成 ===")
        
    except Exception as e:
        print(f"错误: {e}")
        return False
    
    return True

def main():
    """主函数"""
    rospy.init_node('visual_landing_test', anonymous=True)
    
    print("视觉降落测试脚本")
    print("1. 直接测试视觉降落")
    print("2. 测试完整任务（起飞->移动->视觉降落）")
    
    # 这里可以选择测试方式，默认使用方法1
    test_type = input("请选择测试类型 (1 或 2，默认为1): ").strip()
    
    try:
        if test_type == "2":
            success = test_mission_with_visual_landing()
        else:
            success = test_visual_landing()
            
        if success:
            print("\n程序正常结束")
        else:
            print("\n程序异常结束")
            
    except KeyboardInterrupt:
        print("\n收到Ctrl+C信号，正在退出...")
    except Exception as e:
        print(f"\n程序异常: {e}")

if __name__ == "__main__":
    main()
