#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
基础测试脚本 - 验证模块化拆分后的基本功能
'''

import time
import rospy
from vtol_fly import VTOLFlightController

def test_basic_functionality():
    """测试基础功能"""
    print("🔧 开始基础功能测试...")
    
    try:
        # 初始化ROS节点
        if not rospy.core.is_initialized():
            rospy.init_node('vtol_basic_test', anonymous=True)
            print("✅ ROS节点初始化成功")
        
        # 创建飞行控制器
        print("创建飞行控制器...")
        controller = VTOLFlightController()
        
        # 初始化ROS通信
        print("初始化ROS通信...")
        controller.init_ros_communication()
        
        # 等待位置信息
        print("等待位置信息...")
        if controller.wait_for_position(timeout=5):
            pos = controller.current_position
            if pos:
                print(f"✅ 成功获取位置: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f})")
                
                # 检查安全性
                is_safe, msg = controller.check_flight_safety(pos.x, pos.y, pos.z)
                print(f"安全检查: {msg}")
                
                # 检查模式切换
                can_switch = controller.can_switch_to_multirotor(pos.x, pos.y)
                print(f"可切换旋翼模式: {can_switch}")
                
                print("✅ 基础功能测试通过")
                return True
            else:
                print("❌ 位置信息为空")
                return False
        else:
            print("❌ 获取位置信息超时")
            return False
    
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        if 'controller' in locals():
            controller.shutdown()

def test_command_sending():
    """测试命令发送功能"""
    print("\n🔧 测试命令发送...")
    
    try:
        # 初始化ROS节点
        if not rospy.core.is_initialized():
            rospy.init_node('vtol_cmd_test', anonymous=True)
        
        controller = VTOLFlightController()
        controller.init_ros_communication()
        
        # 测试发送简单命令
        print("发送测试命令...")
        controller.send_cmd("HOVER")
        time.sleep(1)
        
        print("✅ 命令发送测试通过")
        return True
        
    except Exception as e:
        print(f"❌ 命令发送测试失败: {e}")
        return False
    
    finally:
        if 'controller' in locals():
            controller.shutdown()

if __name__ == "__main__":
    print("VTOL基础功能测试")
    print("=" * 50)
    
    # 测试1: 基础功能
    success1 = test_basic_functionality()
    
    # 测试2: 命令发送
    success2 = test_command_sending()
    
    print("\n" + "=" * 50)
    print("测试结果:")
    print(f"基础功能: {'✅ 通过' if success1 else '❌ 失败'}")
    print(f"命令发送: {'✅ 通过' if success2 else '❌ 失败'}")
    
    if success1 and success2:
        print("🎉 所有测试通过，模块化拆分成功!")
    else:
        print("❌ 存在问题，需要进一步调试")
