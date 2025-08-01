#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试condition定时器功能的脚本
"""

import sys
import os
import time

# 添加当前脚本目录到Python路径
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

def test_condition_timer():
    """测试condition定时器功能"""
    print("测试condition定时器功能")
    print("=" * 50)
    
    try:
        # 初始化ROS
        import rospy
        rospy.init_node('test_condition_timer', anonymous=True)
        print("✅ ROS节点初始化成功")
        
        # 导入VTOL演示类
        from vtol_demo import VTOLDemoFlight
        
        # 创建演示对象（不执行完整任务）
        print("创建VTOLDemoFlight对象...")
        demo = VTOLDemoFlight()
        
        # 初始化ROS通信
        print("初始化ROS通信...")
        demo.init_ros()
        
        # 测试定时器状态
        print("\n检查定时器状态:")
        status = demo.get_condition_status()
        print(f"  当前状态: 0x{status['current_condition']:02X}")
        print(f"  定时器状态: {status['timer_status']}")
        print(f"  发布频率: {status['publish_rate']}Hz")
        
        # 测试状态更新
        print("\n测试状态更新...")
        test_conditions = [0x01, 0x02, 0x03, 0x04, 0x05]
        
        for condition in test_conditions:
            print(f"设置状态为: 0x{condition:02X}")
            demo.current_condition = condition
            time.sleep(1)  # 等待1秒让定时器发布
            
        # 最终状态检查
        print("\n最终状态检查:")
        final_status = demo.get_condition_status()
        print(f"  最终状态: 0x{final_status['current_condition']:02X}")
        print(f"  定时器状态: {final_status['timer_status']}")
        
        # 清理资源
        print("\n清理资源...")
        demo.shutdown()
        
        print("✅ 测试完成！")
        return True
        
    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_condition_timer()
    sys.exit(0 if success else 1)
