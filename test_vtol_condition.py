#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试VTOL Condition话题功能
验证condition话题的发布和订阅是否正常工作
"""

import rospy
import time
import yaml
import os
from std_msgs.msg import Int8

class ConditionTester:
    def __init__(self):
        self.received_conditions = []
        
        # 初始化ROS节点
        rospy.init_node('condition_tester', anonymous=True)
        
        # 订阅condition话题
        self.condition_sub = rospy.Subscriber(
            '/zhihang2025/vtol_land_sub/done', 
            Int8, self.condition_callback, queue_size=10)
        
        # 发布condition话题
        self.condition_pub = rospy.Publisher(
            '/zhihang2025/vtol_land_sub/done', 
            Int8, queue_size=10)
        
        print("🧪 Condition话题测试器初始化完成")
        print("📡 话题: /zhihang2025/vtol_land_sub/done")
        
    def condition_callback(self, msg):
        """Condition话题回调函数"""
        condition = msg.data
        self.received_conditions.append(condition)
        print(f"📨 收到Condition: 0x{condition:02X}")
    
    def send_test_condition(self, condition_value):
        """发送测试condition"""
        msg = Int8()
        msg.data = condition_value
        self.condition_pub.publish(msg)
        print(f"📤 发送测试Condition: 0x{condition_value:02X}")
    
    def test_yaml_loading(self):
        """测试YAML文件中condition的加载"""
        print("\n🔍 测试YAML文件condition加载...")
        
        yaml_path = '/home/yzy/comsen_challengecup/workspace/vtol_control/vtol_target.yaml'
        
        if not os.path.exists(yaml_path):
            print("❌ YAML文件不存在")
            return False
        
        try:
            with open(yaml_path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            
            if 'targets' in data:
                print("✅ 成功加载目标点配置:")
                for i, target in enumerate(data['targets']):
                    pos = target.get('position', [0, 0, 0])
                    name = target.get('name', 'unknown')
                    condition = target.get('condition', '0x00')
                    
                    # 解析condition
                    if isinstance(condition, str) and condition.startswith('0x'):
                        condition_value = int(condition, 16)
                    else:
                        condition_value = int(condition) if isinstance(condition, (int, str)) else 0
                    
                    print(f"   {i+1}. {name}: {pos} - Condition: 0x{condition_value:02X}")
                return True
            else:
                print("❌ YAML文件中没有targets字段")
                return False
                
        except Exception as e:
            print(f"❌ 加载YAML文件失败: {e}")
            return False
    
    def run_tests(self):
        """运行所有测试"""
        print("🚀 开始Condition功能测试")
        print("=" * 50)
        
        # 测试1: YAML加载
        yaml_ok = self.test_yaml_loading()
        
        # 测试2: 等待话题建立连接
        print("\n⏱️ 等待话题连接...")
        time.sleep(2)
        
        # 测试3: 发送测试condition
        print("\n📤 测试condition发送...")
        test_conditions = [0xAA, 0x01, 0x02, 0x03, 0x04, 0x05]
        
        for condition in test_conditions:
            self.send_test_condition(condition)
            time.sleep(0.5)
        
        # 等待接收
        print("\n⏱️ 等待接收消息...")
        time.sleep(2)
        
        # 测试结果
        print(f"\n📊 测试结果:")
        print(f"   YAML加载: {'✅ 成功' if yaml_ok else '❌ 失败'}")
        print(f"   发送的condition数量: {len(test_conditions)}")
        print(f"   接收的condition数量: {len(self.received_conditions)}")
        
        if self.received_conditions:
            print("   接收到的condition:")
            for condition in self.received_conditions:
                print(f"     - 0x{condition:02X}")
        
        # 检查是否收到了所有发送的condition
        if len(self.received_conditions) >= len(test_conditions):
            print("✅ Condition收发测试成功")
            return True
        else:
            print("⚠️ 部分condition未收到")
            return False

def main():
    """主函数"""
    try:
        tester = ConditionTester()
        success = tester.run_tests()
        
        if success:
            print("\n🎉 所有测试通过！")
            print("💡 VTOL Condition话题功能正常")
        else:
            print("\n⚠️ 部分测试失败")
            print("💡 请检查话题配置和YAML文件")
            
    except KeyboardInterrupt:
        print("\n🛑 测试被用户中断")
    except Exception as e:
        print(f"\n❌ 测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
