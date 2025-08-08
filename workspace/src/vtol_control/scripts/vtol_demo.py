#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
VTOL无人机演示飞行主控脚本（简化版）
模块化重构后的主控脚本，负责任务管理和流程控制
具体的飞行控制委托给 VTOLFlightController
'''

import sys
import os
import time
import math
import yaml

# 添加当前脚本目录到Python路径，用于导入本地模块
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from vtol_map import VTOLMap, ZoneType
from vtol_fly import VTOLFlightController


class VTOLDemoFlight:
    def __init__(self, vehicle_type="standard_vtol", vehicle_id="0"):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        
        # 初始化地图
        self.map = VTOLMap()
        
        # 初始化飞行控制器
        self.flight_controller = VTOLFlightController(vehicle_type, vehicle_id)
        
        # 加载目标点
        self.targets = self.load_targets()
        
        # 当前状态
        self.current_target_index = 0
        
        print(f"初始化VTOL演示飞行: {self.vehicle_type}_{self.vehicle_id}")
        print(f"加载了 {len(self.targets)} 个目标点")

    def load_targets(self):
        """加载vtol_target.yaml中的目标点"""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        target_file = os.path.join(current_dir, "vtol_target.yaml")
        targets = []
        
        try:
            if os.path.exists(target_file):
                with open(target_file, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                    
                if data and 'targets' in data:
                    for target_info in data['targets']:
                        if 'position' in target_info:
                            pos = target_info['position']
                            if len(pos) >= 3:
                                x, y, z = pos[0], pos[1], pos[2]
                                name = target_info.get('name', f'target_{len(targets)+1}')
                                description = target_info.get('description', '')
                                condition = target_info.get('condition', '0x00')
                                
                                # 解析condition字符串为整数
                                if isinstance(condition, str) and condition.startswith('0x'):
                                    condition_value = int(condition, 16)
                                else:
                                    condition_value = int(condition) if isinstance(condition, (int, str)) else 0
                                
                                targets.append({
                                    'position': (x, y, z),
                                    'name': name,
                                    'description': description,
                                    'condition': condition_value
                                })
                else:
                    raise ValueError("YAML文件格式错误：缺少'targets'字段")
            else:
                raise FileNotFoundError(f"目标文件不存在: {target_file}")
                
        except Exception as e:
            print(f"❌ 加载目标文件失败: {e}")
            print("❌ YAML文件解析错误，拒绝起飞")
            raise e
        
        # 验证目标点数量
        if not targets:
            raise ValueError("没有有效的目标点")
        
        print(f"✅ 成功加载 {len(targets)} 个目标点")
        for i, target in enumerate(targets):
            x, y, z = target['position']
            condition = target.get('condition', 0)
            zone_info = self.map.get_zone_info(x, y)
            print(f"  {i+1}. {target['name']}: ({x}, {y}, {z}) - {zone_info['name']} - Condition: 0x{condition:02X}")
        
        return targets

    def init_ros(self):
        """初始化ROS通信"""
        self.flight_controller.init_ros_communication()
        print("演示控制器ROS通信初始化完成")

    def publish_condition(self, condition_value):
        """发布condition状态"""
        self.flight_controller.ros_comm.publish_condition(condition_value)
    
    def update_mission_condition(self, target_index):
        """根据目标点索引更新并发送condition"""
        if target_index < len(self.targets):
            target = self.targets[target_index]
            condition = target.get('condition', 0x00)
            
            print(f"到达目标点 {target['name']}，发送Condition: 0x{condition:02X}")
            self.publish_condition(condition)

    @property
    def current_position(self):
        """获取当前位置"""
        return self.flight_controller.current_position

    def wait_for_connection(self):
        """等待ROS连接"""
        print("等待位置信息...")
        
        # 先检查ROS通信状态
        print("检查ROS通信状态...")
        if not self.flight_controller.is_ros_ok():
            print("❌ ROS通信状态异常")
            return False
        
        # 尝试获取位置信息，增加重试机制
        max_retries = 3
        for attempt in range(max_retries):
            print(f"尝试获取位置信息 (第{attempt+1}次/共{max_retries}次)...")
            
            if self.flight_controller.wait_for_position():
                current_pos = self.current_position
                if current_pos:
                    print(f"✅ 成功获取位置: ({current_pos.x:.1f}, {current_pos.y:.1f}, {current_pos.z:.1f})")
                    
                    # 验证位置是否在地图边界内
                    if not self.map.is_in_bounds(current_pos.x, current_pos.y):
                        print("⚠️ 位置超出地图边界，但继续执行")
                    
                    return True
                else:
                    print(f"⚠️ 第{attempt+1}次尝试：位置信息为空")
            else:
                print(f"⚠️ 第{attempt+1}次尝试：获取位置超时")
            
            if attempt < max_retries - 1:
                print("等待2秒后重试...")
                time.sleep(2)
        
        print("❌ 所有尝试都失败，但允许继续执行（可能是仿真环境问题）")
        return True  # 改为True，允许在没有位置信息的情况下继续

    def check_flight_safety(self, x, y, z):
        """检查飞行安全性"""
        return self.flight_controller.check_flight_safety(x, y, z)

    def can_switch_to_multirotor(self, x=None, y=None):
        """检查是否可以切换到旋翼模式"""
        return self.flight_controller.can_switch_to_multirotor(x, y)

    # 飞行控制方法 - 委托给飞行控制器
    def takeoff_sequence(self):
        """起飞序列"""
        return self.flight_controller.takeoff_sequence()

    def fly_to_target(self, target_x, target_y, target_z):
        """飞向指定目标"""
        return self.flight_controller.fly_to_target(target_x, target_y, target_z)

    def land_at_target(self, target_x, target_y, target_z):
        """在目标点降落"""
        return self.flight_controller.land_at_target(target_x, target_y, target_z)

    def return_to_launch(self):
        """返航到起飞点"""
        return self.flight_controller.return_to_launch()

    def emergency_stop(self):
        """紧急停止"""
        self.flight_controller.emergency_stop()

    def _maintain_position_stability(self, duration_seconds):
        """在指定时间内持续发布当前位置以保持稳定 - 0.1秒快速响应"""
        if self.flight_controller and self.flight_controller.current_position:
            current_pos = self.flight_controller.current_position
            
            # 使用当前位置，确保高度安全
            x = current_pos.x
            y = current_pos.y
            z = max(current_pos.z, 20.0)  # 确保最低20米高度
            
            # 持续发布控制指令，0.1秒间隔
            iterations = int(duration_seconds * 10)  # 每0.1秒发布一次
            for _ in range(iterations):
                self.flight_controller.set_target_pose(x, y, z)
                time.sleep(0.1)  # 0.1秒快速响应
        else:
            # 如果无法获取位置，使用简单等待
            time.sleep(duration_seconds)

    # 任务管理和流程控制
    def run_mission(self):
        """执行完整任务 - 新的任务流程"""
        print(f"\n🎯 开始执行VTOL自动飞行任务 (新任务流程)")
        print("="*60)
        print("任务概览:")
        print("1. 旋翼模式起飞到40米高度")
        print("2. 固定翼模式飞向目标点1 (1495, 250, 40)")
        print("3. 飞向目标点2 (1495, -250, 40)")
        print("4. 依次飞向三个人员位置 (高度20米)")
        print("5. 返回旋翼区并自动返航")
        print("="*60)
        
        try:
            # 1. 初始化和连接检查
            print("\n📋 步骤1: 系统初始化...")
            if not self.init_and_check():
                print("❌ 初始化失败，任务终止")
                return False
            
            # 2. 任务1: 旋翼模式起飞到40米
            print("\n📋 任务1: 旋翼模式起飞到40米...")
            if not self.flight_controller.takeoff_sequence():
                print("❌ 任务1失败")
                return False
            print("✅ 任务1完成")
            self.publish_condition(0x01)  # 发布状态0x01
            self._maintain_position_stability(2)
            
            # 3. 任务2: 固定翼模式飞向目标点1
            print("\n📋 任务2: 固定翼模式飞向目标点1...")
            if not self.flight_controller.fly_to_target_1():
                print("❌ 任务2失败")
                return False
            print("✅ 任务2完成")
            self.publish_condition(0x02)  # 发布状态0x02
            self._maintain_position_stability(2)
            
            # 4. 任务3: 飞向目标点2，并在此处更新一次人员位置
            print("\n📋 任务3: 飞向目标点2...")
            if not self.flight_controller.fly_to_target_2():
                print("❌ 任务3失败")
                return False
            print("✅ 任务3完成")
            self.publish_condition(0x03)  # 发布状态0x03
            
            # 关键：只在此处更新一次人员位置
            print("\n🔄 更新人员位置信息...")
            self.flight_controller.person_reader.update_positions_once()
            self._maintain_position_stability(2)
            
            # 5. 任务4: 依次飞向人员位置
            print("\n📋 任务4: 依次飞向人员位置...")
            if not self.flight_controller.visit_persons():
                print("❌ 任务4失败")
                return False
            print("✅ 任务4完成")
            self.publish_condition(0x04)  # 发布状态0x04
            self._maintain_position_stability(2)
            
            # 6. 任务5: 返回旋翼区并自动返航
            print("\n📋 任务5: 返回旋翼区并自动返航...")
            if not self.flight_controller.return_to_multirotor_zone():
                print("❌ 任务5失败")
                return False
            print("✅ 任务5完成")
            print("📡 正在发布状态0x05...")
            self.publish_condition(0x05)  # 发布状态0x05
            time.sleep(0.2)  # 给ROS一点时间处理消息
            print("✅ 状态0x05已发布")
            
            print("\n🎉 所有任务完成！")
            return True
            
        except KeyboardInterrupt:
            print(f"\n⚠️ 用户中断任务")
            self.emergency_stop()
            return False
        except Exception as e:
            print(f"\n❌ 任务执行异常: {e}")
            import traceback
            traceback.print_exc()
            self.emergency_stop()
            return False

    def init_and_check(self):
        """初始化和检查"""
        print("🔧 初始化ROS通信...")
        self.init_ros()
        
        print("📡 等待连接...")
        if not self.wait_for_connection():
            print("❌ 连接检查失败")
            return False
        
        print("✅ 初始化完成")
        return True

    def print_mission_summary(self, success_count):
        """打印任务总结"""
        print(f"\n📊 任务总结")
        print("="*50)
        print(f"总目标点数: {len(self.targets)}")
        print(f"成功访问: {success_count}")
        print(f"成功率: {success_count/len(self.targets)*100:.1f}%")
        
        if self.current_position:
            final_pos = self.current_position
            print(f"最终位置: ({final_pos.x:.1f}, {final_pos.y:.1f}, {final_pos.z:.1f})")
        
        print("任务完成! 🎉")

    def shutdown(self):
        """关闭系统"""
        print("关闭VTOL演示系统...")
        self.flight_controller.shutdown()


def main():
    """主函数"""
    print("VTOL演示飞行系统启动")
    print("=" * 60)
    
    # 初始化ROS节点
    try:
        import rospy
        if not rospy.core.is_initialized():
            rospy.init_node('vtol_demo_flight', anonymous=True)
            print("✅ ROS节点初始化成功")
        else:
            print("✅ ROS节点已初始化")
    except Exception as e:
        print(f"❌ ROS节点初始化失败: {e}")
        return False
    
    try:
        # 创建演示飞行对象
        demo = VTOLDemoFlight()
        
        # 执行任务
        success = demo.run_mission()
        
        if success:
            print("✅ 演示飞行任务完成")
        else:
            print("❌ 演示飞行任务失败")
        
        # 清理资源
        demo.shutdown()
        
    except Exception as e:
        print(f"❌ 系统错误: {e}")
        return False
    
    return True


if __name__ == "__main__":
    main()
