#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
VTOL精确控制诊断和改进脚本
专门解决无人机到达精度问题
"""

import rospy
import time
import math
import threading
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String

class VTOLPrecisionController:
    def __init__(self, vehicle_type="standard_vtol", vehicle_id="0"):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        
        # 当前状态
        self.current_position = None
        self.target_pose = Pose()
        self.should_publish = False
        
        # 精确控制参数
        self.position_tolerance = 10.0  # 10米容忍度
        self.approach_stages = [50, 20, 10]  # 分阶段接近距离
        self.max_wait_time = 120  # 最大等待时间
        
        print(f"精确控制器初始化: {self.vehicle_type}_{self.vehicle_id}")
        self.init_ros()

    def init_ros(self):
        """初始化ROS节点和通信"""
        rospy.init_node(f"{self.vehicle_type}_{self.vehicle_id}_precision_controller")
        
        # 订阅者
        self.local_pose_sub = rospy.Subscriber(
            f"{self.vehicle_type}_{self.vehicle_id}/mavros/local_position/pose", 
            PoseStamped, self.local_pose_callback, queue_size=1)
        
        # 发布者
        self.cmd_pub = rospy.Publisher(
            f"/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd", 
            String, queue_size=10)
        
        self.cmd_pose_pub = rospy.Publisher(
            f"/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd_pose_enu", 
            Pose, queue_size=10)
        
        print("精确控制器ROS通信初始化完成")

    def local_pose_callback(self, msg):
        """位置回调函数"""
        self.current_position = msg.pose.position

    def send_cmd(self, cmd_str):
        """发送xtdrone命令"""
        cmd_msg = String()
        cmd_msg.data = cmd_str
        self.cmd_pub.publish(cmd_msg)
        print(f"发送命令: {cmd_str}")

    def set_target_pose(self, x, y, z, yaw=0.0):
        """设置目标位置"""
        self.target_pose.position.x = x
        self.target_pose.position.y = y
        self.target_pose.position.z = z
        self.target_pose.orientation.x = 0.0
        self.target_pose.orientation.y = 0.0
        self.target_pose.orientation.z = math.sin(yaw / 2.0)
        self.target_pose.orientation.w = math.cos(yaw / 2.0)

    def publish_position_continuously(self):
        """高频率持续发布位置指令"""
        rate = rospy.Rate(50)  # 50Hz，比之前的20Hz更高
        while self.should_publish and not rospy.is_shutdown():
            try:
                self.cmd_pose_pub.publish(self.target_pose)
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def get_distance_to_target(self, target_x, target_y, target_z):
        """计算到目标点的距离"""
        if self.current_position is None:
            return float('inf')
        
        dx = target_x - self.current_position.x
        dy = target_y - self.current_position.y
        dz = target_z - self.current_position.z
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def wait_for_position(self):
        """等待位置信息"""
        print("等待位置信息...")
        timeout = 10
        start_time = time.time()
        
        while self.current_position is None and not rospy.is_shutdown():
            if time.time() - start_time > timeout:
                print("❌ 位置信息超时")
                return False
            time.sleep(0.1)
        
        if self.current_position:
            print(f"✅ 获取到位置: ({self.current_position.x:.1f}, {self.current_position.y:.1f}, {self.current_position.z:.1f})")
            return True
        return False

    def precise_approach(self, target_x, target_y, target_z):
        """精确接近目标点 - 分阶段接近策略"""
        print(f"\n🎯 开始精确接近目标: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})")
        print("="*60)
        
        if not self.current_position:
            print("❌ 无法获取当前位置")
            return False
        
        # 开始持续发布位置指令
        self.should_publish = True
        publish_thread = threading.Thread(target=self.publish_position_continuously)
        publish_thread.daemon = True
        publish_thread.start()
        
        # 分阶段接近策略
        for stage, stage_tolerance in enumerate(self.approach_stages, 1):
            print(f"\n📍 阶段 {stage}: 接近到 {stage_tolerance}m 内")
            print("-" * 40)
            
            # 设置当前阶段的目标
            self.set_target_pose(target_x, target_y, target_z)
            
            # 等待接近到指定距离
            stage_start_time = time.time()
            min_distance_achieved = float('inf')
            stable_count = 0  # 稳定计数器
            
            while time.time() - stage_start_time < self.max_wait_time:
                # 高频率发布位置指令
                self.set_target_pose(target_x, target_y, target_z)
                
                if self.current_position:
                    current_distance = self.get_distance_to_target(target_x, target_y, target_z)
                    min_distance_achieved = min(min_distance_achieved, current_distance)
                    
                    # 每2秒报告状态
                    elapsed = time.time() - stage_start_time
                    if int(elapsed) % 2 == 0 and elapsed > 0:
                        print(f"  时间: {elapsed:5.1f}s | 距离: {current_distance:6.1f}m | 最小: {min_distance_achieved:6.1f}m")
                        print(f"  位置: ({self.current_position.x:7.1f}, {self.current_position.y:7.1f}, {self.current_position.z:6.1f})")
                    
                    # 检查是否到达阶段目标
                    if current_distance <= stage_tolerance:
                        stable_count += 1
                        if stable_count >= 5:  # 连续5次检查都在容忍度内
                            print(f"  ✅ 阶段 {stage} 完成！距离: {current_distance:.1f}m")
                            break
                    else:
                        stable_count = 0
                
                time.sleep(0.1)  # 10Hz检查频率
            
            # 阶段结果评估
            if self.current_position:
                final_distance = self.get_distance_to_target(target_x, target_y, target_z)
                if final_distance <= stage_tolerance:
                    print(f"  ✅ 阶段 {stage} 成功完成")
                else:
                    print(f"  ⚠️ 阶段 {stage} 未完全达标，距离: {final_distance:.1f}m")
                    
                    # 如果是最后一个阶段，仍然可以接受
                    if stage == len(self.approach_stages):
                        print(f"  📊 最终接近完成，最小距离: {min_distance_achieved:.1f}m")
                        break
            else:
                print(f"  ❌ 阶段 {stage} 失败：无法获取位置")
                break
        
        # 停止发布
        self.should_publish = False
        time.sleep(0.5)
        
        # 最终结果
        if self.current_position:
            final_distance = self.get_distance_to_target(target_x, target_y, target_z)
            print(f"\n🏁 精确接近完成!")
            print(f"   最终距离: {final_distance:.1f}m")
            print(f"   最终位置: ({self.current_position.x:.1f}, {self.current_position.y:.1f}, {self.current_position.z:.1f})")
            
            if final_distance <= self.position_tolerance:
                print(f"   ✅ 精度达标 (容忍度: {self.position_tolerance}m)")
                return True
            else:
                print(f"   ⚠️ 精度未达标 (容忍度: {self.position_tolerance}m)")
                return False
        else:
            print("   ❌ 无法评估最终结果")
            return False

    def test_precision_control(self):
        """测试精确控制"""
        print("🧪 VTOL精确控制测试")
        print("="*60)
        
        if not self.wait_for_position():
            return False
        
        # 测试目标点
        test_targets = [
            (100, 0, 20, "简单目标：东100米"),
            (0, 100, 20, "简单目标：北100米"), 
            (1600, 200, 20, "复杂目标：target_north"),
            (1600, -200, 20, "复杂目标：target_south"),
            (0, 0, 20, "返回原点"),
        ]
        
        success_count = 0
        
        for i, (target_x, target_y, target_z, description) in enumerate(test_targets, 1):
            print(f"\n🎯 测试 {i}: {description}")
            print(f"目标坐标: ({target_x}, {target_y}, {target_z})")
            
            start_time = time.time()
            success = self.precise_approach(target_x, target_y, target_z)
            elapsed_time = time.time() - start_time
            
            if success:
                success_count += 1
                print(f"✅ 测试 {i} 成功 (用时: {elapsed_time:.1f}s)")
            else:
                print(f"❌ 测试 {i} 失败 (用时: {elapsed_time:.1f}s)")
            
            # 测试间隔
            if i < len(test_targets):
                print("等待5秒后进行下一个测试...")
                time.sleep(5)
        
        # 测试总结
        print(f"\n📊 精确控制测试总结:")
        print(f"   成功率: {success_count}/{len(test_targets)} ({success_count/len(test_targets)*100:.1f}%)")
        print(f"   容忍度: {self.position_tolerance}m")
        print(f"   分阶段接近: {self.approach_stages}")
        
        return success_count == len(test_targets)

def main():
    try:
        controller = VTOLPrecisionController("standard_vtol", "0")
        controller.test_precision_control()
        
    except KeyboardInterrupt:
        print("\n用户中断测试")
    except Exception as e:
        print(f"\n❌ 测试异常: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
