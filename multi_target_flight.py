#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
多目标点固定翼无人机自动飞行脚本
支持飞行到多个目标点：person_red, zhihang/downtown等
基于成功的simple_fixedwing_flight.py改进
"""

import rospy
import time
import math
import threading
import sys
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String


class MultiTargetFixedWingFlight:
    def __init__(self, vehicle_type="standard_vtol", vehicle_id="0"):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        
        # 预定义目标点
        self.targets = {
            'person_red': {
                'position': (1495.000, -105.000, 20.0),
                'description': 'Person Red位置'
            },
            'person_yellow': {
                'position': (1492.006, 82.637, 20.0),
                'description': 'Person Yellow位置'
            },
            'person_white': {
                'position': (1497.994, -39.913, 20.0),
                'description': 'Person White位置'
            },
            'zhihang_downtown': {
                'position': (1200.0, 0.0, 15.0),  # 稍微提高高度避免地面
                'description': 'Zhihang Downtown位置'
            },
            'home': {
                'position': (0.0, 0.0, 10.0),
                'description': '起飞点/Home位置'
            }
        }
        
        # 当前目标
        self.current_target = None
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 20.0
        
        # 当前状态
        self.current_position = None
        self.current_yaw = 0
        
        # 飞行参数
        self.takeoff_height = 30.0
        self.cruise_height = 50.0
        self.min_cruise_height = 30.0  # 最小巡航高度
        
        # 位置指令持续发布
        self.target_pose = Pose()
        self.should_publish = False
        
        print(f"初始化多目标点固定翼自动飞行: {self.vehicle_type}_{self.vehicle_id}")
        print("可用目标点:")
        for name, target in self.targets.items():
            pos = target['position']
            print(f"  {name}: {target['description']} -> ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
        
        self.init_ros()

    def init_ros(self):
        """初始化ROS节点和通信"""
        rospy.init_node(f"{self.vehicle_type}_{self.vehicle_id}_multi_target_flight")
        
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
        
        print("ROS通信初始化完成")

    def local_pose_callback(self, msg):
        """位置回调函数"""
        self.current_position = msg.pose.position

    def set_target(self, target_name):
        """设置目标点"""
        if target_name not in self.targets:
            print(f"错误：未知目标点 '{target_name}'")
            return False
        
        self.current_target = target_name
        pos = self.targets[target_name]['position']
        self.target_x, self.target_y, self.target_z = pos
        
        # 确保巡航高度不低于目标高度
        self.cruise_height = max(self.target_z + 20, self.min_cruise_height)
        
        print(f"设置目标: {target_name} - {self.targets[target_name]['description']}")
        print(f"目标位置: ({self.target_x:.1f}, {self.target_y:.1f}, {self.target_z:.1f})")
        print(f"巡航高度: {self.cruise_height:.1f}m")
        return True

    def send_cmd(self, cmd_str):
        """发送xtdrone命令"""
        cmd_msg = String()
        cmd_msg.data = cmd_str
        self.cmd_pub.publish(cmd_msg)
        print(f"发送命令: {cmd_str}")

    def send_pose_cmd(self, x, y, z, yaw=0.0):
        """发送位置命令"""
        pose_msg = Pose()
        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.position.z = z
        
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = math.sin(yaw / 2.0)
        pose_msg.orientation.w = math.cos(yaw / 2.0)
        
        self.cmd_pose_pub.publish(pose_msg)

    def set_target_pose(self, x, y, z, yaw=0.0):
        """设置目标位置并开始持续发布"""
        self.target_pose.position.x = x
        self.target_pose.position.y = y
        self.target_pose.position.z = z
        self.target_pose.orientation.x = 0.0
        self.target_pose.orientation.y = 0.0
        self.target_pose.orientation.z = math.sin(yaw / 2.0)
        self.target_pose.orientation.w = math.cos(yaw / 2.0)
        
        if not self.should_publish:
            self.should_publish = True
            publish_thread = threading.Thread(target=self.continuous_publish)
            publish_thread.daemon = True
            publish_thread.start()
            print(f"开始持续发布位置指令: x={x:.1f}, y={y:.1f}, z={z:.1f}")
        else:
            print(f"更新目标位置: x={x:.1f}, y={y:.1f}, z={z:.1f}")

    def continuous_publish(self):
        """持续发布位置指令"""
        rate = rospy.Rate(20)  # 20Hz
        while self.should_publish and not rospy.is_shutdown():
            try:
                self.cmd_pose_pub.publish(self.target_pose)
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def get_distance_to_target(self):
        """计算到目标点的距离"""
        if self.current_position is None:
            return float('inf')
        
        dx = self.target_x - self.current_position.x
        dy = self.target_y - self.current_position.y
        dz = self.target_z - self.current_position.z
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def get_horizontal_distance(self):
        """计算水平距离"""
        if self.current_position is None:
            return float('inf')
        
        dx = self.target_x - self.current_position.x
        dy = self.target_y - self.current_position.y
        
        return math.sqrt(dx*dx + dy*dy)

    def wait_for_connection(self):
        """等待ROS连接"""
        print("等待位置信息...")
        while self.current_position is None and not rospy.is_shutdown():
            time.sleep(0.1)
        print(f"当前位置: ({self.current_position.x:.1f}, {self.current_position.y:.1f}, {self.current_position.z:.1f})")

    def takeoff_sequence(self):
        """起飞序列"""
        print(f"\n开始起飞序列到 {self.takeoff_height}m...")
        print("="*50)
        
        # 1. 开始持续发布位置指令
        start_x = self.current_position.x
        start_y = self.current_position.y
        self.set_target_pose(start_x, start_y, 5.0)
        time.sleep(2)
        
        # 2. 设置OFFBOARD模式
        print("设置OFFBOARD模式...")
        self.send_cmd("OFFBOARD")
        time.sleep(2)
        
        # 3. 解锁无人机
        print("解锁无人机...")
        self.send_cmd("ARM")
        time.sleep(3)
        
        # 4. 逐步起飞
        for height in [10, 20, self.takeoff_height]:
            print(f"   目标高度: {height}m")
            self.set_target_pose(start_x, start_y, height)
            time.sleep(4)
            if self.current_position:
                print(f"   当前高度: {self.current_position.z:.1f}m")
        
        # 等待到达起飞高度
        print("等待起飞完成...")
        timeout = 30
        start_time = time.time()
        while (self.current_position and self.current_position.z < self.takeoff_height - 5.0 and 
               time.time() - start_time < timeout):
            time.sleep(1)
            print(f"   当前高度: {self.current_position.z:.1f}m")
        
        if self.current_position and self.current_position.z < self.takeoff_height - 5.0:
            print("起飞超时，但继续任务...")
        else:
            print("起飞完成!")
        
        return True

    def fly_to_target(self, target_name):
        """飞向指定目标"""
        if not self.set_target(target_name):
            return False
        
        print(f"\n开始飞向目标: {target_name}")
        print("="*50)
        
        # 1. 切换到固定翼模式
        print("切换到固定翼模式...")
        self.send_cmd("plane")
        time.sleep(5)
        
        # 2. 巡航到目标
        print("开始巡航...")
        self.cruise_to_target()
        
        # 3. 切换回多旋翼模式准备精确定位
        print("切换回多旋翼模式...")
        self.send_cmd("multirotor")
        time.sleep(3)
        
        # 4. 精确飞向目标点
        print("精确接近目标...")
        self.set_target_pose(self.target_x, self.target_y, self.target_z)
        time.sleep(8)
        
        # 检查到达情况
        final_distance = self.get_distance_to_target()
        print(f"到达目标 {target_name}，最终距离: {final_distance:.2f}m")
        
        if final_distance < 5.0:
            print(f"✅ 成功到达 {target_name}!")
            return True
        else:
            print(f"⚠️  接近 {target_name}，但距离较远")
            return False

    def cruise_to_target(self):
        """巡航到目标点"""
        if self.current_position is None:
            return
        
        current_x = self.current_position.x
        current_y = self.current_position.y
        
        # 计算总距离
        total_distance = math.sqrt((self.target_x - current_x)**2 + (self.target_y - current_y)**2)
        print(f"总巡航距离: {total_distance:.1f}m")
        
        # 根据距离动态调整步数
        if total_distance > 1000:
            steps = 8
        elif total_distance > 500:
            steps = 5
        else:
            steps = 3
        
        # 分阶段飞向目标
        for i in range(1, steps + 1):
            progress = i / steps
            intermediate_x = current_x + progress * (self.target_x - current_x)
            intermediate_y = current_y + progress * (self.target_y - current_y)
            
            print(f"中间点 {i}/{steps}: ({intermediate_x:.1f}, {intermediate_y:.1f}, {self.cruise_height})")
            self.send_pose_cmd(intermediate_x, intermediate_y, self.cruise_height)
            
            # 根据距离调整等待时间
            wait_time = max(6, min(12, total_distance / 200))
            time.sleep(wait_time)
            
            if self.current_position:
                current_dist = self.get_distance_to_target()
                print(f"   当前位置: ({self.current_position.x:.1f}, {self.current_position.y:.1f}, {self.current_position.z:.1f})")
                print(f"   距离目标: {current_dist:.1f}m")

    def execute_mission(self, target_sequence):
        """执行多目标点任务"""
        print(f"\n🚁 开始多目标点飞行任务")
        print(f"任务序列: {' -> '.join(target_sequence)}")
        print("="*60)
        
        # 起飞
        if not self.takeoff_sequence():
            print("❌ 起飞失败，任务终止")
            return False
        
        # 依次飞向各个目标点
        for i, target_name in enumerate(target_sequence):
            print(f"\n📍 任务 {i+1}/{len(target_sequence)}: 飞向 {target_name}")
            
            success = self.fly_to_target(target_name)
            
            if success:
                print(f"✅ 完成目标 {target_name}")
            else:
                print(f"⚠️  目标 {target_name} 未完全到达")
            
            # 在目标点停留一下
            if i < len(target_sequence) - 1:  # 不是最后一个目标
                print("在目标点停留5秒...")
                time.sleep(5)
        
        # 任务完成
        print("\n🎉 所有目标点任务完成!")
        self.should_publish = False
        time.sleep(1)
        self.send_cmd("DISARM")
        
        return True

    def run_interactive(self):
        """交互式运行模式"""
        self.wait_for_connection()
        
        while not rospy.is_shutdown():
            print("\n" + "="*50)
            print("多目标点固定翼无人机控制系统")
            print("="*50)
            print("可用目标点:")
            for i, (name, target) in enumerate(self.targets.items(), 1):
                pos = target['position']
                print(f"  {i}. {name}: {target['description']} -> ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
            
            print("\n选项:")
            print("  s <target_name>  - 飞向单个目标点")
            print("  m <target1> <target2> ... - 飞向多个目标点")
            print("  q - 退出")
            
            try:
                cmd = input("\n请输入命令: ").strip().split()
                if not cmd:
                    continue
                
                if cmd[0] == 'q':
                    break
                elif cmd[0] == 's' and len(cmd) == 2:
                    target = cmd[1]
                    if target in self.targets:
                        self.takeoff_sequence()
                        self.fly_to_target(target)
                        self.should_publish = False
                        time.sleep(1)
                        self.send_cmd("DISARM")
                    else:
                        print(f"未知目标: {target}")
                elif cmd[0] == 'm' and len(cmd) > 1:
                    targets = cmd[1:]
                    invalid_targets = [t for t in targets if t not in self.targets]
                    if invalid_targets:
                        print(f"未知目标: {invalid_targets}")
                    else:
                        self.execute_mission(targets)
                else:
                    print("无效命令")
                    
            except KeyboardInterrupt:
                break
            except EOFError:
                break
        
        print("\n程序退出")

    def run_preset_mission(self, targets):
        """运行预设任务"""
        self.wait_for_connection()
        return self.execute_mission(targets)


def main():
    try:
        flight = MultiTargetFixedWingFlight("standard_vtol", "0")
        
        if len(sys.argv) > 1:
            # 命令行模式
            targets = sys.argv[1:]
            flight.run_preset_mission(targets)
        else:
            # 交互模式
            flight.run_interactive()
            
    except rospy.ROSInterruptException:
        print("ROS中断")
    except KeyboardInterrupt:
        print("\n用户中断")


if __name__ == '__main__':
    main()
