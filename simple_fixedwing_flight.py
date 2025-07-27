#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
简化版固定翼无人机自动飞行脚本
使用xtdrone的命令接口，参考vtol_communication_enhanced.py
目标：让standard_vtol_0飞到person_red位置
"""

import rospy
import time
import math
import threading
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String


class SimpleFixedWingFlight:
    def __init__(self, vehicle_type="standard_vtol", vehicle_id="0"):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        
        # 目标位置 (person_red)
        self.target_x = 1495.000
        self.target_y = -105.000
        self.target_z = 20.0  # 降低目标高度到20米
        
        # 当前状态
        self.current_position = None
        self.current_yaw = 0
        
        # 飞行参数
        self.takeoff_height = 30.0
        self.cruise_height = 50.0
        
        # 位置指令持续发布
        self.target_pose = Pose()
        self.should_publish = False
        
        print(f"初始化简化版固定翼自动飞行: {self.vehicle_type}_{self.vehicle_id}")
        print(f"目标: person_red ({self.target_x}, {self.target_y}, {self.target_z})")
        
        self.init_ros()

    def init_ros(self):
        """初始化ROS节点和通信"""
        rospy.init_node(f"{self.vehicle_type}_{self.vehicle_id}_simple_flight")
        
        # 订阅者
        self.local_pose_sub = rospy.Subscriber(
            f"{self.vehicle_type}_{self.vehicle_id}/mavros/local_position/pose", 
            PoseStamped, self.local_pose_callback, queue_size=1)
        
        # 发布者 - 使用xtdrone命令接口
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
        
        # 简化的yaw处理
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = math.sin(yaw / 2.0)
        pose_msg.orientation.w = math.cos(yaw / 2.0)
        
        self.cmd_pose_pub.publish(pose_msg)
        print(f"位置指令: x={x:.1f}, y={y:.1f}, z={z:.1f}")

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
            # 启动持续发布线程
            publish_thread = threading.Thread(target=self.continuous_publish)
            publish_thread.daemon = True
            publish_thread.start()
            print(f"开始持续发布位置指令: x={x:.1f}, y={y:.1f}, z={z:.1f}")

    def continuous_publish(self):
        """持续发布位置指令"""
        rate = rospy.Rate(20)  # 20Hz
        while self.should_publish and not rospy.is_shutdown():
            try:
                self.cmd_pose_pub.publish(self.target_pose)
                rate.sleep()
            except rospy.ROSInterruptException:
                # ROS关闭时正常退出
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

    def execute_mission(self):
        """执行简化的自动飞行任务"""
        print("\n开始执行自动飞行任务...")
        print("="*50)
        
        # 1. 发送初始位置指令并开始持续发布
        print("1. 开始持续发布位置指令...")
        start_x = self.current_position.x
        start_y = self.current_position.y
        self.set_target_pose(start_x, start_y, 5.0)  # 先给一个较低的目标高度
        time.sleep(2)
        
        # 2. 设置OFFBOARD模式
        print("2. 设置OFFBOARD模式...")
        self.send_cmd("OFFBOARD")
        time.sleep(2)
        
        # 3. 解锁无人机
        print("3. 解锁无人机...")
        self.send_cmd("ARM")
        time.sleep(3)
        
        # 4. 逐步起飞
        print(f"4. 起飞到 {self.takeoff_height}m...")
        
        # 逐步上升，更新持续发布的目标
        for height in [10, 20, self.takeoff_height]:
            print(f"   设置目标高度: {height}m")
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
        
        # 5. 切换到固定翼模式
        print("5. 切换到固定翼模式...")
        self.send_cmd("plane")
        time.sleep(5)
        
        # 6. 巡航到目标
        print("6. 开始巡航...")
        self.cruise_to_target()
        
        # 7. 降落
        print("7. 准备降落...")
        self.send_cmd("multirotor")  # 切换回多旋翼模式
        time.sleep(3)
        
        # 精确降落到目标点
        self.send_pose_cmd(self.target_x, self.target_y, self.target_z)
        time.sleep(10)
        
        print("8. 任务完成!")
        self.should_publish = False  # 停止持续发布
        time.sleep(1)  # 给线程时间停止
        self.send_cmd("DISARM")

    def cruise_to_target(self):
        """巡航到目标点"""
        print(f"巡航目标: ({self.target_x}, {self.target_y}, {self.cruise_height})")
        
        # 分阶段飞向目标
        current_x = self.current_position.x
        current_y = self.current_position.y
        
        # 计算中间点
        steps = 5
        for i in range(1, steps + 1):
            progress = i / steps
            intermediate_x = current_x + progress * (self.target_x - current_x)
            intermediate_y = current_y + progress * (self.target_y - current_y)
            
            print(f"中间点 {i}/{steps}: ({intermediate_x:.1f}, {intermediate_y:.1f}, {self.cruise_height})")
            self.send_pose_cmd(intermediate_x, intermediate_y, self.cruise_height)
            
            # 等待接近中间点
            time.sleep(8)
            
            if self.current_position:
                current_dist = self.get_distance_to_target()
                print(f"   当前位置: ({self.current_position.x:.1f}, {self.current_position.y:.1f}, {self.current_position.z:.1f})")
                print(f"   距离目标: {current_dist:.1f}m")
        
        # 最终精确飞向目标
        print("最终接近目标...")
        for _ in range(3):
            self.send_pose_cmd(self.target_x, self.target_y, self.cruise_height)
            time.sleep(5)
            
            if self.current_position:
                horizontal_dist = self.get_horizontal_distance()
                print(f"   水平距离目标: {horizontal_dist:.1f}m")
                
                if horizontal_dist < 20.0:
                    print("已接近目标区域!")
                    break

    def run(self):
        """运行主程序"""
        try:
            self.wait_for_connection()
            self.execute_mission()
        except KeyboardInterrupt:
            print("\n收到中断信号，正在停止...")
            self.send_cmd("HOVER")
            self.send_cmd("DISARM")
        except Exception as e:
            print(f"发生错误: {e}")
            import traceback
            traceback.print_exc()


if __name__ == '__main__':
    try:
        flight = SimpleFixedWingFlight("standard_vtol", "0")
        flight.run()
    except rospy.ROSInterruptException:
        print("ROS中断")
