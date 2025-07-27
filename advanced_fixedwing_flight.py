#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
改进版固定翼无人机自动飞行脚本
解决固定翼模式下的长距离导航问题
"""

import rospy
import time
import math
import threading
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from mavros_msgs.msg import State


class AdvancedFixedWingFlight:
    def __init__(self, vehicle_type="standard_vtol", vehicle_id="0"):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        
        # 目标位置 (person_red)
        self.target_x = 1495.000
        self.target_y = -105.000
        self.target_z = 20.0
        
        # 当前状态
        self.current_position = None
        self.current_state = None
        self.current_mode = ""
        self.is_armed = False
        
        # 飞行参数
        self.takeoff_height = 30.0
        self.cruise_height = 50.0
        self.waypoint_tolerance = 50.0  # 航点容忍度
        
        # 位置指令持续发布
        self.target_pose = Pose()
        self.should_publish = False
        
        print(f"初始化改进版固定翼自动飞行: {self.vehicle_type}_{self.vehicle_id}")
        print(f"目标: person_red ({self.target_x}, {self.target_y}, {self.target_z})")
        
        self.init_ros()

    def init_ros(self):
        """初始化ROS节点和通信"""
        rospy.init_node(f"{self.vehicle_type}_{self.vehicle_id}_advanced_flight")
        
        # 订阅者
        self.local_pose_sub = rospy.Subscriber(
            f"{self.vehicle_type}_{self.vehicle_id}/mavros/local_position/pose", 
            PoseStamped, self.local_pose_callback, queue_size=1)
        
        self.state_sub = rospy.Subscriber(
            f"{self.vehicle_type}_{self.vehicle_id}/mavros/state",
            State, self.state_callback, queue_size=1)
        
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

    def state_callback(self, msg):
        """状态回调函数"""
        self.current_state = msg
        self.current_mode = msg.mode
        self.is_armed = msg.armed

    def send_cmd(self, cmd_str):
        """发送xtdrone命令"""
        cmd_msg = String()
        cmd_msg.data = cmd_str
        self.cmd_pub.publish(cmd_msg)
        print(f"发送命令: {cmd_str}")

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
        
        print(f"更新目标位置: x={x:.1f}, y={y:.1f}, z={z:.1f}")

    def continuous_publish(self):
        """持续发布位置指令"""
        rate = rospy.Rate(20)  # 20Hz
        while self.should_publish and not rospy.is_shutdown():
            self.cmd_pose_pub.publish(self.target_pose)
            rate.sleep()

    def get_distance_to_target(self):
        """计算到目标点的距离"""
        if self.current_position is None:
            return float('inf')
        
        dx = self.target_x - self.current_position.x
        dy = self.target_y - self.current_position.y
        dz = self.target_z - self.current_position.z
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def get_horizontal_distance_to_point(self, x, y):
        """计算到指定点的水平距离"""
        if self.current_position is None:
            return float('inf')
        
        dx = x - self.current_position.x
        dy = y - self.current_position.y
        
        return math.sqrt(dx*dx + dy*dy)

    def wait_for_connection(self):
        """等待ROS连接"""
        print("等待位置信息...")
        while self.current_position is None and not rospy.is_shutdown():
            time.sleep(0.1)
        print(f"当前位置: ({self.current_position.x:.1f}, {self.current_position.y:.1f}, {self.current_position.z:.1f})")

    def wait_for_takeoff(self, target_height, timeout=60):
        """等待起飞到指定高度"""
        print(f"等待起飞到 {target_height}m...")
        start_time = time.time()
        
        while (self.current_position and 
               self.current_position.z < target_height - 3.0 and 
               time.time() - start_time < timeout):
            time.sleep(1)
            print(f"   当前高度: {self.current_position.z:.1f}m")
        
        if self.current_position.z >= target_height - 3.0:
            print(f"起飞完成! 当前高度: {self.current_position.z:.1f}m")
            return True
        else:
            print("起飞超时!")
            return False

    def wait_for_waypoint(self, target_x, target_y, tolerance=50.0, timeout=120):
        """等待到达航点"""
        print(f"等待到达航点 ({target_x:.1f}, {target_y:.1f})...")
        start_time = time.time()
        
        while (self.current_position and 
               self.get_horizontal_distance_to_point(target_x, target_y) > tolerance and 
               time.time() - start_time < timeout):
            time.sleep(2)
            current_dist = self.get_horizontal_distance_to_point(target_x, target_y)
            print(f"   当前位置: ({self.current_position.x:.1f}, {self.current_position.y:.1f})")
            print(f"   距离航点: {current_dist:.1f}m")
        
        final_dist = self.get_horizontal_distance_to_point(target_x, target_y)
        if final_dist <= tolerance:
            print(f"到达航点! 距离: {final_dist:.1f}m")
            return True
        else:
            print(f"航点超时! 当前距离: {final_dist:.1f}m")
            return False

    def print_status(self):
        """打印当前状态"""
        if self.current_position and self.current_state:
            dist_to_target = self.get_distance_to_target()
            print(f"状态监控:")
            print(f"  位置: ({self.current_position.x:.1f}, {self.current_position.y:.1f}, {self.current_position.z:.1f})")
            print(f"  模式: {self.current_mode}, 解锁: {self.is_armed}")
            print(f"  距离目标: {dist_to_target:.1f}m")

    def execute_mission(self):
        """执行改进的自动飞行任务"""
        print("\n开始执行改进版自动飞行任务...")
        print("="*60)
        
        # 记录起始位置
        start_x = self.current_position.x
        start_y = self.current_position.y
        
        # === 阶段1: 准备和起飞 ===
        print("阶段1: 准备和起飞")
        print("-" * 30)
        
        # 开始持续发布位置指令
        self.set_target_pose(start_x, start_y, 5.0)
        time.sleep(2)
        
        # 设置OFFBOARD模式
        self.send_cmd("OFFBOARD")
        time.sleep(2)
        
        # 解锁
        self.send_cmd("ARM")
        time.sleep(3)
        
        # 起飞
        self.set_target_pose(start_x, start_y, self.takeoff_height)
        if not self.wait_for_takeoff(self.takeoff_height):
            print("起飞失败，终止任务")
            return
        
        # === 阶段2: 切换固定翼模式 ===
        print("\n阶段2: 切换固定翼模式")
        print("-" * 30)
        
        self.send_cmd("plane")
        print("等待固定翼模式切换...")
        time.sleep(10)  # 增加等待时间
        
        self.print_status()
        
        # === 阶段3: 分阶段巡航 ===
        print("\n阶段3: 分阶段巡航")
        print("-" * 30)
        
        # 生成更合理的航点序列
        waypoints = self.generate_waypoints(start_x, start_y)
        
        for i, (wp_x, wp_y) in enumerate(waypoints):
            print(f"\n航点 {i+1}/{len(waypoints)}: ({wp_x:.1f}, {wp_y:.1f})")
            
            # 设置航点
            self.set_target_pose(wp_x, wp_y, self.cruise_height)
            
            # 等待到达航点（或超时）
            reached = self.wait_for_waypoint(wp_x, wp_y, self.waypoint_tolerance, timeout=60)
            
            if not reached:
                print(f"航点{i+1}超时，继续下一个航点")
            
            self.print_status()
        
        # === 阶段4: 最终接近 ===
        print("\n阶段4: 最终接近目标")
        print("-" * 30)
        
        # 直接飞向目标
        self.set_target_pose(self.target_x, self.target_y, self.cruise_height)
        time.sleep(20)
        
        # === 阶段5: 降落 ===
        print("\n阶段5: 降落")
        print("-" * 30)
        
        # 切换回多旋翼模式
        self.send_cmd("multirotor")
        time.sleep(5)
        
        # 降落到目标点
        self.set_target_pose(self.target_x, self.target_y, self.target_z)
        time.sleep(15)
        
        # 最终状态
        self.print_status()
        
        print("\n任务完成!")
        self.send_cmd("DISARM")

    def generate_waypoints(self, start_x, start_y):
        """生成更合理的航点序列"""
        waypoints = []
        
        # 计算总距离
        total_dx = self.target_x - start_x
        total_dy = self.target_y - start_y
        total_distance = math.sqrt(total_dx*total_dx + total_dy*total_dy)
        
        print(f"总距离: {total_distance:.1f}m")
        
        # 根据距离决定航点数量
        if total_distance > 1000:
            num_waypoints = 8
        elif total_distance > 500:
            num_waypoints = 5
        else:
            num_waypoints = 3
        
        # 生成等距航点
        for i in range(1, num_waypoints + 1):
            progress = i / num_waypoints
            wp_x = start_x + progress * total_dx
            wp_y = start_y + progress * total_dy
            waypoints.append((wp_x, wp_y))
        
        print(f"生成 {len(waypoints)} 个航点:")
        for i, (wp_x, wp_y) in enumerate(waypoints):
            print(f"  航点{i+1}: ({wp_x:.1f}, {wp_y:.1f})")
        
        return waypoints

    def run(self):
        """运行主程序"""
        try:
            self.wait_for_connection()
            self.execute_mission()
        except KeyboardInterrupt:
            print("\n收到中断信号，正在安全停止...")
            self.send_cmd("HOVER")
            self.send_cmd("DISARM")
        except Exception as e:
            print(f"发生错误: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.should_publish = False


if __name__ == '__main__':
    try:
        flight = AdvancedFixedWingFlight("standard_vtol", "0")
        flight.run()
    except rospy.ROSInterruptException:
        print("ROS中断")
