#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
VTOL无人机速度控制脚本
参考vtol_keyboard_control.py，从/vel_cmd订阅[x,y,z,omega]速度，并控制VTOL无人机

使用说明：
1. 支持多旋翼模式和固定翼模式的切换
2. 在多旋翼模式下使用Twist消息控制
3. 在固定翼模式下使用Pose消息控制
4. 支持模式转换功能

运行方式：
python3 vtol_control.py <vehicle_type> <vehicle_id> <control_type>
例如：python3 vtol_control.py standard_vtol 0 vel
"""

import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from std_msgs.msg import String, Float32MultiArray
import sys
import threading
import time
import math

class VTOLController:
    def __init__(self, vehicle_type="standard_vtol", vehicle_id=0, control_type="vel"):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        self.control_type = control_type
        
        # 速度和角速度限制
        self.MAX_LINEAR = 1000.0  # 与vtol_keyboard_control保持一致
        self.MAX_ANG_VEL = 0.5
        
        # 当前状态
        self.current_twist = Twist()
        self.current_pose = Pose()
        self.is_armed = False
        self.current_mode = "UNKNOWN"
        self.transition_state = 'multirotor'  # 'multirotor' 或 'plane'
        
        # 位置信息
        self.current_position = PoseStamped()
        self.position_available = False
        self.position_lock = threading.Lock()
        
        # ROS初始化
        rospy.init_node(f'{vehicle_type}_{vehicle_id}_vtol_controller')
        
        # 订阅速度命令话题
        self.vel_cmd_sub = rospy.Subscriber('/vel_cmd', Float32MultiArray, self.vel_cmd_callback)
        self.vel_cmd_twist_sub = rospy.Subscriber('/vel_cmd_twist', Twist, self.vel_cmd_twist_callback)
        self.pose_cmd_sub = rospy.Subscriber('/pose_cmd', Pose, self.pose_cmd_callback)
        
        # 订阅位置信息
        self.pose_sub = rospy.Subscriber(f'{vehicle_type}_{vehicle_id}/mavros/vision_pose/pose', 
                                        PoseStamped, self.position_callback)
        
        # 发布器初始化
        self.cmd_pose_enu_pub = rospy.Publisher(f'/xtdrone/{vehicle_type}_{vehicle_id}/cmd_pose_enu', Pose, queue_size=1)
        
        if control_type == 'vel':
            self.cmd_vel_flu_pub = rospy.Publisher(f'/xtdrone/{vehicle_type}_{vehicle_id}/cmd_vel_flu', Twist, queue_size=1)
        else:
            self.cmd_accel_flu_pub = rospy.Publisher(f'/xtdrone/{vehicle_type}_{vehicle_id}/cmd_accel_flu', Twist, queue_size=1)
        
        self.cmd_pub = rospy.Publisher(f'/xtdrone/{vehicle_type}_{vehicle_id}/cmd', String, queue_size=3)
        self.status_pub = rospy.Publisher('/vtol_controller/status', String, queue_size=1)
        
        # 初始化控制值
        self.forward = 0.0
        self.leftward = 0.0
        self.upward = 0.0
        self.angular = 0.0
        
        rospy.loginfo(f"VTOL控制器已初始化: {vehicle_type}_{vehicle_id}, 控制类型: {control_type}")
        rospy.loginfo(f"当前模式: {self.transition_state}")
        
        # 启动状态监控和命令发布线程
        self.monitor_thread = threading.Thread(target=self.status_monitor)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        # 定时发布命令
        self.cmd_timer = rospy.Timer(rospy.Duration(0.1), self.publish_commands)
    
    def vel_cmd_callback(self, msg):
        """
        处理来自/vel_cmd话题的速度命令 (Float32MultiArray格式)
        期望格式: [x, y, z, omega]
        """
        if len(msg.data) >= 4:
            self.forward = self.limit_value(msg.data[0], self.MAX_LINEAR)
            self.leftward = self.limit_value(msg.data[1], self.MAX_LINEAR)
            self.upward = self.limit_value(msg.data[2], self.MAX_LINEAR)
            self.angular = self.limit_value(msg.data[3], self.MAX_ANG_VEL)
            
            self.update_control_messages()
            rospy.loginfo(f"接收到速度命令: f={self.forward:.2f}, l={self.leftward:.2f}, u={self.upward:.2f}, a={self.angular:.2f}")
        else:
            rospy.logwarn(f"速度命令格式错误，期望4个元素，收到{len(msg.data)}个")
    
    def vel_cmd_twist_callback(self, msg):
        """
        处理来自/vel_cmd_twist话题的速度命令 (Twist格式)
        """
        self.forward = self.limit_value(msg.linear.x, self.MAX_LINEAR)
        self.leftward = self.limit_value(msg.linear.y, self.MAX_LINEAR)
        self.upward = self.limit_value(msg.linear.z, self.MAX_LINEAR)
        self.angular = self.limit_value(msg.angular.z, self.MAX_ANG_VEL)
        
        self.update_control_messages()
        rospy.loginfo(f"接收到Twist命令: f={self.forward:.2f}, l={self.leftward:.2f}, u={self.upward:.2f}, a={self.angular:.2f}")
    
    def pose_cmd_callback(self, msg):
        """
        处理位置命令（主要用于固定翼模式）
        """
        self.current_pose = msg
        rospy.loginfo(f"接收到位置命令")
    
    def position_callback(self, msg):
        """
        位置信息回调函数
        """
        with self.position_lock:
            self.current_position = msg
            self.position_available = True
    
    def limit_value(self, value, max_val):
        """限制数值范围"""
        return max(-max_val, min(max_val, value))
    
    def update_control_messages(self):
        """根据当前模式更新控制消息"""
        if self.transition_state == 'plane':
            # 固定翼模式：使用Pose消息
            self.current_pose.position.x = self.forward
            self.current_pose.position.y = self.leftward
            self.current_pose.position.z = self.upward
            self.current_pose.orientation.x = 0.0
            self.current_pose.orientation.y = 0.0
            self.current_pose.orientation.z = self.angular
            self.current_pose.orientation.w = 1.0
        else:
            # 多旋翼模式：使用Twist消息
            self.current_twist.linear.x = self.forward
            self.current_twist.linear.y = self.leftward
            self.current_twist.linear.z = self.upward
            self.current_twist.angular.x = 0.0
            self.current_twist.angular.y = 0.0
            self.current_twist.angular.z = self.angular
    
    def publish_commands(self, event):
        """定时发布控制命令"""
        if self.transition_state == 'plane':
            # 固定翼模式
            self.cmd_pose_enu_pub.publish(self.current_pose)
        else:
            # 多旋翼模式
            if self.control_type == 'vel':
                self.cmd_vel_flu_pub.publish(self.current_twist)
            else:
                self.cmd_accel_flu_pub.publish(self.current_twist)
    
    def get_current_position(self):
        """获取当前位置信息"""
        with self.position_lock:
            return (self.current_position.pose, self.position_available)
    
    def get_position_xyz(self):
        """获取当前位置的x,y,z坐标"""
        with self.position_lock:
            if self.position_available:
                pos = self.current_position.pose.position
                return (pos.x, pos.y, pos.z)
            return None
    
    def send_command(self, command):
        """发送控制命令"""
        cmd_msg = String()
        cmd_msg.data = command
        self.cmd_pub.publish(cmd_msg)
        rospy.loginfo(f"发送命令: {command}")
    
    def arm(self):
        """解锁"""
        self.send_command("ARM")
        self.is_armed = True
    
    def disarm(self):
        """上锁"""
        self.send_command("DISARM")
        self.is_armed = False
    
    def takeoff(self):
        """起飞"""
        self.send_command("AUTO.TAKEOFF")
    
    def land(self):
        """降落"""
        self.send_command("AUTO.LAND")
    
    def set_offboard_mode(self):
        """设置为OFFBOARD模式"""
        self.send_command("OFFBOARD")
        self.current_mode = "OFFBOARD"
    
    def set_hover_mode(self):
        """设置为悬停模式（多旋翼）"""
        if self.transition_state == 'multirotor':
            self.send_command("HOVER")
            self.current_mode = "HOVER"
            self.forward = 0.0
            self.leftward = 0.0
            self.upward = 0.0
            self.angular = 0.0
            self.update_control_messages()
        else:
            rospy.logwarn("固定翼模式下不支持悬停")
    
    def set_loiter_mode(self):
        """设置为巡航模式（固定翼）"""
        if self.transition_state == 'plane':
            self.send_command("loiter")
            self.current_mode = "LOITER"
        else:
            rospy.logwarn("多旋翼模式下不支持巡航")
    
    def set_idle_mode(self):
        """设置为空闲模式（固定翼）"""
        if self.transition_state == 'plane':
            self.send_command("idle")
            self.current_mode = "IDLE"
        else:
            self.set_hover_mode()
    
    def return_home(self):
        """返回起飞点"""
        self.send_command("AUTO.RTL")
        self.current_mode = "RTL"
    
    def transition_to_plane(self):
        """转换到固定翼模式"""
        if self.transition_state == 'multirotor':
            self.transition_state = 'plane'
            self.send_command("plane")
            rospy.loginfo("转换到固定翼模式")
            self.update_control_messages()
    
    def transition_to_multirotor(self):
        """转换到多旋翼模式"""
        if self.transition_state == 'plane':
            self.transition_state = 'multirotor'
            self.send_command("multirotor")
            rospy.loginfo("转换到多旋翼模式")
            self.update_control_messages()
    
    def emergency_stop(self):
        """紧急停止"""
        self.forward = 0.0
        self.leftward = 0.0
        self.upward = 0.0
        self.angular = 0.0
        self.update_control_messages()
        
        if self.transition_state == 'multirotor':
            self.set_hover_mode()
        else:
            self.set_loiter_mode()
        
        rospy.logwarn("紧急停止！所有控制值已清零")
    
    def status_monitor(self):
        """状态监控线程"""
        rate = rospy.Rate(2)  # 2Hz
        while not rospy.is_shutdown():
            pos_xyz = self.get_position_xyz()
            
            status_msg = String()
            status_data = f"Armed: {self.is_armed}, Mode: {self.current_mode}, Transition: {self.transition_state}, "
            
            if self.transition_state == 'plane':
                status_data += f"PoseCmd: [{self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}, " \
                              f"{self.current_pose.position.z:.2f}, {self.current_pose.orientation.z:.2f}]"
            else:
                status_data += f"VelCmd: [{self.current_twist.linear.x:.2f}, {self.current_twist.linear.y:.2f}, " \
                              f"{self.current_twist.linear.z:.2f}, {self.current_twist.angular.z:.2f}]"
            
            if pos_xyz:
                status_data += f", Pos: [{pos_xyz[0]:.2f}, {pos_xyz[1]:.2f}, {pos_xyz[2]:.2f}]"
            else:
                status_data += ", Pos: [N/A]"
            
            status_msg.data = status_data
            self.status_pub.publish(status_msg)
            rate.sleep()
    
    def get_status_info(self):
        """获取状态信息字符串"""
        pos_xyz = self.get_position_xyz()
        pos_str = f"x={pos_xyz[0]:.2f}, y={pos_xyz[1]:.2f}, z={pos_xyz[2]:.2f}" if pos_xyz else "N/A"
        
        return f"""
========== VTOL状态 ==========
类型: {self.vehicle_type}_{self.vehicle_id}
控制模式: {self.control_type}
转换状态: {self.transition_state}
是否解锁: {self.is_armed}
飞行模式: {self.current_mode}
位置信息可用: {self.position_available}
当前位置: {pos_str}
控制值: forward={self.forward:.2f}, leftward={self.leftward:.2f}, 
        upward={self.upward:.2f}, angular={self.angular:.2f}
=============================
"""

def main():
    """主函数"""
    if len(sys.argv) < 4:
        print("用法: python3 vtol_control.py <vehicle_type> <vehicle_id> <control_type>")
        print("例如: python3 vtol_control.py standard_vtol 0 vel")
        print("控制类型: vel (速度控制) 或 accel (加速度控制)")
        return
    
    vehicle_type = sys.argv[1]
    vehicle_id = int(sys.argv[2])
    control_type = sys.argv[3]
    
    # 创建控制器
    controller = VTOLController(vehicle_type, vehicle_id, control_type)
    
    print("=" * 50)
    print("VTOL无人机控制脚本已启动")
    print("=" * 50)
    print(controller.get_status_info())
    print("控制命令:")
    print("  发布到 /vel_cmd (Float32MultiArray): [x, y, z, omega]")
    print("  发布到 /vel_cmd_twist (Twist): 标准ROS Twist消息")
    print("  发布到 /pose_cmd (Pose): 位置命令（固定翼模式）")
    print()
    print("位置监控:")
    print(f"  订阅位置: {vehicle_type}_{vehicle_id}/mavros/vision_pose/pose")
    print("  注意: 需要运行get_local_pose.py来提供位置服务")
    print()
    print("可用的控制话题:")
    print(f"  解锁:     rostopic pub /vtol_cmd std_msgs/String 'ARM'")
    print(f"  上锁:     rostopic pub /vtol_cmd std_msgs/String 'DISARM'")
    print(f"  起飞:     rostopic pub /vtol_cmd std_msgs/String 'TAKEOFF'")
    print(f"  降落:     rostopic pub /vtol_cmd std_msgs/String 'LAND'")
    print(f"  OFFBOARD: rostopic pub /vtol_cmd std_msgs/String 'OFFBOARD'")
    print(f"  悬停:     rostopic pub /vtol_cmd std_msgs/String 'HOVER'")
    print(f"  巡航:     rostopic pub /vtol_cmd std_msgs/String 'LOITER'")
    print(f"  空闲:     rostopic pub /vtol_cmd std_msgs/String 'IDLE'")
    print(f"  返航:     rostopic pub /vtol_cmd std_msgs/String 'RTL'")
    print(f"  紧急停止: rostopic pub /vtol_cmd std_msgs/String 'EMERGENCY'")
    print(f"  转为固定翼: rostopic pub /vtol_cmd std_msgs/String 'TRANSITION_PLANE'")
    print(f"  转为多旋翼: rostopic pub /vtol_cmd std_msgs/String 'TRANSITION_MULTIROTOR'")
    print()
    print("速度命令示例:")
    print("  多旋翼模式: rostopic pub /vel_cmd std_msgs/Float32MultiArray '{data: [1.0, 0.0, 0.5, 0.2]}'")
    print("  固定翼模式: 使用位置控制")
    print()
    print("按 Ctrl+C 退出")
    print("=" * 50)
    
    # 订阅控制命令话题
    def control_cmd_callback(msg):
        command = msg.data.upper()
        if command == "ARM":
            controller.arm()
        elif command == "DISARM":
            controller.disarm()
        elif command == "TAKEOFF":
            controller.takeoff()
        elif command == "LAND":
            controller.land()
        elif command == "OFFBOARD":
            controller.set_offboard_mode()
        elif command == "HOVER":
            controller.set_hover_mode()
        elif command == "LOITER":
            controller.set_loiter_mode()
        elif command == "IDLE":
            controller.set_idle_mode()
        elif command == "RTL":
            controller.return_home()
        elif command == "EMERGENCY":
            controller.emergency_stop()
        elif command == "TRANSITION_PLANE":
            controller.transition_to_plane()
        elif command == "TRANSITION_MULTIROTOR":
            controller.transition_to_multirotor()
        else:
            rospy.logwarn(f"未知命令: {command}")
    
    rospy.Subscriber('/vtol_cmd', String, control_cmd_callback)
    
    try:
        # 主循环
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            rate.sleep()
    except KeyboardInterrupt:
        print("\n接收到退出信号，正在停止...")
        controller.emergency_stop()
        time.sleep(1)
        print("VTOL控制器已停止")

if __name__ == '__main__':
    main()