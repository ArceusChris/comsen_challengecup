#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
无人机速度控制脚本
参考multirotor_keyboard_control.py，从/vel_cmd订阅[x,y,z,omega]速度，并控制无人机

使用说明：
1. 便可以通过键盘控制1架iris的解锁/上锁(arm/disarm)，修改飞行模式，飞机速度等。
2. 使用v起飞利用的是takeoff飞行模式，相关参数（起飞速度、高度）要在rcS中设置。
3. 一般可以使用offboard模式起飞，这时起飞速度要大于0.3m/s才能起飞(即：upward velocity 需要大于0.3)。
4. 注意，飞机要先解锁才能起飞！飞到一定高度后可以切换为'hover'模式悬停，再运行自己的飞行脚本，或利用键盘控制飞机。

推荐起飞流程：
- 按i把向上速度加到0.3以上
- 再按b切offboard模式
- 最后按t解锁

运行方式：
python3 drone_control.py <multirotor_type> <multirotor_id> <control_type>
例如：python3 drone_control.py iris 0 vel
"""

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Float32MultiArray
import sys
import threading
import time

class DroneController:
    def __init__(self, multirotor_type="iris", multirotor_id=0, control_type="vel"):
        self.multirotor_type = multirotor_type
        self.multirotor_id = multirotor_id
        self.control_type = control_type
        
        # 速度限制
        self.MAX_LINEAR = 20.0
        self.MAX_ANG_VEL = 3.0
        
        # 当前速度状态
        self.current_twist = Twist()
        self.is_armed = False
        self.current_mode = "UNKNOWN"
        
        # 当前位置信息
        self.current_pose = PoseStamped()
        self.position_available = False  # 位置信息是否可用
        self.position_lock = threading.Lock()  # 位置数据的线程锁
        
        # ROS初始化
        rospy.init_node(f'{multirotor_type}_{multirotor_id}_velocity_controller')
        
        # 订阅速度命令话题 - 期待Float32MultiArray格式 [x, y, z, omega]
        self.vel_cmd_sub = rospy.Subscriber('/vel_cmd', Float32MultiArray, self.vel_cmd_callback)
        
        # 或者也支持Twist格式的速度命令
        self.vel_cmd_twist_sub = rospy.Subscriber('/vel_cmd_twist', Twist, self.vel_cmd_twist_callback)
        
        # 订阅位置信息 (参考get_local_pose.py的发布话题)
        self.pose_sub = rospy.Subscriber(f'{multirotor_type}_{multirotor_id}/mavros/vision_pose/pose', 
                                        PoseStamped, self.pose_callback)
        
        # 发布速度命令
        if control_type == 'vel':
            self.cmd_vel_pub = rospy.Publisher(f'/xtdrone/{multirotor_type}_{multirotor_id}/cmd_vel_flu', Twist, queue_size=1)
        else:
            self.cmd_vel_pub = rospy.Publisher(f'/xtdrone/{multirotor_type}_{multirotor_id}/cmd_accel_flu', Twist, queue_size=1)
        
        # 发布控制命令（解锁、模式切换等）
        self.cmd_pub = rospy.Publisher(f'/xtdrone/{multirotor_type}_{multirotor_id}/cmd', String, queue_size=3)
        
        # 状态发布（可选）
        self.status_pub = rospy.Publisher('/drone_controller/status', String, queue_size=1)
        
        rospy.loginfo(f"无人机控制器已初始化: {multirotor_type}_{multirotor_id}, 控制类型: {control_type}")
        rospy.loginfo("等待速度命令...")
        rospy.loginfo("订阅话题: /vel_cmd (Float32MultiArray格式: [x, y, z, omega])")
        rospy.loginfo("订阅话题: /vel_cmd_twist (Twist格式)")
        rospy.loginfo(f"订阅位置话题: {multirotor_type}_{multirotor_id}/mavros/vision_pose/pose")
        
        # 启动状态监控线程
        self.monitor_thread = threading.Thread(target=self.status_monitor)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        # 定时发布当前速度命令
        self.cmd_timer = rospy.Timer(rospy.Duration(0.1), self.publish_velocity_command)
    
    def vel_cmd_callback(self, msg):
        """
        处理来自/vel_cmd话题的速度命令 (Float32MultiArray格式)
        期望格式: [x, y, z, omega]
        """
        if len(msg.data) >= 4:
            # 限制速度范围
            x = max(-self.MAX_LINEAR, min(self.MAX_LINEAR, msg.data[0]))
            y = max(-self.MAX_LINEAR, min(self.MAX_LINEAR, msg.data[1]))
            z = max(-self.MAX_LINEAR, min(self.MAX_LINEAR, msg.data[2]))
            omega = max(-self.MAX_ANG_VEL, min(self.MAX_ANG_VEL, msg.data[3]))
            
            # 更新当前速度
            self.current_twist.linear.x = x
            self.current_twist.linear.y = y
            self.current_twist.linear.z = z
            self.current_twist.angular.x = 0.0
            self.current_twist.angular.y = 0.0
            self.current_twist.angular.z = omega
            
            rospy.loginfo(f"接收到速度命令: x={x:.2f}, y={y:.2f}, z={z:.2f}, omega={omega:.2f}")
        else:
            rospy.logwarn(f"速度命令格式错误，期望4个元素，收到{len(msg.data)}个")
    
    def vel_cmd_twist_callback(self, msg):
        """
        处理来自/vel_cmd_twist话题的速度命令 (Twist格式)
        """
        # 限制速度范围
        x = max(-self.MAX_LINEAR, min(self.MAX_LINEAR, msg.linear.x))
        y = max(-self.MAX_LINEAR, min(self.MAX_LINEAR, msg.linear.y))
        z = max(-self.MAX_LINEAR, min(self.MAX_LINEAR, msg.linear.z))
        omega = max(-self.MAX_ANG_VEL, min(self.MAX_ANG_VEL, msg.angular.z))
        
        # 更新当前速度
        self.current_twist.linear.x = x
        self.current_twist.linear.y = y
        self.current_twist.linear.z = z
        self.current_twist.angular.x = 0.0
        self.current_twist.angular.y = 0.0
        self.current_twist.angular.z = omega
        
        rospy.loginfo(f"接收到Twist速度命令: x={x:.2f}, y={y:.2f}, z={z:.2f}, omega={omega:.2f}")
    
    def pose_callback(self, msg):
        """
        位置信息回调函数
        """
        with self.position_lock:
            self.current_pose = msg
            self.position_available = True
    
    def publish_velocity_command(self, event):
        """
        定时发布速度命令到无人机
        """
        self.cmd_vel_pub.publish(self.current_twist)
    
    def get_current_position(self):
        """
        获取当前位置信息
        返回: (position, available) 元组
        """
        with self.position_lock:
            return (self.current_pose.pose, self.position_available)
    
    def get_position_xyz(self):
        """
        获取当前位置的x,y,z坐标
        返回: (x, y, z) 或 None (如果位置不可用)
        """
        with self.position_lock:
            if self.position_available:
                pos = self.current_pose.pose.position
                return (pos.x, pos.y, pos.z)
            return None
    
    def send_command(self, command):
        """
        发送控制命令（解锁、模式切换等）
        """
        cmd_msg = String()
        cmd_msg.data = command
        self.cmd_pub.publish(cmd_msg)
        rospy.loginfo(f"发送命令: {command}")
    
    def arm(self):
        """解锁无人机"""
        self.send_command("ARM")
        self.is_armed = True
    
    def disarm(self):
        """上锁无人机"""
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
        """设置为悬停模式"""
        self.send_command("HOVER")
        self.current_mode = "HOVER"
        # 悬停时清零速度
        self.current_twist = Twist()
    
    def return_home(self):
        """返回起飞点"""
        self.send_command("AUTO.RTL")
        self.current_mode = "RTL"
    
    def emergency_stop(self):
        """紧急停止"""
        self.current_twist = Twist()
        self.set_hover_mode()
        rospy.logwarn("紧急停止！所有速度已清零，切换到悬停模式")
    
    def status_monitor(self):
        """
        状态监控线程
        """
        rate = rospy.Rate(2)  # 2Hz
        while not rospy.is_shutdown():
            # 获取位置信息
            pos_xyz = self.get_position_xyz()
            
            status_msg = String()
            status_data = f"Armed: {self.is_armed}, Mode: {self.current_mode}, " \
                         f"CmdVel: [{self.current_twist.linear.x:.2f}, {self.current_twist.linear.y:.2f}, " \
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
========== 无人机状态 ==========
类型: {self.multirotor_type}_{self.multirotor_id}
控制模式: {self.control_type}
是否解锁: {self.is_armed}
飞行模式: {self.current_mode}
位置信息可用: {self.position_available}
当前位置: {pos_str}
命令速度: x={self.current_twist.linear.x:.2f}, y={self.current_twist.linear.y:.2f}, 
         z={self.current_twist.linear.z:.2f}, omega={self.current_twist.angular.z:.2f}
=============================
"""

    def auto_takeoff_sequence(self):
        """
        自动执行推荐的起飞流程：
        1. 设置上升速度到0.3以上
        2. 切换到offboard模式
        3. 解锁无人机
        """
        rospy.loginfo("开始自动起飞流程...")
        
        # 步骤1: 设置上升速度到0.3以上
        rospy.loginfo("步骤1: 设置上升速度到0.5m/s")
        self.current_twist.linear.x = 0.0
        self.current_twist.linear.y = 0.0
        self.current_twist.linear.z = 0.5  # 上升速度0.5m/s
        self.current_twist.angular.x = 0.0
        self.current_twist.angular.y = 0.0
        self.current_twist.angular.z = 0.0
        
        # 等待2秒确保速度命令生效
        rospy.sleep(2.0)
        
        # 步骤2: 切换到offboard模式
        rospy.loginfo("步骤2: 切换到OFFBOARD模式")
        self.set_offboard_mode()
        rospy.sleep(1.0)
        
        # 步骤3: 解锁无人机
        rospy.loginfo("步骤3: 解锁无人机")
        self.arm()
        rospy.sleep(1.0)
        
        rospy.loginfo("自动起飞流程完成！无人机应该开始上升")
        rospy.loginfo("可以通过/vel_cmd话题发送新的速度命令来控制无人机")
    
    def auto_landing_sequence(self):
        """
        自动降落流程：
        1. 切换到悬停模式
        2. 缓慢下降
        3. 降落并上锁
        """
        rospy.loginfo("开始自动降落流程...")
        
        # 步骤1: 悬停
        rospy.loginfo("步骤1: 悬停稳定")
        self.set_hover_mode()
        rospy.sleep(2.0)
        
        # 步骤2: 缓慢下降
        rospy.loginfo("步骤2: 缓慢下降")
        self.current_twist.linear.x = 0.0
        self.current_twist.linear.y = 0.0
        self.current_twist.linear.z = -0.3  # 下降速度0.3m/s
        self.current_twist.angular.z = 0.0
        
        rospy.loginfo("自动降落流程完成！")

def main():
    """主函数"""
    if len(sys.argv) < 4:
        print("用法: python3 drone_control.py <multirotor_type> <multirotor_id> <control_type> [auto_takeoff]")
        print("例如: python3 drone_control.py iris 0 vel")
        print("     python3 drone_control.py iris 0 vel auto_takeoff  # 自动起飞")
        print("控制类型: vel (速度控制) 或 accel (加速度控制)")
        print("auto_takeoff: 可选参数，启动时自动执行起飞流程")
        return
    
    multirotor_type = sys.argv[1]
    multirotor_id = int(sys.argv[2])
    control_type = sys.argv[3]
    auto_takeoff = len(sys.argv) > 4 and sys.argv[4].lower() == 'auto_takeoff'
    
    # 创建控制器
    controller = DroneController(multirotor_type, multirotor_id, control_type)
    
    print("=" * 50)
    print("无人机速度控制脚本已启动")
    if auto_takeoff:
        print("*** 自动起飞模式已启用 ***")
    print("=" * 50)
    print(controller.get_status_info())
    print("控制命令:")
    print("  发布到 /vel_cmd (Float32MultiArray): [x, y, z, omega]")
    print("  发布到 /vel_cmd_twist (Twist): 标准ROS Twist消息")
    print()
    print("位置监控:")
    print(f"  订阅位置: {multirotor_type}_{multirotor_id}/mavros/vision_pose/pose")
    print("  注意: 需要运行get_local_pose.py来提供位置服务")
    print()
    print("可用的控制话题:")
    print(f"  解锁:     rostopic pub /drone_cmd std_msgs/String 'ARM'")
    print(f"  上锁:     rostopic pub /drone_cmd std_msgs/String 'DISARM'")
    print(f"  起飞:     rostopic pub /drone_cmd std_msgs/String 'TAKEOFF'")
    print(f"  降落:     rostopic pub /drone_cmd std_msgs/String 'LAND'")
    print(f"  OFFBOARD: rostopic pub /drone_cmd std_msgs/String 'OFFBOARD'")
    print(f"  悬停:     rostopic pub /drone_cmd std_msgs/String 'HOVER'")
    print(f"  返航:     rostopic pub /drone_cmd std_msgs/String 'RTL'")
    print(f"  紧急停止: rostopic pub /drone_cmd std_msgs/String 'EMERGENCY'")
    print(f"  自动起飞: rostopic pub /drone_cmd std_msgs/String 'AUTO_TAKEOFF'")
    print(f"  自动降落: rostopic pub /drone_cmd std_msgs/String 'AUTO_LAND'")
    print()
    print("速度命令示例:")
    print("  rostopic pub /vel_cmd std_msgs/Float32MultiArray '{data: [1.0, 0.0, 0.5, 0.2]}'")
    print("  # [前进1m/s, 左移0m/s, 上升0.5m/s, 逆时针旋转0.2rad/s]")
    print()
    
    if auto_takeoff:
        print("!!! 5秒后将自动执行起飞流程 !!!")
        print("如需取消，请按 Ctrl+C")
        for i in range(5, 0, -1):
            print(f"倒计时: {i} 秒...")
            time.sleep(1)
    else:
        print("手动控制模式，需要手动发送控制命令")
    
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
        elif command == "RTL":
            controller.return_home()
        elif command == "EMERGENCY":
            controller.emergency_stop()
        elif command == "AUTO_TAKEOFF":
            # 在新线程中执行自动起飞，避免阻塞回调
            threading.Thread(target=controller.auto_takeoff_sequence, daemon=True).start()
        elif command == "AUTO_LAND":
            # 在新线程中执行自动降落，避免阻塞回调
            threading.Thread(target=controller.auto_landing_sequence, daemon=True).start()
        else:
            rospy.logwarn(f"未知命令: {command}")
    
    rospy.Subscriber('/drone_cmd', String, control_cmd_callback)
    
    # 如果启用自动起飞，则执行起飞流程
    if auto_takeoff:
        try:
            controller.auto_takeoff_sequence()
        except KeyboardInterrupt:
            print("\n自动起飞已取消")
            controller.emergency_stop()
            return
    
    try:
        # 主循环
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            rate.sleep()
    except KeyboardInterrupt:
        print("\n接收到退出信号，正在停止...")
        controller.emergency_stop()
        time.sleep(1)
        print("无人机控制器已停止")

if __name__ == '__main__':
    main()
