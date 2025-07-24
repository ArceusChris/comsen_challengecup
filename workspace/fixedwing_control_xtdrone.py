#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
固定翼无人机控制脚本 (兼容xtdrone接口)
基于drone_control.py修改，适配固定翼无人机特性

固定翼特点：
1. 需要保持最小前进速度以维持升力
2. 不能悬停，必须持续前进
3. 起飞需要足够的前进速度
4. 转弯通过倾斜和偏航实现

推荐起飞流程：
- 设置前进速度到15m/s以上
- 设置上升速度到2m/s
- 切换到OFFBOARD模式
- 解锁无人机

运行方式：
python3 fixedwing_control_xtdrone.py <aircraft_type> <aircraft_id> <control_type> [auto_takeoff]
例如：python3 fixedwing_control_xtdrone.py standard_vtol 0 vel auto_takeoff
"""

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32MultiArray
import sys
import threading
import time

class FixedwingController:
    def __init__(self, aircraft_type="standard_vtol", aircraft_id=0, control_type="vel"):
        self.aircraft_type = aircraft_type
        self.aircraft_id = aircraft_id
        self.control_type = control_type
        
        # 固定翼特定的速度限制
        self.MAX_FORWARD_SPEED = 30.0   # 最大前进速度
        self.MIN_FORWARD_SPEED = 12.0   # 最小前进速度（保持升力）
        self.MAX_CLIMB_RATE = 5.0       # 最大爬升率
        self.MAX_YAW_RATE = 1.0         # 最大偏航角速度
        self.CRUISE_SPEED = 18.0        # 巡航速度
        
        # 当前速度状态
        self.current_twist = Twist()
        self.is_armed = False
        self.current_mode = "UNKNOWN"
        self.is_flying = False
        
        # 默认设置巡航速度
        self.current_twist.linear.x = self.CRUISE_SPEED
        
        # ROS初始化
        rospy.init_node(f'{aircraft_type}_{aircraft_id}_fixedwing_controller')
        
        # 订阅速度命令话题 - 兼容原有格式，但适配固定翼
        self.vel_cmd_sub = rospy.Subscriber('/vel_cmd', Float32MultiArray, self.vel_cmd_callback)
        self.vel_cmd_twist_sub = rospy.Subscriber('/vel_cmd_twist', Twist, self.vel_cmd_twist_callback)
        
        # 也支持固定翼专用话题
        self.fixedwing_vel_cmd_sub = rospy.Subscriber('/fixedwing_vel_cmd', Float32MultiArray, self.fixedwing_vel_cmd_callback)
        
        # 发布速度命令 - 使用xtdrone格式
        if control_type == 'vel':
            self.cmd_vel_pub = rospy.Publisher(f'/xtdrone/{aircraft_type}_{aircraft_id}/cmd_vel_flu', Twist, queue_size=1)
        else:
            self.cmd_vel_pub = rospy.Publisher(f'/xtdrone/{aircraft_type}_{aircraft_id}/cmd_accel_flu', Twist, queue_size=1)
        
        # 发布控制命令
        self.cmd_pub = rospy.Publisher(f'/xtdrone/{aircraft_type}_{aircraft_id}/cmd', String, queue_size=3)
        
        # 状态发布
        self.status_pub = rospy.Publisher('/fixedwing_controller/status', String, queue_size=1)
        
        rospy.loginfo(f"固定翼控制器已初始化: {aircraft_type}_{aircraft_id}, 控制类型: {control_type}")
        rospy.loginfo("订阅话题: /vel_cmd (兼容多旋翼格式，但会适配固定翼)")
        rospy.loginfo("订阅话题: /fixedwing_vel_cmd (固定翼专用格式: [forward_speed, climb_rate, yaw_rate])")
        
        # 启动状态监控线程
        self.monitor_thread = threading.Thread(target=self.status_monitor)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        # 定时发布速度命令
        self.cmd_timer = rospy.Timer(rospy.Duration(0.1), self.publish_velocity_command)
    
    def vel_cmd_callback(self, msg):
        """
        处理多旋翼格式的速度命令，但适配到固定翼
        期望格式: [x, y, z, omega] -> 适配为 [forward_speed, 0, climb_rate, yaw_rate]
        """
        if len(msg.data) >= 4:
            # 将x,y速度合成为前进速度
            x_vel = msg.data[0]
            y_vel = msg.data[1] 
            z_vel = msg.data[2]
            yaw_rate = msg.data[3]
            
            # 计算合成的前进速度
            forward_speed = (x_vel**2 + y_vel**2)**0.5
            
            # 如果有侧向速度，转换为偏航调整
            if abs(y_vel) > 0.1:
                yaw_rate += y_vel * 0.2  # 简单的侧向到偏航转换
            
            # 应用固定翼限制
            forward_speed = max(self.MIN_FORWARD_SPEED, min(self.MAX_FORWARD_SPEED, forward_speed))
            climb_rate = max(-self.MAX_CLIMB_RATE, min(self.MAX_CLIMB_RATE, z_vel))
            yaw_rate = max(-self.MAX_YAW_RATE, min(self.MAX_YAW_RATE, yaw_rate))
            
            # 更新当前速度
            self.current_twist.linear.x = forward_speed
            self.current_twist.linear.y = 0.0  # 固定翼不侧飞
            self.current_twist.linear.z = climb_rate
            self.current_twist.angular.x = 0.0
            self.current_twist.angular.y = 0.0
            self.current_twist.angular.z = yaw_rate
            
            rospy.loginfo(f"多旋翼命令适配: 原始[{x_vel:.2f},{y_vel:.2f},{z_vel:.2f},{msg.data[3]:.2f}] -> 固定翼[前进={forward_speed:.2f}, 爬升={climb_rate:.2f}, 偏航={yaw_rate:.2f}]")
    
    def fixedwing_vel_cmd_callback(self, msg):
        """
        处理固定翼专用速度命令
        期望格式: [forward_speed, climb_rate, yaw_rate]
        """
        if len(msg.data) >= 3:
            forward_speed = max(self.MIN_FORWARD_SPEED, min(self.MAX_FORWARD_SPEED, msg.data[0]))
            climb_rate = max(-self.MAX_CLIMB_RATE, min(self.MAX_CLIMB_RATE, msg.data[1]))
            yaw_rate = max(-self.MAX_YAW_RATE, min(self.MAX_YAW_RATE, msg.data[2]))
            
            self.current_twist.linear.x = forward_speed
            self.current_twist.linear.y = 0.0
            self.current_twist.linear.z = climb_rate
            self.current_twist.angular.x = 0.0
            self.current_twist.angular.y = 0.0
            self.current_twist.angular.z = yaw_rate
            
            rospy.loginfo(f"固定翼专用命令: 前进={forward_speed:.2f}m/s, 爬升={climb_rate:.2f}m/s, 偏航={yaw_rate:.2f}rad/s")
    
    def vel_cmd_twist_callback(self, msg):
        """
        处理Twist格式的速度命令，适配固定翼
        """
        # 计算前进速度
        forward_speed = max(self.MIN_FORWARD_SPEED, min(self.MAX_FORWARD_SPEED, 
                           (msg.linear.x**2 + msg.linear.y**2)**0.5))
        
        # 如果前进速度太小，使用最小速度
        if forward_speed < self.MIN_FORWARD_SPEED:
            forward_speed = self.CRUISE_SPEED
        
        climb_rate = max(-self.MAX_CLIMB_RATE, min(self.MAX_CLIMB_RATE, msg.linear.z))
        yaw_rate = max(-self.MAX_YAW_RATE, min(self.MAX_YAW_RATE, msg.angular.z))
        
        self.current_twist.linear.x = forward_speed
        self.current_twist.linear.y = 0.0
        self.current_twist.linear.z = climb_rate
        self.current_twist.angular.x = 0.0
        self.current_twist.angular.y = 0.0
        self.current_twist.angular.z = yaw_rate
        
        rospy.loginfo(f"Twist命令适配: 前进={forward_speed:.2f}m/s, 爬升={climb_rate:.2f}m/s, 偏航={yaw_rate:.2f}rad/s")
    
    def publish_velocity_command(self, event):
        """定时发布速度命令"""
        self.cmd_vel_pub.publish(self.current_twist)
    
    def send_command(self, command):
        """发送控制命令"""
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
        self.is_flying = False
    
    def takeoff(self):
        """起飞"""
        self.send_command("AUTO.TAKEOFF")
    
    def land(self):
        """降落"""
        self.send_command("AUTO.LAND")
        self.is_flying = False
    
    def set_offboard_mode(self):
        """设置为OFFBOARD模式"""
        self.send_command("OFFBOARD")
        self.current_mode = "OFFBOARD"
    
    def set_mission_mode(self):
        """设置为任务模式（固定翼常用）"""
        self.send_command("AUTO.MISSION")
        self.current_mode = "MISSION"
    
    def set_cruise_mode(self):
        """设置巡航模式（替代悬停）"""
        self.set_cruise_speed()
        rospy.loginfo("固定翼巡航模式：保持水平巡航飞行")
    
    def set_cruise_speed(self, speed=None):
        """设置巡航速度"""
        if speed is None:
            speed = self.CRUISE_SPEED
        
        speed = max(self.MIN_FORWARD_SPEED, min(self.MAX_FORWARD_SPEED, speed))
        self.current_twist.linear.x = speed
        self.current_twist.linear.z = 0.0  # 水平飞行
        self.current_twist.angular.z = 0.0  # 直线飞行
        
        rospy.loginfo(f"设置巡航速度: {speed}m/s")
    
    def return_home(self):
        """返回起飞点"""
        self.send_command("AUTO.RTL")
        self.current_mode = "RTL"
    
    def emergency_return(self):
        """紧急返航 - 固定翼版本"""
        self.set_cruise_speed()  # 设置安全巡航速度
        self.return_home()
        rospy.logwarn("紧急返航！设置巡航速度并返回起飞点")
    
    def status_monitor(self):
        """状态监控线程"""
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            status_msg = String()
            status_msg.data = f"Armed: {self.is_armed}, Mode: {self.current_mode}, Flying: {self.is_flying}, " \
                            f"Speed: [前进={self.current_twist.linear.x:.2f}, 爬升={self.current_twist.linear.z:.2f}, " \
                            f"偏航={self.current_twist.angular.z:.2f}]"
            self.status_pub.publish(status_msg)
            rate.sleep()
    
    def get_status_info(self):
        """获取状态信息"""
        return f"""
========== 固定翼无人机状态 ==========
机型: {self.aircraft_type}_{self.aircraft_id}
控制模式: {self.control_type}
是否解锁: {self.is_armed}
飞行模式: {self.current_mode}
飞行状态: {self.is_flying}
当前速度: 前进={self.current_twist.linear.x:.2f}m/s, 
         爬升={self.current_twist.linear.z:.2f}m/s, 
         偏航={self.current_twist.angular.z:.2f}rad/s
速度限制: 前进[{self.MIN_FORWARD_SPEED}-{self.MAX_FORWARD_SPEED}]m/s, 
         爬升[±{self.MAX_CLIMB_RATE}]m/s
===================================
"""

    def auto_takeoff_sequence(self):
        """
        固定翼自动起飞流程
        """
        rospy.loginfo("开始固定翼自动起飞流程...")
        
        # 步骤1: 设置起飞速度
        rospy.loginfo("步骤1: 设置起飞速度")
        takeoff_speed = 18.0  # 起飞前进速度
        climb_rate = 2.5      # 爬升率
        
        self.current_twist.linear.x = takeoff_speed
        self.current_twist.linear.y = 0.0
        self.current_twist.linear.z = climb_rate
        self.current_twist.angular.x = 0.0
        self.current_twist.angular.y = 0.0
        self.current_twist.angular.z = 0.0
        
        rospy.loginfo(f"起飞参数: 前进速度={takeoff_speed}m/s, 爬升率={climb_rate}m/s")
        rospy.sleep(2.0)
        
        # 步骤2: 切换到OFFBOARD模式
        rospy.loginfo("步骤2: 切换到OFFBOARD模式")
        self.set_offboard_mode()
        rospy.sleep(1.0)
        
        # 步骤3: 解锁无人机
        rospy.loginfo("步骤3: 解锁无人机")
        self.arm()
        self.is_flying = True
        rospy.sleep(2.0)
        
        rospy.loginfo("固定翼起飞流程完成！")
        rospy.loginfo(f"无人机正在以{takeoff_speed}m/s前进，{climb_rate}m/s爬升")
        
        # 步骤4: 10秒后切换到巡航模式
        rospy.loginfo("10秒后将切换到巡航模式...")
        rospy.sleep(10.0)
        
        rospy.loginfo("切换到巡航模式")
        self.set_cruise_speed()
    
    def auto_landing_sequence(self):
        """
        固定翼自动降落流程
        """
        rospy.loginfo("开始固定翼自动降落流程...")
        
        # 步骤1: 设置进近速度
        rospy.loginfo("步骤1: 设置进近速度")
        approach_speed = 15.0
        descent_rate = -1.5
        
        self.current_twist.linear.x = approach_speed
        self.current_twist.linear.z = descent_rate
        self.current_twist.angular.z = 0.0
        
        rospy.loginfo(f"进近参数: 前进速度={approach_speed}m/s, 下降率={descent_rate}m/s")
        rospy.sleep(8.0)
        
        # 步骤2: 切换到降落模式
        rospy.loginfo("步骤2: 切换到自动降落模式")
        self.land()
        rospy.sleep(2.0)
        
        rospy.loginfo("固定翼降落流程已启动")
        
        # 等待降落完成
        rospy.sleep(15.0)
        rospy.loginfo("降落完成，上锁无人机")
        self.disarm()

def main():
    """主函数"""
    if len(sys.argv) < 4:
        print("用法: python3 fixedwing_control_xtdrone.py <aircraft_type> <aircraft_id> <control_type> [auto_takeoff]")
        print("例如: python3 fixedwing_control_xtdrone.py standard_vtol 0 vel")
        print("     python3 fixedwing_control_xtdrone.py standard_vtol 0 vel auto_takeoff")
        print("机型类型: standard_vtol, plane, fw 等")
        print("控制类型: vel (速度控制) 或 accel (加速度控制)")
        print("auto_takeoff: 可选参数，启动时自动执行起飞流程")
        return
    
    aircraft_type = sys.argv[1]
    aircraft_id = int(sys.argv[2])
    control_type = sys.argv[3]
    auto_takeoff = len(sys.argv) > 4 and sys.argv[4].lower() == 'auto_takeoff'
    
    # 创建控制器
    controller = FixedwingController(aircraft_type, aircraft_id, control_type)
    
    print("=" * 60)
    print("固定翼无人机控制脚本已启动 (兼容xtdrone)")
    if auto_takeoff:
        print("*** 自动起飞模式已启用 ***")
    print("=" * 60)
    print(controller.get_status_info())
    print("控制命令 (兼容多旋翼格式):")
    print("  发布到 /vel_cmd (Float32MultiArray): [x, y, z, omega] -> 自动适配固定翼")
    print("  发布到 /vel_cmd_twist (Twist): 标准ROS Twist消息 -> 自动适配固定翼")
    print("  发布到 /fixedwing_vel_cmd (Float32MultiArray): [forward_speed, climb_rate, yaw_rate]")
    print()
    print("可用的控制话题:")
    print(f"  解锁:     rostopic pub /drone_cmd std_msgs/String 'ARM'")
    print(f"  上锁:     rostopic pub /drone_cmd std_msgs/String 'DISARM'")
    print(f"  起飞:     rostopic pub /drone_cmd std_msgs/String 'TAKEOFF'")
    print(f"  降落:     rostopic pub /drone_cmd std_msgs/String 'LAND'")
    print(f"  OFFBOARD: rostopic pub /drone_cmd std_msgs/String 'OFFBOARD'")
    print(f"  巡航:     rostopic pub /drone_cmd std_msgs/String 'CRUISE'")
    print(f"  任务模式: rostopic pub /drone_cmd std_msgs/String 'MISSION'")
    print(f"  返航:     rostopic pub /drone_cmd std_msgs/String 'RTL'")
    print(f"  紧急返航: rostopic pub /drone_cmd std_msgs/String 'EMERGENCY'")
    print(f"  自动起飞: rostopic pub /drone_cmd std_msgs/String 'AUTO_TAKEOFF'")
    print(f"  自动降落: rostopic pub /drone_cmd std_msgs/String 'AUTO_LAND'")
    print()
    print("固定翼速度命令示例:")
    print("  # 兼容多旋翼格式 - 会自动适配")
    print("  rostopic pub /vel_cmd std_msgs/Float32MultiArray '{data: [18.0, 0.0, 2.0, 0.1]}'")
    print("  # 固定翼专用格式 - 推荐使用")
    print("  rostopic pub /fixedwing_vel_cmd std_msgs/Float32MultiArray '{data: [20.0, 0.0, 0.0]}'")
    print()
    print("⚠️  固定翼注意事项:")
    print("   - 自动保持最小前进速度12m/s以维持升力")
    print("   - 多旋翼的x,y速度会被合成为前进速度")
    print("   - 侧向速度会转换为偏航调整")
    print("   - 悬停命令会被转换为巡航命令")
    print("   - 紧急停止会切换为安全巡航而不是停止")
    print()
    
    if auto_takeoff:
        print("!!! 5秒后将自动执行固定翼起飞流程 !!!")
        print("如需取消，请按 Ctrl+C")
        for i in range(5, 0, -1):
            print(f"倒计时: {i} 秒...")
            time.sleep(1)
    else:
        print("手动控制模式，需要手动发送控制命令")
    
    print("按 Ctrl+C 退出")
    print("=" * 60)
    
    # 订阅控制命令话题 - 兼容原有接口
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
        elif command == "HOVER":  # 固定翼不能悬停，转为巡航
            controller.set_cruise_mode()
        elif command == "CRUISE":
            controller.set_cruise_mode()
        elif command == "MISSION":
            controller.set_mission_mode()
        elif command == "RTL":
            controller.return_home()
        elif command == "EMERGENCY":
            controller.emergency_return()
        elif command == "AUTO_TAKEOFF":
            threading.Thread(target=controller.auto_takeoff_sequence, daemon=True).start()
        elif command == "AUTO_LAND":
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
            controller.emergency_return()
            return
    
    try:
        # 主循环
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            rate.sleep()
    except KeyboardInterrupt:
        print("\n接收到退出信号，正在停止...")
        controller.emergency_return()
        time.sleep(1)
        print("固定翼控制器已停止")

if __name__ == '__main__':
    main()
