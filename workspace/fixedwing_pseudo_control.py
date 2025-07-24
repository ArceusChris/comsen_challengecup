#!/usr/bin/env python3
"""
固定翼伪控制脚本
通过连续瞬移来模拟固定翼的飞行控制
"""

import rospy
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import math
import time
import threading

class FixedwingPseudoController:
    def __init__(self, vehicle_type="standard_vtol", vehicle_id=0):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        
        # 当前状态
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 100.0  # 默认高度100m
        self.current_yaw = 0.0
        self.current_speed = 18.0
        
        # 目标速度
        self.target_forward = 18.0
        self.target_climb = 0.0
        self.target_yaw_rate = 0.0
        
        # 控制参数
        self.update_rate = 10.0  # 10Hz更新
        self.min_speed = 12.0
        self.max_speed = 30.0
        
        rospy.init_node(f'fixedwing_pseudo_controller_{vehicle_id}', anonymous=True)
        
        # 订阅速度命令
        self.vel_cmd_sub = rospy.Subscriber('/vel_cmd', Float32MultiArray, self.vel_cmd_callback)
        self.fixedwing_vel_sub = rospy.Subscriber('/fixedwing_vel_cmd', Float32MultiArray, self.fixedwing_vel_callback)
        
        # Gazebo瞬移服务
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # 等待服务可用
        rospy.wait_for_service('/gazebo/set_model_state', timeout=5.0)
        
        print(f"固定翼伪控制器已初始化: {vehicle_type}_{vehicle_id}")
        print("通过连续瞬移模拟固定翼飞行")
        
        # 启动控制循环
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
    def vel_cmd_callback(self, msg):
        """处理标准速度命令 [x, y, z, omega]"""
        if len(msg.data) >= 4:
            x_vel = msg.data[0]
            y_vel = msg.data[1]
            z_vel = msg.data[2]
            yaw_rate = msg.data[3]
            
            # 转换为固定翼参数
            forward_speed = max(abs(x_vel), abs(y_vel), self.min_speed)
            if forward_speed > self.max_speed:
                forward_speed = self.max_speed
                
            # 如果有侧向速度，转换为偏航
            if abs(y_vel) > 0.1:
                yaw_rate += y_vel * 0.3
                
            self.target_forward = forward_speed if x_vel >= 0 else -forward_speed
            self.target_climb = max(-5.0, min(5.0, z_vel))
            self.target_yaw_rate = max(-1.0, min(1.0, yaw_rate))
            
            print(f"伪控制命令: 前进={self.target_forward:.1f}m/s, 爬升={self.target_climb:.1f}m/s, 偏航率={self.target_yaw_rate:.2f}rad/s")
    
    def fixedwing_vel_callback(self, msg):
        """处理固定翼专用速度命令 [forward, climb, yaw_rate]"""
        if len(msg.data) >= 3:
            self.target_forward = max(self.min_speed, min(self.max_speed, msg.data[0]))
            self.target_climb = max(-5.0, min(5.0, msg.data[1]))
            self.target_yaw_rate = max(-1.0, min(1.0, msg.data[2]))
            
            print(f"固定翼伪控制: 前进={self.target_forward:.1f}m/s, 爬升={self.target_climb:.1f}m/s, 偏航率={self.target_yaw_rate:.2f}rad/s")
    
    def control_loop(self):
        """主控制循环"""
        rate = rospy.Rate(self.update_rate)
        dt = 1.0 / self.update_rate
        
        while not rospy.is_shutdown():
            # 更新位置
            # 前进方向基于当前偏航角
            dx = self.target_forward * math.cos(self.current_yaw) * dt
            dy = self.target_forward * math.sin(self.current_yaw) * dt
            dz = self.target_climb * dt
            dyaw = self.target_yaw_rate * dt
            
            self.current_x += dx
            self.current_y += dy
            self.current_z += dz
            self.current_yaw += dyaw
            
            # 限制高度
            self.current_z = max(5.0, min(200.0, self.current_z))
            
            # 归一化偏航角
            while self.current_yaw > math.pi:
                self.current_yaw -= 2 * math.pi
            while self.current_yaw < -math.pi:
                self.current_yaw += 2 * math.pi
            
            # 执行瞬移
            self.teleport_to_position()
            
            rate.sleep()
    
    def teleport_to_position(self):
        """瞬移到当前计算位置"""
        try:
            model_state = ModelState()
            model_state.model_name = f"{self.vehicle_type}_{self.vehicle_id}"
            
            # 设置位置
            model_state.pose.position.x = self.current_x
            model_state.pose.position.y = self.current_y
            model_state.pose.position.z = self.current_z
            
            # 设置姿态
            quat_z = math.sin(self.current_yaw / 2.0)
            quat_w = math.cos(self.current_yaw / 2.0)
            model_state.pose.orientation.x = 0.0
            model_state.pose.orientation.y = 0.0
            model_state.pose.orientation.z = quat_z
            model_state.pose.orientation.w = quat_w
            
            # 设置速度
            model_state.twist.linear.x = self.target_forward * math.cos(self.current_yaw)
            model_state.twist.linear.y = self.target_forward * math.sin(self.current_yaw)
            model_state.twist.linear.z = self.target_climb
            model_state.twist.angular.x = 0.0
            model_state.twist.angular.y = 0.0
            model_state.twist.angular.z = self.target_yaw_rate
            
            model_state.reference_frame = 'world'
            
            # 执行瞬移
            self.set_model_state(model_state)
            
        except Exception as e:
            print(f"瞬移失败: {e}")
    
    def get_status(self):
        """获取当前状态"""
        return f"""
========== 固定翼伪控制状态 ==========
位置: ({self.current_x:.1f}, {self.current_y:.1f}, {self.current_z:.1f})
偏航角: {math.degrees(self.current_yaw):.1f}°
目标速度: 前进={self.target_forward:.1f}m/s, 爬升={self.target_climb:.1f}m/s
偏航率: {self.target_yaw_rate:.2f}rad/s
===================================
"""

def main():
    controller = FixedwingPseudoController("standard_vtol", 0)
    
    print("=" * 50)
    print("固定翼伪控制器已启动")
    print("=" * 50)
    print("这个控制器通过连续瞬移来模拟固定翼飞行")
    print("支持标准速度命令:")
    print("  rostopic pub /vel_cmd std_msgs/Float32MultiArray '{data: [15.0, 0.0, 2.0, 0.1]}'")
    print("支持固定翼专用命令:")
    print("  rostopic pub /fixedwing_vel_cmd std_msgs/Float32MultiArray '{data: [20.0, -1.0, 0.2]}'")
    print()
    print(controller.get_status())
    print("按 Ctrl+C 退出")
    print("=" * 50)
    
    try:
        while not rospy.is_shutdown():
            rospy.sleep(1)
            # 每秒打印一次状态
            # print(controller.get_status())
    except KeyboardInterrupt:
        print("\n固定翼伪控制器已停止")

if __name__ == '__main__':
    main()
