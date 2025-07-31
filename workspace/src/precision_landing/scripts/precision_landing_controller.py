#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float64
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from tf.transformations import euler_from_quaternion

class PrecisionLandingController:
    """
    精准降落控制器 - 基于视觉检测的无人机自动降落
    """
    
    def __init__(self):
        rospy.init_node('precision_landing_controller', anonymous=True)
        
        # 无人机参数
        self.drone_id = 0  # iris_0
        self.drone_type = "iris"
        
        # 控制参数
        self.image_width = 640
        self.image_height = 480
        self.camera_fov_h = 60.0  # 水平视场角(度)
        self.camera_fov_v = 45.0  # 垂直视场角(度)
        
        # PID控制器参数
        self.pid_x = {'kp': 0.5, 'ki': 0.01, 'kd': 0.1, 'integral': 0, 'prev_error': 0}
        self.pid_y = {'kp': 0.5, 'ki': 0.01, 'kd': 0.1, 'integral': 0, 'prev_error': 0}
        self.pid_z = {'kp': 0.3, 'ki': 0.005, 'kd': 0.05, 'integral': 0, 'prev_error': 0}
        self.pid_yaw = {'kp': 0.8, 'ki': 0.02, 'kd': 0.15, 'integral': 0, 'prev_error': 0}
        
        # 降落参数
        self.landing_threshold = 50  # 像素误差阈值
        self.min_altitude = 0.5      # 最小降落高度
        self.descent_rate = 0.3      # 下降速率 m/s
        self.max_vel = 2.0           # 最大速度限制
        
        # 状态变量
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target_detected = False
        self.target_position = PointStamped()
        self.landing_enabled = False
        self.altitude = 0.0
        
        # 订阅话题
        self.state_sub = rospy.Subscriber(f'/iris_{self.drone_id}/mavros/state', State, self.state_callback)
        self.pose_sub = rospy.Subscriber(f'/iris_{self.drone_id}/mavros/local_position/pose', PoseStamped, self.pose_callback)
        
        # 订阅降落平台检测结果 (支持多种检测器)
        self.target_sub_basic = rospy.Subscriber('/landing_target', PointStamped, self.target_callback_basic)
        self.target_sub_camo = rospy.Subscriber('/landing_target_camo', PointStamped, self.target_callback_camo)
        self.target_sub_red = rospy.Subscriber('/landing_target_red', PointStamped, self.target_callback_red)
        
        # 控制命令订阅
        self.landing_cmd_sub = rospy.Subscriber('/precision_landing/enable', Bool, self.landing_enable_callback)
        
        # 发布控制命令
        self.cmd_vel_pub = rospy.Publisher(f'/xtdrone/{self.drone_type}_{self.drone_id}/cmd_vel_flu', Twist, queue_size=1)
        self.cmd_pub = rospy.Publisher(f'/xtdrone/{self.drone_type}_{self.drone_id}/cmd', String, queue_size=1)
        
        # 发布状态信息
        self.status_pub = rospy.Publisher('/precision_landing/status', String, queue_size=1)
        self.error_pub = rospy.Publisher('/precision_landing/error', PointStamped, queue_size=1)
        self.altitude_pub = rospy.Publisher('/precision_landing/altitude', Float64, queue_size=1)
        
        # 服务客户端
        self.arming_client = rospy.ServiceProxy(f'/iris_{self.drone_id}/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy(f'/iris_{self.drone_id}/mavros/set_mode', SetMode)
        self.takeoff_client = rospy.ServiceProxy(f'/iris_{self.drone_id}/mavros/cmd/takeoff', CommandTOL)
        self.land_client = rospy.ServiceProxy(f'/iris_{self.drone_id}/mavros/cmd/land', CommandTOL)
        
        # 控制循环
        self.control_rate = rospy.Rate(30)  # 30Hz控制频率
        self.last_target_time = rospy.Time.now()
        self.target_timeout = 2.0  # 目标丢失超时时间
        
        rospy.loginfo("Precision Landing Controller initialized")
        
    def state_callback(self, msg):
        """飞行器状态回调"""
        self.current_state = msg
        
    def pose_callback(self, msg):
        """位置回调"""
        self.current_pose = msg
        self.altitude = msg.pose.position.z
        
        # 发布高度信息
        altitude_msg = Float64()
        altitude_msg.data = self.altitude
        self.altitude_pub.publish(altitude_msg)
        
    def target_callback_basic(self, msg):
        """基础检测器目标回调"""
        self.update_target(msg, "basic")
        
    def target_callback_camo(self, msg):
        """迷彩图案检测器目标回调"""
        self.update_target(msg, "camouflage")
        
    def target_callback_red(self, msg):
        """红色图案检测器目标回调"""
        self.update_target(msg, "red")
        
    def update_target(self, msg, detector_type):
        """更新目标位置信息"""
        self.target_position = msg
        self.target_detected = True
        self.last_target_time = rospy.Time.now()
        
        rospy.loginfo(f"Target detected by {detector_type} detector at ({msg.point.x:.1f}, {msg.point.y:.1f}) "
                     f"confidence: {msg.point.z:.2f}")
        
    def landing_enable_callback(self, msg):
        """降落使能回调"""
        self.landing_enabled = msg.data
        if self.landing_enabled:
            rospy.loginfo("Precision landing ENABLED")
            self.publish_status("ENABLED")
        else:
            rospy.loginfo("Precision landing DISABLED")
            self.publish_status("DISABLED")
            
    def pixel_to_velocity(self, pixel_error_x, pixel_error_y):
        """
        将像素误差转换为速度命令
        使用视场角计算实际角度误差，再转换为速度
        """
        # 计算像素到角度的转换
        angle_error_x = (pixel_error_x / self.image_width) * math.radians(self.camera_fov_h)
        angle_error_y = (pixel_error_y / self.image_height) * math.radians(self.camera_fov_v)
        
        # 根据高度计算水平距离误差
        horizontal_error_x = self.altitude * math.tan(angle_error_x)
        horizontal_error_y = self.altitude * math.tan(angle_error_y)
        
        return horizontal_error_x, horizontal_error_y
        
    def pid_control(self, error, pid_params, dt):
        """PID控制器"""
        pid_params['integral'] += error * dt
        derivative = (error - pid_params['prev_error']) / dt
        
        # 积分限幅
        if abs(pid_params['integral']) > 10.0:
            pid_params['integral'] = 10.0 * np.sign(pid_params['integral'])
        
        output = (pid_params['kp'] * error + 
                 pid_params['ki'] * pid_params['integral'] + 
                 pid_params['kd'] * derivative)
        
        pid_params['prev_error'] = error
        return output
        
    def calculate_control_command(self, dt):
        """计算控制命令"""
        if not self.target_detected:
            return None
            
        # 检查目标是否超时
        if (rospy.Time.now() - self.last_target_time).to_sec() > self.target_timeout:
            self.target_detected = False
            rospy.logwarn("Target lost - timeout")
            return None
            
        # 计算像素误差 (图像中心为目标)
        pixel_error_x = self.target_position.point.x - self.image_width / 2
        pixel_error_y = self.target_position.point.y - self.image_height / 2
        
        # 转换为距离误差
        horizontal_error_x, horizontal_error_y = self.pixel_to_velocity(pixel_error_x, pixel_error_y)
        
        # PID控制计算速度命令
        vel_x = -self.pid_control(horizontal_error_y, self.pid_x, dt)  # forward/backward
        vel_y = -self.pid_control(horizontal_error_x, self.pid_y, dt)  # left/right
        
        # 高度控制 - 缓慢下降
        if abs(pixel_error_x) < self.landing_threshold and abs(pixel_error_y) < self.landing_threshold:
            # 目标对准，开始下降
            vel_z = -self.descent_rate
        else:
            # 目标未对准，保持高度
            vel_z = 0.0
            
        # 偏航控制 (保持当前偏航)
        vel_yaw = 0.0
        
        # 速度限幅
        vel_x = max(-self.max_vel, min(self.max_vel, vel_x))
        vel_y = max(-self.max_vel, min(self.max_vel, vel_y))
        vel_z = max(-self.max_vel, min(self.max_vel, vel_z))
        
        # 发布误差信息用于调试
        error_msg = PointStamped()
        error_msg.header.stamp = rospy.Time.now()
        error_msg.point.x = horizontal_error_x
        error_msg.point.y = horizontal_error_y
        error_msg.point.z = self.altitude
        self.error_pub.publish(error_msg)
        
        return vel_x, vel_y, vel_z, vel_yaw
        
    def publish_control_command(self, vel_x, vel_y, vel_z, vel_yaw):
        """发布控制命令"""
        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = vel_y
        twist.linear.z = vel_z
        twist.angular.z = vel_yaw
        
        self.cmd_vel_pub.publish(twist)
        
    def publish_status(self, status):
        """发布状态信息"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
        
    def emergency_land(self):
        """紧急降落"""
        rospy.logwarn("Emergency landing activated")
        cmd_msg = String()
        cmd_msg.data = "AUTO.LAND"
        self.cmd_pub.publish(cmd_msg)
        self.publish_status("EMERGENCY_LANDING")
        
    def run(self):
        """主控制循环"""
        rospy.loginfo("Precision landing controller started")
        
        prev_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - prev_time).to_sec()
            prev_time = current_time
            
            if dt <= 0:
                dt = 0.033  # 默认30Hz
                
            # 检查是否启用精准降落
            if not self.landing_enabled:
                self.control_rate.sleep()
                continue
                
            # 检查飞行器状态
            if not self.current_state.connected:
                rospy.logwarn("Vehicle not connected")
                self.control_rate.sleep()
                continue
                
            # 安全检查 - 最小高度
            if self.altitude < self.min_altitude:
                rospy.loginfo("Reached minimum altitude, initiating final landing")
                self.emergency_land()
                break
                
            # 计算控制命令
            if self.target_detected:
                control_result = self.calculate_control_command(dt)
                
                if control_result is not None:
                    vel_x, vel_y, vel_z, vel_yaw = control_result
                    self.publish_control_command(vel_x, vel_y, vel_z, vel_yaw)
                    
                    # 发布状态
                    pixel_error_x = self.target_position.point.x - self.image_width / 2
                    pixel_error_y = self.target_position.point.y - self.image_height / 2
                    error_magnitude = math.sqrt(pixel_error_x**2 + pixel_error_y**2)
                    
                    if error_magnitude < self.landing_threshold:
                        self.publish_status(f"DESCENDING - Error: {error_magnitude:.1f}px")
                    else:
                        self.publish_status(f"ALIGNING - Error: {error_magnitude:.1f}px")
                        
                    rospy.loginfo(f"Control: vx={vel_x:.2f}, vy={vel_y:.2f}, vz={vel_z:.2f}, "
                                f"error={error_magnitude:.1f}px, alt={self.altitude:.2f}m")
                else:
                    # 目标丢失，悬停
                    self.publish_control_command(0, 0, 0, 0)
                    self.publish_status("TARGET_LOST - HOVERING")
            else:
                # 没有检测到目标，悬停
                self.publish_control_command(0, 0, 0, 0)
                self.publish_status("WAITING_FOR_TARGET")
                
            self.control_rate.sleep()
            
        rospy.loginfo("Precision landing controller stopped")

if __name__ == '__main__':
    try:
        controller = PrecisionLandingController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Precision landing controller interrupted")
