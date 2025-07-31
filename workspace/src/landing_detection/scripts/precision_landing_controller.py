#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from geometry_msgs.msg import Point, PointStamped, PoseStamped, TwistStamped
from sensor_msgs.msg import Image
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
import cv2
from cv_bridge import CvBridge

class PrecisionLandingController:
    def __init__(self):
        rospy.init_node('precision_landing_controller', anonymous=True)
        
        # 参数设置
        self.image_width = 640   # 图像宽度（需要根据实际摄像头调整）
        self.image_height = 480  # 图像高度
        self.image_center_x = self.image_width // 2
        self.image_center_y = self.image_height // 2
        
        # PID控制器参数
        self.kp_x = 0.5    # X方向比例增益
        self.ki_x = 0.01   # X方向积分增益
        self.kd_x = 0.1    # X方向微分增益
        
        self.kp_y = 0.5    # Y方向比例增益
        self.ki_y = 0.01   # Y方向积分增益
        self.kd_y = 0.1    # Y方向微分增益
        
        # 误差积分和上次误差
        self.error_sum_x = 0
        self.error_sum_y = 0
        self.last_error_x = 0
        self.last_error_y = 0
        
        # 状态变量
        self.target_detected = False
        self.target_position = None
        self.current_pose = None
        self.current_state = None
        self.landing_threshold = 20  # 像素误差阈值
        self.stable_count = 0
        self.stable_threshold = 10   # 需要连续稳定的次数
        self.landing_altitude = 2.0  # 开始精准降落的高度
        
        # ROS订阅者
        self.target_sub = rospy.Subscriber('/landing_target', PointStamped, self.target_callback)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_callback)
        self.image_sub = rospy.Subscriber('/iris_0/camera/image_raw', Image, self.image_callback)
        
        # ROS发布者
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=1)
        
        # ROS服务客户端
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.land_client = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        
        self.bridge = CvBridge()
        
        rospy.loginfo("Precision landing controller initialized")
    
    def target_callback(self, msg):
        """
        目标位置回调函数
        """
        self.target_detected = True
        self.target_position = (msg.point.x, msg.point.y)
        
        # 计算目标相对于图像中心的偏移
        error_x = self.target_position[0] - self.image_center_x
        error_y = self.target_position[1] - self.image_center_y
        
        rospy.logdebug(f"Target detected at ({msg.point.x}, {msg.point.y}), "
                      f"Error: ({error_x}, {error_y})")
    
    def pose_callback(self, msg):
        """
        位置回调函数
        """
        self.current_pose = msg
    
    def state_callback(self, msg):
        """
        状态回调函数
        """
        self.current_state = msg
    
    def image_callback(self, msg):
        """
        图像回调函数 - 用于获取图像尺寸
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_height, self.image_width = cv_image.shape[:2]
            self.image_center_x = self.image_width // 2
            self.image_center_y = self.image_height // 2
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def calculate_control_command(self):
        """
        计算控制命令
        """
        if not self.target_detected or not self.target_position:
            return None
        
        # 计算像素误差
        error_x = self.target_position[0] - self.image_center_x
        error_y = self.target_position[1] - self.image_center_y
        
        # PID控制器
        # 比例项
        p_x = self.kp_x * error_x
        p_y = self.kp_y * error_y
        
        # 积分项
        self.error_sum_x += error_x
        self.error_sum_y += error_y
        i_x = self.ki_x * self.error_sum_x
        i_y = self.ki_y * self.error_sum_y
        
        # 微分项
        d_x = self.kd_x * (error_x - self.last_error_x)
        d_y = self.kd_y * (error_y - self.last_error_y)
        
        # 更新上次误差
        self.last_error_x = error_x
        self.last_error_y = error_y
        
        # 计算控制输出
        control_x = (p_x + i_x + d_x) / 1000.0  # 转换为合适的速度值
        control_y = (p_y + i_y + d_y) / 1000.0
        
        # 限制最大速度
        max_velocity = 1.0  # m/s
        control_x = max(-max_velocity, min(max_velocity, control_x))
        control_y = max(-max_velocity, min(max_velocity, control_y))
        
        return control_x, control_y, error_x, error_y
    
    def is_target_centered(self):
        """
        检查目标是否已经居中
        """
        if not self.target_detected or not self.target_position:
            return False
        
        error_x = abs(self.target_position[0] - self.image_center_x)
        error_y = abs(self.target_position[1] - self.image_center_y)
        
        return error_x < self.landing_threshold and error_y < self.landing_threshold
    
    def publish_velocity_command(self, vx=0, vy=0, vz=0):
        """
        发布速度命令
        """
        cmd = TwistStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.twist.linear.x = vx
        cmd.twist.linear.y = vy
        cmd.twist.linear.z = vz
        
        self.velocity_pub.publish(cmd)
    
    def start_precision_landing(self):
        """
        开始精准降落过程
        """
        rospy.loginfo("Starting precision landing...")
        
        rate = rospy.Rate(20)  # 20Hz控制频率
        
        while not rospy.is_shutdown():
            if not self.target_detected:
                # 如果没有检测到目标，悬停等待
                rospy.logwarn("No target detected, hovering...")
                self.publish_velocity_command(0, 0, 0)
                self.stable_count = 0
                rate.sleep()
                continue
            
            # 计算控制命令
            control_result = self.calculate_control_command()
            if control_result is None:
                self.publish_velocity_command(0, 0, 0)
                rate.sleep()
                continue
            
            control_x, control_y, error_x, error_y = control_result
            
            # 检查是否居中
            if self.is_target_centered():
                self.stable_count += 1
                rospy.loginfo(f"Target centered! Stable count: {self.stable_count}/{self.stable_threshold}")
                
                if self.stable_count >= self.stable_threshold:
                    # 目标已稳定居中，开始最终降落
                    rospy.loginfo("Target stable, starting final landing...")
                    self.final_landing()
                    break
                else:
                    # 保持位置
                    self.publish_velocity_command(0, 0, 0)
            else:
                # 重置稳定计数器
                self.stable_count = 0
                
                # 发布调整命令
                rospy.loginfo(f"Adjusting position: error=({error_x:.1f}, {error_y:.1f}), "
                             f"control=({control_x:.3f}, {control_y:.3f})")
                
                # 注意：这里的坐标系可能需要根据实际情况调整
                # X方向：图像右为正，对应无人机前进方向
                # Y方向：图像下为正，对应无人机右移方向
                self.publish_velocity_command(control_y, control_x, 0)
            
            # 重置目标检测标志（等待下一次检测）
            self.target_detected = False
            
            rate.sleep()
    
    def final_landing(self):
        """
        最终降落
        """
        rospy.loginfo("Executing final landing...")
        
        # 切换到降落模式
        try:
            response = self.land_client()
            if response.success:
                rospy.loginfo("Landing command sent successfully")
            else:
                rospy.logerr("Failed to send landing command")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        
        # 或者使用缓慢下降
        rate = rospy.Rate(10)
        landing_duration = 30  # 30秒降落时间
        landing_speed = -0.3   # 下降速度 m/s
        
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > landing_duration:
                break
            
            # 继续调整水平位置
            if self.target_detected and self.target_position:
                control_result = self.calculate_control_command()
                if control_result:
                    control_x, control_y, _, _ = control_result
                    self.publish_velocity_command(control_y, control_x, landing_speed)
                else:
                    self.publish_velocity_command(0, 0, landing_speed)
            else:
                self.publish_velocity_command(0, 0, landing_speed)
            
            self.target_detected = False
            rate.sleep()
        
        rospy.loginfo("Landing sequence completed")
    
    def run(self):
        """
        运行精准降落控制器
        """
        rospy.loginfo("Precision landing controller running...")
        
        # 等待系统就绪
        rospy.loginfo("Waiting for system to be ready...")
        while not rospy.is_shutdown():
            if (self.current_state and self.current_state.connected and 
                self.current_pose):
                break
            rospy.sleep(0.1)
        
        rospy.loginfo("System ready, starting precision landing")
        
        # 开始精准降落
        self.start_precision_landing()

if __name__ == '__main__':
    try:
        controller = PrecisionLandingController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Precision landing controller stopped")
