#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, PointStamped
from std_msgs.msg import String, Bool, Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VisualServoController:
    """
    视觉伺服控制器 - 基于图像特征的精准控制
    """
    
    def __init__(self):
        rospy.init_node('visual_servo_controller', anonymous=True)
        
        self.bridge = CvBridge()
        
        # 控制参数
        self.image_width = 640
        self.image_height = 480
        self.target_radius = 50  # 期望的目标半径(像素)
        
        # 视觉伺服参数
        self.lambda_param = 0.5  # 伺服增益
        self.depth_gain = 0.3    # 深度控制增益
        
        # 特征点参数
        self.feature_points = []  # 当前特征点
        self.desired_features = []  # 期望特征点
        
        # 状态变量
        self.current_image = None
        self.target_detected = False
        self.target_position = PointStamped()
        self.servo_enabled = False
        
        # 订阅话题
        self.image_sub = rospy.Subscriber('/iris_0/camera/image_raw', Image, self.image_callback)
        self.target_sub = rospy.Subscriber('/landing_target', PointStamped, self.target_callback)
        self.enable_sub = rospy.Subscriber('/visual_servo/enable', Bool, self.enable_callback)
        
        # 发布控制命令
        self.cmd_vel_pub = rospy.Publisher('/xtdrone/iris_0/cmd_vel_flu', Twist, queue_size=1)
        
        # 发布调试信息
        self.debug_image_pub = rospy.Publisher('/visual_servo/debug_image', Image, queue_size=1)
        self.features_pub = rospy.Publisher('/visual_servo/features', Float64MultiArray, queue_size=1)
        self.error_pub = rospy.Publisher('/visual_servo/error', Float64MultiArray, queue_size=1)
        
        rospy.loginfo("Visual Servo Controller initialized")
        
    def image_callback(self, msg):
        """图像回调"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
            
    def target_callback(self, msg):
        """目标检测回调"""
        self.target_position = msg
        self.target_detected = True
        
    def enable_callback(self, msg):
        """使能回调"""
        self.servo_enabled = msg.data
        if self.servo_enabled:
            rospy.loginfo("Visual servo ENABLED")
            # 初始化期望特征
            self.initialize_desired_features()
        else:
            rospy.loginfo("Visual servo DISABLED")
            
    def initialize_desired_features(self):
        """初始化期望特征点"""
        # 期望特征：图像中心的圆形，特定半径
        center_x = self.image_width // 2
        center_y = self.image_height // 2
        
        # 圆形特征点：中心 + 圆周上的点
        self.desired_features = [
            [center_x, center_y],  # 中心点
            [center_x + self.target_radius, center_y],  # 右
            [center_x, center_y + self.target_radius],  # 下
            [center_x - self.target_radius, center_y],  # 左
            [center_x, center_y - self.target_radius],  # 上
        ]
        
        rospy.loginfo(f"Desired features initialized: {self.desired_features}")
        
    def extract_features(self, image, target_center, target_radius):
        """从图像中提取特征点"""
        if target_center is None or target_radius is None:
            return []
            
        cx, cy = int(target_center[0]), int(target_center[1])
        r = int(target_radius)
        
        # 提取圆形特征点
        features = [
            [cx, cy],  # 中心点
            [cx + r, cy],  # 右
            [cx, cy + r],  # 下
            [cx - r, cy],  # 左
            [cx, cy - r],  # 上
        ]
        
        return features
        
    def calculate_interaction_matrix(self, features):
        """计算交互矩阵(图像雅可比)"""
        # 对于2D点特征，交互矩阵为2x6
        # 这里简化为2D情况，只考虑x,y,z平移和偏航
        L = np.zeros((len(features) * 2, 4))  # 4DOF: vx, vy, vz, vyaw
        
        for i, feature in enumerate(features):
            x, y = feature
            
            # 归一化图像坐标
            u = (x - self.image_width / 2) / self.image_width
            v = (y - self.image_height / 2) / self.image_height
            
            # 简化的交互矩阵
            L[2*i, 0] = -1  # vx对u的影响
            L[2*i, 1] = 0   # vy对u的影响
            L[2*i, 2] = u   # vz对u的影响
            L[2*i, 3] = v   # vyaw对u的影响
            
            L[2*i+1, 0] = 0   # vx对v的影响
            L[2*i+1, 1] = -1  # vy对v的影响
            L[2*i+1, 2] = v   # vz对v的影响
            L[2*i+1, 3] = -u  # vyaw对v的影响
            
        return L
        
    def calculate_feature_error(self, current_features, desired_features):
        """计算特征误差"""
        if len(current_features) != len(desired_features):
            return None
            
        error = []
        for i in range(len(current_features)):
            error.extend([
                current_features[i][0] - desired_features[i][0],
                current_features[i][1] - desired_features[i][1]
            ])
            
        return np.array(error)
        
    def visual_servo_control(self):
        """视觉伺服控制计算"""
        if not self.target_detected or self.current_image is None:
            return None
            
        # 估计目标半径（基于置信度或其他方法）
        # 这里简化处理，假设半径与高度成反比
        estimated_radius = max(20, min(100, self.target_radius))
        
        # 提取当前特征
        target_center = [self.target_position.point.x, self.target_position.point.y]
        current_features = self.extract_features(self.current_image, target_center, estimated_radius)
        
        if len(current_features) == 0:
            return None
            
        # 计算特征误差
        feature_error = self.calculate_feature_error(current_features, self.desired_features)
        
        if feature_error is None:
            return None
            
        # 计算交互矩阵
        L = self.calculate_interaction_matrix(current_features)
        
        # 计算伪逆
        try:
            L_pinv = np.linalg.pinv(L)
        except np.linalg.LinAlgError:
            rospy.logwarn("Singular interaction matrix")
            return None
            
        # 计算速度命令
        velocity = -self.lambda_param * np.dot(L_pinv, feature_error)
        
        # 提取各轴速度
        vel_x = velocity[0] if len(velocity) > 0 else 0
        vel_y = velocity[1] if len(velocity) > 1 else 0
        vel_z = velocity[2] if len(velocity) > 2 else 0
        vel_yaw = velocity[3] if len(velocity) > 3 else 0
        
        # 速度限制
        max_vel = 1.0
        vel_x = max(-max_vel, min(max_vel, vel_x))
        vel_y = max(-max_vel, min(max_vel, vel_y))
        vel_z = max(-max_vel, min(max_vel, vel_z))
        vel_yaw = max(-1.0, min(1.0, vel_yaw))
        
        # 发布调试信息
        self.publish_debug_info(current_features, feature_error)
        
        return vel_x, vel_y, vel_z, vel_yaw
        
    def publish_debug_info(self, current_features, feature_error):
        """发布调试信息"""
        # 发布特征点
        features_msg = Float64MultiArray()
        features_flat = []
        for feature in current_features:
            features_flat.extend(feature)
        features_msg.data = features_flat
        self.features_pub.publish(features_msg)
        
        # 发布误差
        error_msg = Float64MultiArray()
        error_msg.data = feature_error.tolist()
        self.error_pub.publish(error_msg)
        
        # 绘制调试图像
        if self.current_image is not None:
            debug_image = self.current_image.copy()
            
            # 绘制当前特征点(红色)
            for feature in current_features:
                cv2.circle(debug_image, (int(feature[0]), int(feature[1])), 5, (0, 0, 255), -1)
                
            # 绘制期望特征点(绿色)
            for feature in self.desired_features:
                cv2.circle(debug_image, (int(feature[0]), int(feature[1])), 5, (0, 255, 0), 2)
                
            # 绘制连线
            if len(current_features) >= 1 and len(self.desired_features) >= 1:
                cv2.line(debug_image, 
                        (int(current_features[0][0]), int(current_features[0][1])),
                        (int(self.desired_features[0][0]), int(self.desired_features[0][1])),
                        (255, 0, 0), 2)
                        
            # 发布调试图像
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                self.debug_image_pub.publish(debug_msg)
            except Exception as e:
                rospy.logerr(f"Error publishing debug image: {e}")
                
    def run(self):
        """主循环"""
        rospy.loginfo("Visual Servo Controller started")
        
        rate = rospy.Rate(30)  # 30Hz
        
        while not rospy.is_shutdown():
            if self.servo_enabled:
                # 计算视觉伺服控制
                control_result = self.visual_servo_control()
                
                if control_result is not None:
                    vel_x, vel_y, vel_z, vel_yaw = control_result
                    
                    # 发布控制命令
                    twist = Twist()
                    twist.linear.x = vel_x
                    twist.linear.y = vel_y
                    twist.linear.z = vel_z
                    twist.angular.z = vel_yaw
                    
                    self.cmd_vel_pub.publish(twist)
                    
                    rospy.loginfo(f"Visual servo: vx={vel_x:.2f}, vy={vel_y:.2f}, vz={vel_z:.2f}, vyaw={vel_yaw:.2f}")
                else:
                    # 没有有效控制，发送零速度
                    twist = Twist()
                    self.cmd_vel_pub.publish(twist)
            else:
                # 禁用时发送零速度
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                
            rate.sleep()
            
        rospy.loginfo("Visual Servo Controller stopped")

if __name__ == '__main__':
    try:
        controller = VisualServoController()
        controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Visual Servo Controller interrupted")
