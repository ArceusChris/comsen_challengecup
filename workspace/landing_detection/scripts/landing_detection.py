#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import math

class LandingPlatformDetector:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('landing_platform_detector', anonymous=True)
        
        # 创建CvBridge对象
        self.bridge = CvBridge()
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber('/iris_0/camera/image_raw', Image, self.image_callback)
        
        # 发布检测结果
        self.center_pub = rospy.Publisher('/landing_platform/center', Point, queue_size=1)
        
        # 发布带有检测标记的图像（用于调试）
        self.debug_image_pub = rospy.Publisher('/landing_platform/debug_image', Image, queue_size=1)
        
        # 参数设置
        self.min_circle_radius = 30  # 最小圆圈半径
        self.max_circle_radius = 200  # 最大圆圈半径
        self.white_threshold = 200   # 白色像素阈值
        self.cross_length_ratio = 0.3  # 十字相对于圆圈的长度比例
        
        rospy.loginfo("Landing platform detector initialized")
    
    def preprocess_image(self, image):
        """
        图像预处理：转换为灰度图并进行滤波
        """
        # 转换为灰度图
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 高斯滤波去噪
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 二值化处理，提取白色区域
        _, binary = cv2.threshold(blurred, self.white_threshold, 255, cv2.THRESH_BINARY)
        
        return gray, binary
    
    def detect_circles(self, binary_image):
        """
        使用霍夫圆变换检测圆圈
        """
        circles = cv2.HoughCircles(
            binary_image,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=100,
            param1=50,
            param2=30,
            minRadius=self.min_circle_radius,
            maxRadius=self.max_circle_radius
        )
        
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            return circles
        return None
    
    def validate_circle_with_cross(self, image, center_x, center_y, radius):
        """
        验证圆圈中心是否有十字标记
        """
        # 在圆圈中心区域搜索十字
        search_radius = int(radius * 0.4)  # 在圆圈内部40%的区域搜索
        
        # 提取圆圈中心区域
        y1 = max(0, center_y - search_radius)
        y2 = min(image.shape[0], center_y + search_radius)
        x1 = max(0, center_x - search_radius)
        x2 = min(image.shape[1], center_x + search_radius)
        
        roi = image[y1:y2, x1:x2]
        
        if roi.size == 0:
            return False, 0
        
        # 转换为灰度图并二值化
        if len(roi.shape) == 3:
            roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        else:
            roi_gray = roi
        
        _, roi_binary = cv2.threshold(roi_gray, self.white_threshold, 255, cv2.THRESH_BINARY)
        
        # 检测十字的水平和垂直线
        horizontal_score = self.detect_horizontal_line(roi_binary)
        vertical_score = self.detect_vertical_line(roi_binary)
        
        # 计算十字检测置信度
        cross_confidence = min(horizontal_score, vertical_score)
        
        # 如果水平和垂直线都检测到，认为存在十字
        return cross_confidence > 0.3, cross_confidence
    
    def detect_horizontal_line(self, binary_image):
        """
        检测水平线
        """
        h, w = binary_image.shape
        center_y = h // 2
        
        # 在中心附近几行搜索水平线
        search_range = 3
        max_score = 0
        
        for dy in range(-search_range, search_range + 1):
            y = center_y + dy
            if 0 <= y < h:
                row = binary_image[y, :]
                # 计算水平线的连续白色像素比例
                white_pixels = np.sum(row == 255)
                score = white_pixels / w
                max_score = max(max_score, score)
        
        return max_score
    
    def detect_vertical_line(self, binary_image):
        """
        检测垂直线
        """
        h, w = binary_image.shape
        center_x = w // 2
        
        # 在中心附近几列搜索垂直线
        search_range = 3
        max_score = 0
        
        for dx in range(-search_range, search_range + 1):
            x = center_x + dx
            if 0 <= x < w:
                col = binary_image[:, x]
                # 计算垂直线的连续白色像素比例
                white_pixels = np.sum(col == 255)
                score = white_pixels / h
                max_score = max(max_score, score)
        
        return max_score
    
    def find_best_landing_platform(self, image, circles):
        """
        从检测到的圆圈中找到最佳的降落平台
        """
        best_circle = None
        best_confidence = 0
        
        for (x, y, r) in circles:
            # 验证圆圈是否包含十字
            has_cross, confidence = self.validate_circle_with_cross(image, x, y, r)
            
            if has_cross and confidence > best_confidence:
                best_confidence = confidence
                best_circle = (x, y, r, confidence)
        
        return best_circle
    
    def draw_debug_info(self, image, circles, best_circle):
        """
        在图像上绘制调试信息
        """
        debug_image = image.copy()
        
        # 绘制所有检测到的圆圈（绿色）
        if circles is not None:
            for (x, y, r) in circles:
                cv2.circle(debug_image, (x, y), r, (0, 255, 0), 2)
                cv2.circle(debug_image, (x, y), 2, (0, 255, 0), 3)
        
        # 绘制最佳降落平台（红色）
        if best_circle is not None:
            x, y, r, confidence = best_circle
            cv2.circle(debug_image, (x, y), r, (0, 0, 255), 3)
            cv2.circle(debug_image, (x, y), 5, (0, 0, 255), -1)
            
            # 绘制十字标记
            cross_size = int(r * 0.3)
            cv2.line(debug_image, (x - cross_size, y), (x + cross_size, y), (255, 0, 0), 2)
            cv2.line(debug_image, (x, y - cross_size), (x, y + cross_size), (255, 0, 0), 2)
            
            # 显示置信度
            cv2.putText(debug_image, f"Confidence: {confidence:.2f}", 
                       (x - 50, y - r - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        return debug_image
    
    def image_callback(self, msg):
        """
        图像回调函数
        """
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 图像预处理
            gray, binary = self.preprocess_image(cv_image)
            
            # 检测圆圈
            circles = self.detect_circles(binary)
            
            best_circle = None
            if circles is not None:
                # 找到最佳的降落平台
                best_circle = self.find_best_landing_platform(cv_image, circles)
            
            # 发布检测结果
            if best_circle is not None:
                x, y, r, confidence = best_circle
                
                # 创建Point消息
                center_point = Point()
                center_point.x = x
                center_point.y = y
                center_point.z = confidence  # 使用z字段存储置信度
                
                self.center_pub.publish(center_point)
                
                rospy.loginfo(f"Landing platform detected at ({x}, {y}) with confidence {confidence:.2f}")
            else:
                rospy.logwarn("No landing platform detected")
            
            # 发布调试图像
            debug_image = self.draw_debug_info(cv_image, circles, best_circle)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def run(self):
        """
        运行检测器
        """
        rospy.loginfo("Landing platform detector is running...")
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = LandingPlatformDetector()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Landing platform detector stopped")