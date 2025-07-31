#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge
import math

class SimpleLandingDetector:
    def __init__(self):
        rospy.init_node('simple_landing_detector', anonymous=True)
        
        self.bridge = CvBridge()
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber('/iris_0/camera/image_raw', Image, self.image_callback)
        
        # 发布降落点坐标（像素坐标）
        self.target_pub = rospy.Publisher('/landing_target', PointStamped, queue_size=1)
        
        # 发布调试图像
        self.debug_pub = rospy.Publisher('/landing_debug', Image, queue_size=1)
        
        # 检测参数
        self.white_lower = np.array([200, 200, 200])  # 白色下界
        self.white_upper = np.array([255, 255, 255])  # 白色上界
        self.min_contour_area = 1000  # 最小轮廓面积
        self.circularity_threshold = 0.6  # 圆形度阈值
        
        rospy.loginfo("Simple landing detector started")
    
    def detect_landing_platform(self, image):
        """
        检测降落平台
        """
        # 创建白色掩码
        mask = cv2.inRange(image, self.white_lower, self.white_upper)
        
        # 形态学操作去噪
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # 找到轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_candidate = None
        best_score = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_contour_area:
                continue
            
            # 计算圆形度
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
                
            circularity = 4 * math.pi * area / (perimeter * perimeter)
            
            if circularity > self.circularity_threshold:
                # 获取最小外接圆
                (x, y), radius = cv2.minEnclosingCircle(contour)
                center = (int(x), int(y))
                
                # 检查中心区域是否有十字结构
                cross_score = self.check_cross_pattern(mask, center, int(radius))
                
                # 综合评分
                total_score = circularity * 0.5 + cross_score * 0.5
                
                if total_score > best_score:
                    best_score = total_score
                    best_candidate = {
                        'center': center,
                        'radius': int(radius),
                        'score': total_score,
                        'circularity': circularity,
                        'cross_score': cross_score
                    }
        
        return best_candidate, mask
    
    def check_cross_pattern(self, mask, center, radius):
        """
        检查圆心附近是否有十字图案
        """
        cx, cy = center
        
        # 定义十字检查区域
        check_radius = min(radius // 3, 20)
        
        # 检查水平线
        horizontal_score = 0
        for x in range(max(0, cx - check_radius), min(mask.shape[1], cx + check_radius)):
            if mask[cy, x] == 255:
                horizontal_score += 1
        horizontal_score = horizontal_score / (2 * check_radius) if check_radius > 0 else 0
        
        # 检查垂直线
        vertical_score = 0
        for y in range(max(0, cy - check_radius), min(mask.shape[0], cy + check_radius)):
            if mask[y, cx] == 255:
                vertical_score += 1
        vertical_score = vertical_score / (2 * check_radius) if check_radius > 0 else 0
        
        # 十字得分是水平和垂直得分的最小值
        return min(horizontal_score, vertical_score)
    
    def draw_debug_info(self, image, candidate, mask):
        """
        绘制调试信息
        """
        debug_image = image.copy()
        
        # 将掩码叠加到原图上
        mask_colored = cv2.applyColorMap(mask, cv2.COLORMAP_HOT)
        debug_image = cv2.addWeighted(debug_image, 0.7, mask_colored, 0.3, 0)
        
        if candidate:
            center = candidate['center']
            radius = candidate['radius']
            
            # 绘制检测到的圆圈
            cv2.circle(debug_image, center, radius, (0, 255, 0), 3)
            cv2.circle(debug_image, center, 5, (0, 255, 0), -1)
            
            # 绘制十字
            cross_size = radius // 3
            cv2.line(debug_image, 
                    (center[0] - cross_size, center[1]), 
                    (center[0] + cross_size, center[1]), 
                    (255, 0, 0), 2)
            cv2.line(debug_image, 
                    (center[0], center[1] - cross_size), 
                    (center[0], center[1] + cross_size), 
                    (255, 0, 0), 2)
            
            # 显示信息
            info_text = f"Score: {candidate['score']:.2f}"
            cv2.putText(debug_image, info_text, 
                       (center[0] - 50, center[1] - radius - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            circ_text = f"Circ: {candidate['circularity']:.2f}"
            cv2.putText(debug_image, circ_text, 
                       (center[0] - 50, center[1] - radius - 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            cross_text = f"Cross: {candidate['cross_score']:.2f}"
            cv2.putText(debug_image, cross_text, 
                       (center[0] - 50, center[1] - radius - 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        return debug_image
    
    def image_callback(self, msg):
        """
        图像回调函数
        """
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 检测降落平台
            candidate, mask = self.detect_landing_platform(cv_image)
            
            # 发布结果
            if candidate and candidate['score'] > 0.3:  # 设置最低置信度阈值
                point_msg = PointStamped()
                point_msg.header.stamp = rospy.Time.now()
                point_msg.header.frame_id = "camera_frame"
                point_msg.point.x = candidate['center'][0]
                point_msg.point.y = candidate['center'][1]
                point_msg.point.z = candidate['score']
                
                self.target_pub.publish(point_msg)
                
                rospy.loginfo(f"Landing target: ({candidate['center'][0]}, {candidate['center'][1]}) "
                             f"Score: {candidate['score']:.2f}")
            else:
                rospy.logdebug("No suitable landing target found")
            
            # 发布调试图像
            debug_image = self.draw_debug_info(cv_image, candidate, mask)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            rospy.logerr(f"Error in image processing: {e}")

    def run(self):
        rospy.loginfo("Simple landing detector running...")
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = SimpleLandingDetector()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Simple landing detector stopped")
