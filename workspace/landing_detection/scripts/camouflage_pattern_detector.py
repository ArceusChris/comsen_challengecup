#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge
import math

class CamouflagePatternDetector:
    def __init__(self):
        rospy.init_node('camouflage_pattern_detector', anonymous=True)
        
        self.bridge = CvBridge()
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber('/iris_0/camera/image_raw', Image, self.image_callback)
        
        # 发布降落点坐标
        self.target_pub = rospy.Publisher('/landing_target_camo', PointStamped, queue_size=1)
        
        # 发布调试图像
        self.debug_pub = rospy.Publisher('/landing_debug_camo', Image, queue_size=1)
        
        # 检测参数
        self.white_threshold = 200  # 白色阈值
        self.min_circle_radius = 30
        self.max_circle_radius = 200
        self.min_contour_area = 1000
        
        # 迷彩图案特征参数
        self.texture_window_size = 15  # 纹理分析窗口大小
        self.edge_threshold = 50  # 边缘检测阈值
        self.circularity_threshold = 0.6  # 圆形度阈值
        
        rospy.loginfo("Camouflage pattern detector initialized")
    
    def preprocess_image(self, image):
        """
        图像预处理 - 针对迷彩图案优化
        """
        # 转换为灰度图
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 自适应直方图均衡化，增强对比度
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        enhanced = clahe.apply(gray)
        
        # 高斯滤波
        blurred = cv2.GaussianBlur(enhanced, (5, 5), 0)
        
        return gray, enhanced, blurred
    
    def detect_camouflage_circle(self, image):
        """
        检测迷彩圆形 - 基于边缘和纹理特征
        """
        gray, enhanced, blurred = self.preprocess_image(image)
        
        # 边缘检测
        edges = cv2.Canny(blurred, self.edge_threshold, self.edge_threshold * 2)
        
        # 形态学操作连接断裂的边缘
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        # 霍夫圆变换检测圆形
        circles = cv2.HoughCircles(
            edges,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=80,
            param1=50,
            param2=30,
            minRadius=self.min_circle_radius,
            maxRadius=self.max_circle_radius
        )
        
        candidates = []
        
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            
            for (x, y, r) in circles:
                # 验证迷彩纹理特征
                texture_score = self.analyze_camouflage_texture(enhanced, x, y, r)
                
                # 检测内部白色圆圈
                white_circle_score = self.detect_inner_white_circle(image, x, y, r)
                
                # 检测十字标记
                cross_score = self.detect_cross_in_center(image, x, y, r)
                
                # 综合评分
                total_score = texture_score * 0.4 + white_circle_score * 0.3 + cross_score * 0.3
                
                if total_score > 0.5:  # 阈值可调
                    candidates.append({
                        'center': (x, y),
                        'radius': r,
                        'texture_score': texture_score,
                        'white_score': white_circle_score,
                        'cross_score': cross_score,
                        'total_score': total_score
                    })
        
        # 返回得分最高的候选
        if candidates:
            best_candidate = max(candidates, key=lambda x: x['total_score'])
            return best_candidate
        
        return None
    
    def analyze_camouflage_texture(self, gray_image, cx, cy, radius):
        """
        分析迷彩纹理特征
        """
        # 提取圆形区域
        mask = np.zeros(gray_image.shape[:2], dtype=np.uint8)
        cv2.circle(mask, (cx, cy), radius, 255, -1)
        
        # 排除内部白色圆圈区域（假设内部白色圆圈半径为外圆的60%）
        inner_radius = int(radius * 0.6)
        cv2.circle(mask, (cx, cy), inner_radius, 0, -1)
        
        # 提取迷彩区域
        roi = cv2.bitwise_and(gray_image, gray_image, mask=mask)
        
        # 计算纹理特征
        # 1. 灰度方差 - 迷彩图案应该有较高的灰度变化
        pixels = roi[mask == 255]
        if len(pixels) == 0:
            return 0
        
        gray_variance = np.var(pixels)
        variance_score = min(gray_variance / 1000.0, 1.0)  # 归一化
        
        # 2. 边缘密度 - 迷彩图案边缘较多
        edges = cv2.Canny(roi, 50, 150)
        edge_pixels = cv2.bitwise_and(edges, edges, mask=mask)
        edge_density = np.sum(edge_pixels == 255) / np.sum(mask == 255)
        
        # 3. 局部二值模式(LBP)纹理特征
        lbp_score = self.calculate_lbp_texture(roi, mask)
        
        # 综合纹理得分
        texture_score = variance_score * 0.4 + edge_density * 0.3 + lbp_score * 0.3
        
        return min(texture_score, 1.0)
    
    def calculate_lbp_texture(self, image, mask):
        """
        计算局部二值模式纹理特征
        """
        lbp = np.zeros_like(image)
        
        for i in range(1, image.shape[0] - 1):
            for j in range(1, image.shape[1] - 1):
                if mask[i, j] == 0:
                    continue
                    
                center = image[i, j]
                code = 0
                
                # 8邻域比较
                neighbors = [
                    image[i-1, j-1], image[i-1, j], image[i-1, j+1],
                    image[i, j+1], image[i+1, j+1], image[i+1, j],
                    image[i+1, j-1], image[i, j-1]
                ]
                
                for k, neighbor in enumerate(neighbors):
                    if neighbor >= center:
                        code |= (1 << k)
                
                lbp[i, j] = code
        
        # 计算LBP直方图的熵作为纹理复杂度指标
        hist = cv2.calcHist([lbp], [0], mask, [256], [0, 256])
        hist = hist.flatten()
        hist = hist / (np.sum(hist) + 1e-7)
        
        entropy = -np.sum(hist * np.log2(hist + 1e-7))
        
        return min(entropy / 8.0, 1.0)  # 归一化
    
    def detect_inner_white_circle(self, image, cx, cy, outer_radius):
        """
        检测内部白色圆圈
        """
        # 转换为HSV以更好地检测白色
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 白色范围（在HSV空间中）
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        
        # 在外圆内部寻找白色圆圈
        search_mask = np.zeros(white_mask.shape, dtype=np.uint8)
        cv2.circle(search_mask, (cx, cy), outer_radius, 255, -1)
        
        white_in_circle = cv2.bitwise_and(white_mask, search_mask)
        
        # 寻找白色区域的轮廓
        contours, _ = cv2.findContours(white_in_circle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_score = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 500:  # 最小面积阈值
                continue
            
            # 计算圆形度
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
                
            circularity = 4 * math.pi * area / (perimeter * perimeter)
            
            if circularity > self.circularity_threshold:
                # 检查是否在中心附近
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    contour_cx = int(M["m10"] / M["m00"])
                    contour_cy = int(M["m01"] / M["m00"])
                    
                    # 计算与外圆中心的距离
                    distance = math.sqrt((contour_cx - cx)**2 + (contour_cy - cy)**2)
                    distance_score = max(0, 1 - distance / (outer_radius * 0.3))
                    
                    score = circularity * distance_score
                    best_score = max(best_score, score)
        
        return best_score
    
    def detect_cross_in_center(self, image, cx, cy, radius):
        """
        检测中心十字标记
        """
        # 转换为灰度图
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 二值化处理
        _, binary = cv2.threshold(gray, self.white_threshold, 255, cv2.THRESH_BINARY)
        
        # 在中心区域检测十字
        search_radius = min(radius // 4, 30)
        
        # 检测水平线
        horizontal_score = 0
        for dy in range(-3, 4):
            y = cy + dy
            if 0 <= y < binary.shape[0]:
                line_pixels = 0
                total_pixels = 0
                for dx in range(-search_radius, search_radius + 1):
                    x = cx + dx
                    if 0 <= x < binary.shape[1]:
                        total_pixels += 1
                        if binary[y, x] == 255:
                            line_pixels += 1
                
                if total_pixels > 0:
                    score = line_pixels / total_pixels
                    horizontal_score = max(horizontal_score, score)
        
        # 检测垂直线
        vertical_score = 0
        for dx in range(-3, 4):
            x = cx + dx
            if 0 <= x < binary.shape[1]:
                line_pixels = 0
                total_pixels = 0
                for dy in range(-search_radius, search_radius + 1):
                    y = cy + dy
                    if 0 <= y < binary.shape[0]:
                        total_pixels += 1
                        if binary[y, x] == 255:
                            line_pixels += 1
                
                if total_pixels > 0:
                    score = line_pixels / total_pixels
                    vertical_score = max(vertical_score, score)
        
        # 十字得分
        cross_score = min(horizontal_score, vertical_score)
        return cross_score
    
    def draw_debug_info(self, image, candidate):
        """
        绘制调试信息
        """
        debug_image = image.copy()
        
        if candidate:
            cx, cy = candidate['center']
            radius = candidate['radius']
            
            # 绘制外圆
            cv2.circle(debug_image, (cx, cy), radius, (0, 255, 0), 3)
            
            # 绘制中心点
            cv2.circle(debug_image, (cx, cy), 5, (0, 0, 255), -1)
            
            # 绘制十字
            cross_size = radius // 4
            cv2.line(debug_image, (cx - cross_size, cy), (cx + cross_size, cy), (255, 0, 0), 2)
            cv2.line(debug_image, (cx, cy - cross_size), (cx, cy + cross_size), (255, 0, 0), 2)
            
            # 显示得分信息
            info_text = [
                f"Total: {candidate['total_score']:.2f}",
                f"Texture: {candidate['texture_score']:.2f}",
                f"White: {candidate['white_score']:.2f}",
                f"Cross: {candidate['cross_score']:.2f}"
            ]
            
            for i, text in enumerate(info_text):
                cv2.putText(debug_image, text, (cx - 80, cy - radius - 40 + i * 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        return debug_image
    
    def image_callback(self, msg):
        """
        图像回调函数
        """
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 检测迷彩降落平台
            candidate = self.detect_camouflage_circle(cv_image)
            
            # 发布结果
            if candidate:
                target_msg = PointStamped()
                target_msg.header.stamp = rospy.Time.now()
                target_msg.header.frame_id = "camera"
                
                target_msg.point.x = candidate['center'][0]
                target_msg.point.y = candidate['center'][1]
                target_msg.point.z = candidate['total_score']
                
                self.target_pub.publish(target_msg)
                
                rospy.loginfo(f"Camouflage platform detected at {candidate['center']} with score {candidate['total_score']:.2f}")
            else:
                rospy.logwarn("No camouflage platform detected")
            
            # 发布调试图像
            debug_image = self.draw_debug_info(cv_image, candidate)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            rospy.logerr(f"Error in camouflage detector: {e}")
    
    def run(self):
        rospy.loginfo("Camouflage pattern detector running...")
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = CamouflagePatternDetector()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Camouflage pattern detector stopped")
