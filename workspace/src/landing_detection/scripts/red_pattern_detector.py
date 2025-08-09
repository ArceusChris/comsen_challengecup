#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge
import math

class RedPatternDetector:
    def __init__(self):
        rospy.init_node('red_pattern_detector', anonymous=True)
        
        self.bridge = CvBridge()
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber('/iris_0/camera/image_raw', Image, self.image_callback)
        
        # 订阅YOLO11红色目标检测结果作为备用
        self.yolo_sub = rospy.Subscriber('/yolo11/pixel_position/red', Point, self.yolo_callback)
        
        # 发布降落点坐标
        self.target_pub = rospy.Publisher('/landing_target_red', PointStamped, queue_size=1)
        
        # 发布调试图像
        self.debug_pub = rospy.Publisher('/landing_debug_red', Image, queue_size=1)
        
        # YOLO11检测结果缓存
        self.latest_yolo_detection = None
        self.yolo_detection_time = None
        
        # CV检测状态跟踪
        self.last_cv_detection_time = None
        self.cv_detection_timeout = 3.0  # CV检测超时时间（秒）
        self.cv_detection_lost = False
        
        # 红色检测参数 (HSV色彩空间)
        # 红色在HSV中有两个范围：0-10和170-180
        self.red_lower1 = np.array([0, 43, 46])    # 红色范围1下界
        self.red_upper1 = np.array([10, 255, 255])   # 红色范围1上界
        self.red_lower2 = np.array([156, 43, 46])  # 红色范围2下界
        self.red_upper2 = np.array([180, 255, 255])  # 红色范围2上界
        

        
        # 几何参数
        self.min_circle_radius = 5
        self.max_circle_radius = 250
        self.min_contour_area = 5
        self.circularity_threshold = 0.5
        
        # 检测参数
        self.red_area_threshold = 0.6  # 红色区域面积比例阈值
        
        # YOLO11备用检测参数
        self.yolo_timeout = 2.0  # YOLO检测结果超时时间（秒）
        
        rospy.loginfo("Red pattern detector initialized")
    
    def create_red_mask(self, hsv_image):
        """
        创建红色掩码 - 处理红色在HSV空间的特殊性
        """
        # 红色在HSV中跨越0度，需要两个范围
        mask1 = cv2.inRange(hsv_image, self.red_lower1, self.red_upper1)
        mask2 = cv2.inRange(hsv_image, self.red_lower2, self.red_upper2)
        
        # 合并两个红色范围
        red_mask = cv2.bitwise_or(mask1, mask2)
        
        # 形态学操作去噪声
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 1))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        
        return red_mask
    

    
    def detect_red_circle(self, image):
        """
        检测红色圆形降落平台
        """
        # 转换为HSV色彩空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 创建红色掩码
        red_mask = self.create_red_mask(hsv)
        
        # 寻找红色区域的轮廓
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        candidates = []
        
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
                radius = int(radius)
                
                if self.min_circle_radius <= radius <= self.max_circle_radius:
                    # 验证红色区域比例
                    red_ratio = self.calculate_red_ratio(red_mask, center, radius)
                    
                    # 简化评分，只考虑红色比例和圆形度
                    total_score = red_ratio * 0.7 + circularity * 0.3
                    
                    if total_score > 0.5:  # 总体阈值
                        candidates.append({
                            'center': center,
                            'radius': radius,
                            'red_ratio': red_ratio,
                            'white_score': 0,  # 保持兼容性
                            'cross_score': 0,  # 保持兼容性
                            'circularity': circularity,
                            'total_score': total_score
                        })
        
        # 返回得分最高的候选
        if candidates:
            best_candidate = max(candidates, key=lambda x: x['total_score'])
            return best_candidate, red_mask
        
        return None, red_mask
    
    def calculate_red_ratio(self, red_mask, center, radius):
        """
        计算圆形区域内红色像素的比例
        """
        # 创建圆形掩码
        circle_mask = np.zeros(red_mask.shape, dtype=np.uint8)
        cv2.circle(circle_mask, center, radius, 255, -1)
        
        # 计算红色像素比例
        red_in_circle = cv2.bitwise_and(red_mask, circle_mask)
        red_pixels = np.sum(red_in_circle == 255)
        total_pixels = np.sum(circle_mask == 255)
        
        if total_pixels == 0:
            return 0
        
        red_ratio = red_pixels / total_pixels
        return red_ratio
    

    
    def draw_debug_info(self, image, candidate, red_mask):
        """
        绘制调试信息
        """
        debug_image = image.copy()
        
        # 创建彩色掩码叠加
        red_overlay = cv2.cvtColor(red_mask, cv2.COLOR_GRAY2BGR)
        red_overlay[:, :, 1:] = 0  # 只保留红色通道
        
        # 叠加掩码（半透明）
        debug_image = cv2.addWeighted(debug_image, 0.7, red_overlay, 0.3, 0)
        
        if candidate:
            cx, cy = candidate['center']
            radius = candidate['radius']
            
            # 根据检测来源选择颜色
            if candidate.get('source') == 'yolo11':
                circle_color = (255, 165, 0)  # 橙色表示YOLO11检测
                text_color = (255, 165, 0)
                source_text = "YOLO11"
            else:
                circle_color = (0, 255, 0)    # 绿色表示主检测器
                text_color = (0, 255, 255)
                source_text = "CV"
            
            # 绘制外圆
            cv2.circle(debug_image, (cx, cy), radius, circle_color, 3)
            
            # 绘制中心点
            cv2.circle(debug_image, (cx, cy), 5, (0, 0, 255), -1)
            
            # 绘制十字
            cross_size = radius // 4
            cv2.line(debug_image, (cx - cross_size, cy), (cx + cross_size, cy), (255, 0, 255), 3)
            cv2.line(debug_image, (cx, cy - cross_size), (cx, cy + cross_size), (255, 0, 255), 3)
            
            # 显示得分信息
            info_text = [
                f"Source: {source_text}",
                f"Total: {candidate['total_score']:.2f}",
                f"Red ratio: {candidate['red_ratio']:.2f}",
                f"Circularity: {candidate['circularity']:.2f}"
            ]
            
            for i, text in enumerate(info_text):
                cv2.putText(debug_image, text, (cx - 100, cy - radius - 80 + i * 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)
        
        return debug_image
    
    def image_callback(self, msg):
        """
        图像回调函数
        """
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 检测红色降落平台
            candidate, red_mask = self.detect_red_circle(cv_image)
            
            # 更新CV检测状态
            cv_detection_successful = candidate is not None
            self.update_cv_detection_status(cv_detection_successful)
            
            # 如果CV检测失败且超时，或者CV检测成功但检测丢失状态下需要双重确认，使用YOLO备用检测
            if candidate is None and self.should_use_yolo_backup():
                candidate = self.use_yolo_as_backup()
                # 为YOLO11检测创建空的掩码用于调试显示
                if candidate is not None:
                    red_mask = np.zeros(cv_image.shape[:2], dtype=np.uint8)
            
            # 发布结果
            if candidate is not None:
                target_msg = PointStamped()
                target_msg.header.stamp = rospy.Time.now()
                target_msg.header.frame_id = "camera"
                
                target_msg.point.x = candidate['center'][0]
                target_msg.point.y = candidate['center'][1]
                target_msg.point.z = candidate['total_score']
                
                self.target_pub.publish(target_msg)
                
                # 根据检测来源显示不同的日志信息
                source = candidate.get('source', 'cv')
                if source == 'yolo11':
                    rospy.loginfo(f"Red platform detected via YOLO11 backup at {candidate['center']} with confidence {candidate['total_score']:.2f}")
                else:
                    rospy.loginfo(f"Red platform detected via CV at {candidate['center']} with score {candidate['total_score']:.2f}")

            debug_image = self.draw_debug_info(cv_image, candidate, red_mask)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_pub.publish(debug_msg)
            
            # 更新CV检测状态
            self.update_cv_detection_status(candidate is not None)
            
        except Exception as e:
            rospy.logerr(f"Error in red detector: {e}")
    
    def run(self):
        rospy.loginfo("Red pattern detector running...")
        rospy.spin()
    
    def yolo_callback(self, msg):
        """
        YOLO11红色目标检测结果回调函数
        """
        self.latest_yolo_detection = msg
        self.yolo_detection_time = rospy.Time.now()
        #rospy.logdebug(f"Received YOLO11 detection: x={msg.x}, y={msg.y}, z={msg.z}")
    
    def is_yolo_detection_valid(self):
        """
        检查YOLO11检测结果是否有效（未超时）
        """
        if self.latest_yolo_detection is None or self.yolo_detection_time is None:
            return False
        
        current_time = rospy.Time.now()
        time_diff = (current_time - self.yolo_detection_time).to_sec()
        
        return time_diff <= self.yolo_timeout
    
    def use_yolo_as_backup(self):
        """
        使用YOLO11检测结果作为备用
        """
        if not self.is_yolo_detection_valid():
            return None
        
        # 创建候选对象，格式与主检测器一致
        yolo_candidate = {
            'center': (int(self.latest_yolo_detection.x), int(self.latest_yolo_detection.y)),
            'radius': 50,  # 默认半径，可以根据需要调整
            'red_ratio': 0.7,  # 假设YOLO检测到的是合格的红色目标
            'white_score': 0.5,  # 中等白色得分
            'cross_score': 0.5,  # 中等十字得分
            'circularity': 0.8,  # 假设YOLO检测到的目标圆形度较好
            'total_score': float(self.latest_yolo_detection.z) if self.latest_yolo_detection.z > 0 else 0.6,  # 使用z值作为置信度
            'source': 'yolo11'  # 标记数据来源
        }
        
        rospy.loginfo(f"Using YOLO11 backup detection at ({yolo_candidate['center'][0]}, {yolo_candidate['center'][1]}) with confidence {yolo_candidate['total_score']:.2f}")
        
        return yolo_candidate
    
    def update_cv_detection_status(self, detection_successful):
        """
        更新CV检测状态
        """
        current_time = rospy.Time.now()
        
        if detection_successful:
            self.last_cv_detection_time = current_time
            if self.cv_detection_lost:
                rospy.loginfo("CV detection recovered")
                self.cv_detection_lost = False
        else:
            # 检查是否超时
            if self.last_cv_detection_time is not None:
                time_since_last_detection = (current_time - self.last_cv_detection_time).to_sec()
                if time_since_last_detection > self.cv_detection_timeout and not self.cv_detection_lost:
                    rospy.logwarn(f"CV detection lost for {time_since_last_detection:.1f}s, switching to YOLO backup")
                    self.cv_detection_lost = True
            else:
                # 首次启动时没有检测到目标
                if not self.cv_detection_lost:
                    rospy.logwarn("Initial CV detection failed, will use YOLO backup if available")
                    self.cv_detection_lost = True
    
    def should_use_yolo_backup(self):
        """
        判断是否应该使用YOLO备用检测
        """
        return self.cv_detection_lost

if __name__ == '__main__':
    try:
        detector = RedPatternDetector()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Red pattern detector stopped")
