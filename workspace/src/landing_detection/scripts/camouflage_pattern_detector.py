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
        rospy.init_node('white_square_detector', anonymous=True)
        
        self.bridge = CvBridge()
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber('/iris_0/camera/image_raw', Image, self.image_callback)
        
        # 订阅yolo12白色目标检测结果作为备用
        self.yolo_sub = rospy.Subscriber('/yolo12/pixel_position/white', Point, self.yolo_callback)
        
        # 发布降落点坐标
        self.target_pub = rospy.Publisher('/landing_target_camo', PointStamped, queue_size=1)
        
        # 发布调试图像
        self.debug_pub = rospy.Publisher('/landing_debug_camo', Image, queue_size=1)
        
        # yolo12检测结果缓存
        self.latest_yolo_detection = None
        self.yolo_detection_time = None
        
        # CV检测状态跟踪
        self.last_cv_detection_time = None
        self.cv_detection_timeout = 3.0  # CV检测超时时间（秒）
        self.cv_detection_lost = False
        
        # 检测参数
        self.white_threshold = 200  # 白色阈值
        self.min_circle_radius = 5  # 减小最小半边长，更好地检测远处的目标
        self.max_circle_radius = 200  # 正方形的最大半边长
        self.min_contour_area = 25   # 减小最小轮廓面积
        
        # 自适应面积参数
        self.use_adaptive_area = False  # 是否使用自适应面积阈值
        self.adaptive_area_min_ratio = 0.00001  # 相对于图像面积的最小比例
        self.adaptive_area_max_ratio = 0.75   # 相对于图像面积的最大比例
        self.min_detection_score = 0.65  # 最小检测得分
        
        # 白色正方形参数
        self.min_square_ratio = 0.7  # 最小方形度（宽高比）
        self.min_white_coverage = 0.7  # 最小白色覆盖率
        
        # 形态学操作参数
        self.close_kernel_size = 7  # 闭操作核大小
        self.open_kernel_size = 2   # 开操作核大小
        
        # 过滤和平滑参数
        self.detection_history = []  # 检测历史
        self.history_size = 5        # 历史大小
        self.position_smooth_factor = 0.3  # 位置平滑因子
        
        # 备用检测参数
        self.yolo_timeout = 2.0  # YOLO检测结果超时时间（秒）
        
        rospy.loginfo("Enhanced white square detector initialized")
    
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
        检测白色正方形 - 基于颜色阈值和轮廓检测
        """
        # 添加帧ID用于警告日志
        frame_id = rospy.Time.now().to_nsec()
        
        # 1. 转换为HSV颜色空间，更容易分离白色区域
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 2. 定义白色的HSV范围
        lower_white = np.array([0, 0, 200])  # 低饱和度，高亮度
        upper_white = np.array([180, 30, 255])
        
        # 3. 创建白色区域的掩码
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        
        # 检查白色掩码是否为空
        white_pixels = np.sum(white_mask > 0)
        if white_pixels == 0:
            rospy.logwarn(f"[Frame {frame_id}] 步骤3警告: 白色掩码中没有像素，HSV阈值可能需要调整")
        
        # 4. 应用更强的形态学操作，去除噪点并连接断开的区域
        # 先进行更强的闭操作来填充内部黑色噪声
        close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.close_kernel_size, self.close_kernel_size))
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, close_kernel)
        
        # 再进行开操作去除外部小的白色噪点
        open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.open_kernel_size, self.open_kernel_size))
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, open_kernel)
        
        # 检查形态学操作后的掩码是否为空
        morph_pixels = np.sum(white_mask > 0)
        if morph_pixels == 0:
            rospy.logwarn(f"[Frame {frame_id}] 步骤4警告: 形态学操作后白色掩码为空，可能需要调整形态学参数")
        
        # 对于更大的黑色噪声（如阴影），使用填充孔洞操作
        # 复制掩码用于填充
        fill_mask = white_mask.copy()
        
        # 获取掩码尺寸
        h, w = fill_mask.shape
        
        # 创建稍大一点的图像，确保边界连通
        flood_mask = np.zeros((h+2, w+2), np.uint8)
        
        # 从四个边界点进行洪水填充，填充背景
        for x,y in [(0,0), (0,h-1), (w-1,0), (w-1,h-1)]:
            cv2.floodFill(fill_mask, flood_mask, (x,y), 255)
        
        # 反转填充后的图像，得到孔洞
        holes = cv2.bitwise_not(fill_mask)
        
        # 将原始掩码与孔洞合并，填充所有内部孔洞
        white_mask = cv2.bitwise_or(white_mask, holes)
        
        # 检查填充孔洞后的掩码是否为空
        fill_pixels = np.sum(white_mask > 0)
        if fill_pixels == 0:
            rospy.logwarn(f"[Frame {frame_id}] 步骤4警告: 填充孔洞后白色掩码为空")
        
        # 5. 查找轮廓
        contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # 检查轮廓是否为空
        if len(contours) == 0:
            rospy.logwarn(f"[Frame {frame_id}] 步骤5警告: 未找到任何轮廓")
            return None
        
        best_square = None
        best_score = 0
        
        # 计算面积阈值
        img_area = image.shape[0] * image.shape[1]
        
        if self.use_adaptive_area:
            # 使用自适应面积阈值
            min_area = max(self.min_contour_area, int(img_area * self.adaptive_area_min_ratio))
            max_area = min(int(img_area * self.adaptive_area_max_ratio), self.max_circle_radius * self.max_circle_radius)
        else:
            # 使用固定面积阈值
            min_area = self.min_circle_radius * self.min_circle_radius
            max_area = self.max_circle_radius * self.max_circle_radius
        
        # 如果之前有检测到目标，使用更严格的筛选条件
        if False:
            last_detection = self.detection_history[-1]
            last_radius = last_detection.get('radius', 0)
            # 调整搜索范围，考虑目标大小可能的变化
            expected_min_area = max(min_area, (last_radius * 0.6) ** 2)
            expected_max_area = min(max_area, (last_radius * 1.4) ** 2)
        else:
            expected_min_area = min_area
            expected_max_area = max_area
        
        rospy.logdebug(f"Area thresholds - min: {expected_min_area}, max: {expected_max_area}")
        
        # 记录通过面积筛选的轮廓数量
        contours_after_area_filter = 0
        
        for contour in contours:
            # 计算轮廓面积
            area = cv2.contourArea(contour)
            
            # 面积过滤
            # if area < expected_min_area or area > expected_max_area:
            #     continue
            
            contours_after_area_filter += 1
            
            # 计算周长
            perimeter = cv2.arcLength(contour, True)
            
            # 多边形逼近，检测是否为方形
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
            
            # 检查是否为四边形
            if len(approx) == 4:
                # 计算矩形度
                rect = cv2.minAreaRect(contour)
                rect_area = rect[1][0] * rect[1][1]
                rect_ratio = area / rect_area if rect_area > 0 else 0
                
                # 计算方形度 (长宽比接近1)
                width, height = rect[1]
                aspect_ratio = min(width, height) / max(width, height) if max(width, height) > 0 else 0
                
                # 检查角点是否接近直角
                angle_score = self.check_corner_angles(approx)
                
                # 计算边的平行度
                parallelism_score = self.check_edge_parallelism(approx)
                
                # 计算总得分 (矩形度、方形度、角度和平行度的加权平均)
                score = (rect_ratio * 0.4 + aspect_ratio * 0.3 + 
                         angle_score * 0.15 + parallelism_score * 0.15)
                
                if score > best_score:
                    best_score = score
                    
                    # 计算中心点
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # 计算"半径" (正方形的半边长)
                        radius = int(math.sqrt(area) / 2)
                        
                        best_square = {
                            'center': (cx, cy),
                            'radius': radius,
                            'contour': contour,
                            'area': area,
                            'aspect_ratio': aspect_ratio,
                            'rect_ratio': rect_ratio
                        }
        
        # 检查面积筛选后是否没有轮廓
        if contours_after_area_filter == 0:
            rospy.logwarn(f"[Frame {frame_id}] 步骤5警告: 面积筛选后没有轮廓(min_area={expected_min_area}, max_area={expected_max_area})")
        
        # 检查四边形筛选后是否没有候选
        if best_square is None:
            rospy.logwarn(f"[Frame {frame_id}] 步骤5警告: 未找到符合要求的四边形")
            return None
        
        # 返回检测到的最佳正方形
        if best_square:
            return {
                'center': best_square['center'],
                'radius': best_square['radius'],
                'texture_score': 1.0,  # 简化评分
                'white_score': best_score,
                'cross_score': 1.0,
                'total_score': best_score
            }
        
        return None
    
    def analyze_white_square_quality(self, image, cx, cy, radius):
        """
        分析白色正方形的质量
        """
        # 转换为HSV颜色空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 创建正方形区域掩码
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        square_size = radius * 2
        half_size = radius
        top_left = (cx - half_size, cy - half_size)
        bottom_right = (cx + half_size, cy + half_size)
        cv2.rectangle(mask, top_left, bottom_right, 255, -1)
        
        # 提取区域
        h, s, v = cv2.split(hsv)
        roi_v = cv2.bitwise_and(v, v, mask=mask)
        roi_s = cv2.bitwise_and(s, s, mask=mask)
        
        # 计算白色质量（亮度高，饱和度低）
        if np.sum(mask == 255) == 0:
            return 0
            
        avg_v = np.sum(roi_v) / np.sum(mask == 255)  # 平均亮度
        avg_s = np.sum(roi_s) / np.sum(mask == 255)  # 平均饱和度
        
        # 白色质量得分 (亮度高，饱和度低为佳)
        v_score = min(avg_v / 200.0, 1.0)  # 亮度归一化
        s_score = max(0, 1.0 - avg_s / 50.0)  # 饱和度归一化（越低越好）
        
        white_score = v_score * 0.7 + s_score * 0.3
        
        return min(white_score, 1.0)
        
    def calculate_square_quality(self, contour):
        """
        计算检测到的正方形的质量
        """
        # 计算矩形拟合度
        rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        rect_area = cv2.contourArea(box)
        actual_area = cv2.contourArea(contour)
        
        if rect_area == 0:
            return 0
            
        fill_ratio = actual_area / rect_area
        
        # 计算方形度（宽高比接近1）
        width, height = rect[1]
        if max(width, height) == 0:
            return 0
            
        aspect_ratio = min(width, height) / max(width, height)
        
        # 综合得分
        quality_score = fill_ratio * 0.6 + aspect_ratio * 0.4
        
        return min(quality_score, 1.0)
    
    def check_white_square_quality(self, image, cx, cy, radius):
        """
        检查白色正方形的质量
        """
        # 转换为HSV以更好地检测白色
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 白色范围（在HSV空间中）
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        
        # 提取白色区域
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        
        # 创建正方形区域掩码
        square_mask = np.zeros(white_mask.shape, dtype=np.uint8)
        square_size = radius * 2
        half_size = radius
        top_left = (cx - half_size, cy - half_size)
        bottom_right = (cx + half_size, cy + half_size)
        cv2.rectangle(square_mask, top_left, bottom_right, 255, -1)
        
        # 计算重叠区域
        overlap = cv2.bitwise_and(white_mask, square_mask)
        white_pixels = np.sum(overlap == 255)
        total_pixels = np.sum(square_mask == 255)
        
        if total_pixels == 0:
            return 0
            
        # 计算白色覆盖率
        white_coverage = white_pixels / total_pixels
        
        return white_coverage
    
    def draw_debug_info(self, image, candidate):
        """
        绘制调试信息
        """
        debug_image = image.copy()
        
        # 显示当前阈值信息
        cv2.putText(debug_image, f"Min Area: {self.min_contour_area}", (10, 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(debug_image, f"Min Score: {self.min_detection_score:.2f}", (10, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # 显示检测状态
        status_text = "状态: "
        if self.cv_detection_lost:
            status_text += "使用YOLO备用" if self.is_yolo_detection_valid() else "无检测"
            status_color = (0, 0, 255)  # 红色
        else:
            status_text += "正常检测"
            status_color = (0, 255, 0)  # 绿色
        
        cv2.putText(debug_image, status_text, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color, 1)
        
        if candidate:
            cx, cy = candidate['center']
            radius = candidate['radius']
            
            # 根据检测来源选择颜色
            if candidate.get('source') == 'yolo12':
                box_color = (255, 165, 0)  # 橙色表示yolo12检测
                text_color = (255, 165, 0)
                source_text = "yolo12"
            else:
                box_color = (0, 255, 0)    # 绿色表示主检测器
                text_color = (0, 255, 255)
                source_text = "CV"
            
            # 绘制正方形边框
            square_size = radius * 2
            half_size = radius
            top_left = (cx - half_size, cy - half_size)
            bottom_right = (cx + half_size, cy + half_size)
            cv2.rectangle(debug_image, top_left, bottom_right, box_color, 3)
            
            # 绘制中心点
            cv2.circle(debug_image, (cx, cy), 5, (0, 0, 255), -1)
            
            # 绘制十字
            cross_size = radius // 2
            cv2.line(debug_image, (cx - cross_size, cy), (cx + cross_size, cy), (255, 0, 0), 2)
            cv2.line(debug_image, (cx, cy - cross_size), (cx, cy + cross_size), (255, 0, 0), 2)
            
            # 显示检测结果信息
            area = math.pi * radius * radius
            aspect_ratio = candidate.get('aspect_ratio', 1.0)
            
            info_text = [
                f"来源: {source_text}",
                f"总分: {candidate['total_score']:.2f}",
                f"白色得分: {candidate['white_score']:.2f}",
                f"面积: {int(area)}",
                f"边长: {radius*2}",
                f"方形度: {aspect_ratio:.2f}"
            ]
            
            for i, text in enumerate(info_text):
                cv2.putText(debug_image, text, (cx - 80, cy - radius - 60 + i * 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)
                           
            # 如果是平滑后的结果，添加指示
            if len(self.detection_history) > 1:
                cv2.putText(debug_image, "已平滑", (cx - 30, cy + radius + 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        return debug_image
    
    def image_callback(self, msg):
        """
        图像回调函数
        """
        try:
            # 添加帧ID用于警告日志
            frame_id = rospy.Time.now().to_nsec()
            
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 检测白色正方形降落平台
            candidate = self.detect_camouflage_circle(cv_image)  # 保留函数名但内部逻辑已更改
            
            # 如果初始检测失败，记录警告
            if candidate is None:
                rospy.logwarn(f"[Frame {frame_id}] 初始检测失败: detect_camouflage_circle返回None")
            
            # 如果检测成功且得分高于阈值
            if candidate is not None and candidate['total_score'] >= self.min_detection_score:
                # 应用平滑处理
                candidate = self.smooth_detection(candidate)
                # 更新CV检测状态为成功
                self.update_cv_detection_status(True)
            else:
                # 检测失败
                if candidate is not None:
                    rospy.logwarn(f"[Frame {frame_id}] 检测得分过低: {candidate['total_score']:.2f} < {self.min_detection_score}")
                self.update_cv_detection_status(False)
                candidate = None
            
            # 如果CV检测失败且超时，使用YOLO备用检测
            if candidate is None and self.should_use_yolo_backup():
                rospy.loginfo(f"[Frame {frame_id}] 使用yolo12备用检测")
                candidate = self.use_yolo_as_backup()
                
                # 检查YOLO备用检测结果
                if candidate is None:
                    rospy.logwarn(f"[Frame {frame_id}] yolo12备用检测失败: 返回None")
                else:
                    # 对YOLO结果也进行平滑处理
                    original_candidate = candidate.copy()
                    candidate = self.smooth_detection(candidate)
                    
                    # 检查平滑处理是否导致候选区域为空
                    if candidate is None:
                        rospy.logwarn(f"[Frame {frame_id}] 平滑处理后yolo12候选区域变为空")
                        # 恢复原始候选区域
                        candidate = original_candidate
            
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
                if source == 'yolo12':
                    rospy.loginfo(f"白色正方形降落平台通过yolo12备用检测在 {candidate['center']} 位置，置信度为 {candidate['total_score']:.2f}")
                else:
                    rospy.loginfo(f"白色正方形降落平台通过CV检测在 {candidate['center']} 位置，得分为 {candidate['total_score']:.2f}")
            else:
                if self.should_use_yolo_backup():
                    rospy.logwarn(f"[Frame {frame_id}] CV检测（超时）和yolo12备用都未检测到白色正方形降落平台")
                else:
                    rospy.logdebug("CV未检测到白色正方形降落平台（未超时）")
                    
            # 发布调试图像
            debug_image = self.draw_debug_info(cv_image, candidate)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            rospy.logerr(f"白色正方形检测器错误: {e}")
    
    def yolo_callback(self, msg):
        """
        yolo12白色目标检测结果回调函数
        """
        self.latest_yolo_detection = msg
        self.yolo_detection_time = rospy.Time.now()
        rospy.logdebug(f"Received yolo12 white detection: x={msg.x}, y={msg.y}, z={msg.z}")
    
    def is_yolo_detection_valid(self):
        """
        检查yolo12检测结果是否有效（未超时）
        """
        if self.latest_yolo_detection is None or self.yolo_detection_time is None:
            return False
        
        current_time = rospy.Time.now()
        time_diff = (current_time - self.yolo_detection_time).to_sec()
        
        return time_diff <= self.yolo_timeout
    
    def use_yolo_as_backup(self):
        """
        使用yolo12检测结果作为备用
        """
        # 添加帧ID用于警告日志
        frame_id = rospy.Time.now().to_nsec()
        
        if not self.is_yolo_detection_valid():
            rospy.logwarn(f"[Frame {frame_id}] yolo12备用检测结果无效或超时")
            return None
        
        # 创建候选对象，格式与主检测器一致
        yolo_candidate = {
            'center': (int(self.latest_yolo_detection.x), int(self.latest_yolo_detection.y)),
            'radius': 50,  # 默认半径，可以根据需要调整
            'white_score': 0.9,  # 较高的白色得分，因为YOLO专门检测白色目标
            'texture_score': 1.0,  # 不再需要纹理分析
            'cross_score': 1.0,  # 不再需要十字分析
            'total_score': float(self.latest_yolo_detection.z) if self.latest_yolo_detection.z > 0 else 0.75,  # 使用z值作为置信度
            'source': 'yolo12'  # 标记数据来源
        }
        
        rospy.loginfo(f"使用yolo12备用检测，白色目标位置：({yolo_candidate['center'][0]}, {yolo_candidate['center'][1]})，置信度：{yolo_candidate['total_score']:.2f}")
        
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
    
    def run(self):
        rospy.loginfo("白色正方形检测器正在运行...")
        rospy.spin()

    def check_corner_angles(self, points):
        """
        检查四边形的四个角是否接近直角
        返回角度得分(越接近直角得分越高)
        """
        if len(points) != 4:
            return 0
            
        # 重新排列点，确保它们是按顺序的
        points = points.reshape(-1, 2)
        center = np.mean(points, axis=0)
        
        # 计算每个点相对于中心点的角度
        angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
        # 按角度排序点
        sorted_idx = np.argsort(angles)
        sorted_points = points[sorted_idx]
        
        # 计算每个角的角度
        angles_diff = []
        for i in range(4):
            p1 = sorted_points[i]
            p2 = sorted_points[(i+1) % 4]
            p3 = sorted_points[(i+2) % 4]
            
            # 计算两条边的向量
            v1 = p1 - p2
            v2 = p3 - p2
            
            # 计算两条边的夹角(弧度)
            dot_product = np.dot(v1, v2)
            norm_v1 = np.linalg.norm(v1)
            norm_v2 = np.linalg.norm(v2)
            
            if norm_v1 == 0 or norm_v2 == 0:
                return 0
                
            cos_angle = dot_product / (norm_v1 * norm_v2)
            # 限制在-1到1之间，避免数值误差
            cos_angle = max(-1, min(1, cos_angle))
            angle = np.arccos(cos_angle)
            
            # 计算与90度的接近程度
            angle_diff = abs(angle - np.pi/2)
            angles_diff.append(angle_diff)
        
        # 计算角度得分(0到1，越接近直角越接近1)
        max_diff = np.pi/2  # 最大可能差异
        angle_scores = [1 - (diff / max_diff) for diff in angles_diff]
        
        # 取所有角度得分的平均
        return np.mean(angle_scores)
    
    def check_edge_parallelism(self, points):
        """
        检查四边形的对边是否平行
        返回平行度得分(越平行得分越高)
        """
        if len(points) != 4:
            return 0
            
        # 重新排列点，确保它们是按顺序的
        points = points.reshape(-1, 2)
        center = np.mean(points, axis=0)
        
        # 计算每个点相对于中心点的角度
        angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
        # 按角度排序点
        sorted_idx = np.argsort(angles)
        sorted_points = points[sorted_idx]
        
        # 计算两组对边的方向向量
        edge1 = sorted_points[1] - sorted_points[0]
        edge3 = sorted_points[3] - sorted_points[2]
        
        edge2 = sorted_points[2] - sorted_points[1]
        edge4 = sorted_points[0] - sorted_points[3]
        
        # 归一化向量
        norm1 = np.linalg.norm(edge1)
        norm2 = np.linalg.norm(edge2)
        norm3 = np.linalg.norm(edge3)
        norm4 = np.linalg.norm(edge4)
        
        if norm1 == 0 or norm2 == 0 or norm3 == 0 or norm4 == 0:
            return 0
            
        edge1_norm = edge1 / norm1
        edge2_norm = edge2 / norm2
        edge3_norm = edge3 / norm3
        edge4_norm = edge4 / norm4
        
        # 计算对边的平行度(点积的绝对值，越接近1越平行)
        parallel1 = abs(np.dot(edge1_norm, edge3_norm))
        parallel2 = abs(np.dot(edge2_norm, edge4_norm))
        
        # 计算平行度得分(越接近1越好)
        parallelism_score = (parallel1 + parallel2) / 2
        
        return parallelism_score
    
    def smooth_detection(self, current_detection):
        """
        对检测结果进行时间序列平滑处理，减少抖动和误检
        """
        if current_detection is None:
            rospy.logwarn("[平滑处理] 输入检测结果为空")
            return None
        
        # 添加帧ID用于警告日志
        frame_id = rospy.Time.now().to_nsec()
        
        # 添加时间戳
        current_detection['timestamp'] = rospy.Time.now()
        
        # 添加到历史记录
        self.detection_history.append(current_detection)
        
        # 保持历史记录大小
        if len(self.detection_history) > self.history_size:
            self.detection_history.pop(0)
        
        # 如果历史记录不足，直接返回当前检测
        if len(self.detection_history) < 3:
            return current_detection
        
        # 计算历史检测的置信度加权平均
        total_weight = 0
        smoothed_center_x = 0
        smoothed_center_y = 0
        smoothed_radius = 0
        smoothed_score = 0
        
        current_time = rospy.Time.now()
        
        for i, detection in enumerate(self.detection_history):
            # 根据时间计算权重（越近的检测权重越大）
            time_diff = (current_time - detection['timestamp']).to_sec()
            recency_weight = max(0, 1.0 - time_diff / 2.0)  # 2秒前的检测权重降为0
            
            # 根据置信度计算权重
            confidence_weight = detection['total_score']
            
            # 综合权重
            weight = recency_weight * confidence_weight
            total_weight += weight
            
            # 加权累加
            cx, cy = detection['center']
            smoothed_center_x += cx * weight
            smoothed_center_y += cy * weight
            smoothed_radius += detection['radius'] * weight
            smoothed_score += detection['total_score'] * weight
        
        if total_weight > 0:
            # 归一化
            smoothed_center_x /= total_weight
            smoothed_center_y /= total_weight
            smoothed_radius /= total_weight
            smoothed_score /= total_weight
            
            # 对当前检测和平滑结果进行混合
            cx, cy = current_detection['center']
            alpha = self.position_smooth_factor
            
            smoothed_detection = current_detection.copy()
            smoothed_detection['center'] = (
                int(alpha * cx + (1-alpha) * smoothed_center_x),
                int(alpha * cy + (1-alpha) * smoothed_center_y)
            )
            smoothed_detection['radius'] = int(alpha * current_detection['radius'] + 
                                               (1-alpha) * smoothed_radius)
            smoothed_detection['total_score'] = alpha * current_detection['total_score'] + \
                                                (1-alpha) * smoothed_score
            
            return smoothed_detection
        else:
            rospy.logwarn(f"[Frame {frame_id}] 平滑处理警告: 总权重为0，无法进行平滑")
        
        return current_detection

if __name__ == '__main__':
    try:
        detector = CamouflagePatternDetector()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("白色正方形检测器已停止")
