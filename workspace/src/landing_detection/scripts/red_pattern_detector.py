#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from cv_bridge import CvBridge
import math
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_matrix

class RedPatternDetector:
    def __init__(self):
        rospy.init_node('red_pattern_detector', anonymous=True)
        
        self.bridge = CvBridge()
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber('/iris_0/camera/image_raw', Image, self.image_callback)
        
        # 订阅位姿和相机内参话题用于世界坐标计算
        self.pose_sub = rospy.Subscriber('/iris_0/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.camera_info_sub = rospy.Subscriber('/iris_0/camera/camera_info', CameraInfo, self.camera_info_callback)
        
        # 订阅YOLO12红色目标检测结果作为备用
        self.yolo_sub = rospy.Subscriber('/yolo12/pixel_position/red', Point, self.yolo_callback)
        
        # 发布降落点坐标
        self.target_pub = rospy.Publisher('/landing_target_red', PointStamped, queue_size=1)
        
        # 发布世界坐标
        self.world_coord_pub = rospy.Publisher('/landing_target_red/world_coord', Point, queue_size=1)
        
        # 发布调试图像
        self.debug_pub = rospy.Publisher('/landing_debug_red', Image, queue_size=1)
        
        # 位姿和相机内参数据
        self.current_pose = None
        self.camera_info = None
        
        # 初始化TF2系统
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.camera_frame_id = 'iris_0/camera_link'
        
        # YOLO12检测结果缓存
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
        
        # YOLO12备用检测参数
        self.yolo_timeout = 2.0  # YOLO检测结果超时时间（秒）
        
        rospy.loginfo("Red pattern detector initialized")
        rospy.loginfo("等待TF系统建立...")
        
        # 等待TF系统建立
        rospy.sleep(1.0)
    
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
            if candidate.get('source') == 'yolo12':
                circle_color = (255, 165, 0)  # 橙色表示YOLO12检测
                text_color = (255, 165, 0)
                source_text = "YOLO12"
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
                # 为YOLO12检测创建空的掩码用于调试显示
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
                
                # 计算并发布世界坐标
                if self.current_pose is not None and self.camera_info is not None:
                    world_pos = self.pixel_to_world_coordinate(
                        candidate['center'][0], candidate['center'][1], 
                        self.current_pose, self.camera_info
                    )
                    
                    if world_pos is not None:
                        world_coord_msg = Point()
                        world_coord_msg.x = world_pos[0]
                        world_coord_msg.y = world_pos[1] 
                        world_coord_msg.z = world_pos[2]
                        self.world_coord_pub.publish(world_coord_msg)
                        
                        # 根据检测来源显示不同的日志信息
                        source = candidate.get('source', 'cv')
                        if source == 'yolo12':
                            rospy.loginfo(f"红色降落平台通过YOLO12备用检测在像素位置 {candidate['center']}，世界坐标 [{world_pos[0]:.2f}, {world_pos[1]:.2f}, {world_pos[2]:.2f}]，置信度为 {candidate['total_score']:.2f}")
                        else:
                            rospy.loginfo(f"红色降落平台通过CV检测在像素位置 {candidate['center']}，世界坐标 [{world_pos[0]:.2f}, {world_pos[1]:.2f}, {world_pos[2]:.2f}]，得分为 {candidate['total_score']:.2f}")
                    else:
                        rospy.logwarn("无法计算世界坐标")
                else:
                    rospy.logwarn_throttle(5.0, "缺少位姿或相机内参信息，无法计算世界坐标")
                
                # 根据检测来源显示不同的日志信息（备用，如果没有世界坐标）
                source = candidate.get('source', 'cv')
                if source == 'yolo12':
                    rospy.logdebug(f"Red platform detected via YOLO12 backup at {candidate['center']} with confidence {candidate['total_score']:.2f}")
                else:
                    rospy.logdebug(f"Red platform detected via CV at {candidate['center']} with score {candidate['total_score']:.2f}")

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
        YOLO12红色目标检测结果回调函数
        """
        self.latest_yolo_detection = msg
        self.yolo_detection_time = rospy.Time.now()
        #rospy.logdebug(f"Received YOLO12 detection: x={msg.x}, y={msg.y}, z={msg.z}")
    
    def is_yolo_detection_valid(self):
        """
        检查YOLO12检测结果是否有效（未超时）
        """
        if self.latest_yolo_detection is None or self.yolo_detection_time is None:
            return False
        
        current_time = rospy.Time.now()
        time_diff = (current_time - self.yolo_detection_time).to_sec()
        
        return time_diff <= self.yolo_timeout
    
    def use_yolo_as_backup(self):
        """
        使用YOLO12检测结果作为备用
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
            'source': 'yolo12'  # 标记数据来源
        }
        
        rospy.loginfo(f"Using YOLO12 backup detection at ({yolo_candidate['center'][0]}, {yolo_candidate['center'][1]}) with confidence {yolo_candidate['total_score']:.2f}")
        
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
    
    def pose_callback(self, msg):
        """位姿回调函数"""
        self.current_pose = msg
        
    def camera_info_callback(self, msg):
        """相机内参回调函数"""
        self.camera_info = msg

    def pixel_to_world_coordinate(self, pixel_x, pixel_y, pose, camera_info):
        """将像素坐标转换为世界坐标（假设物体在地面上，Z=0）"""
        if camera_info is None or pose is None:
            return None
        
        try:
            # 记录开始时间用于超时检测
            start_time = rospy.Time.now()
            
            # 第一步：计算物体相对于相机坐标系的齐次坐标
            # 获取相机内参
            fx = camera_info.K[0]  # 焦距x
            fy = camera_info.K[4]  # 焦距y
            cx = camera_info.K[2]  # 光心x
            cy = camera_info.K[5]  # 光心y
            
            # 将像素坐标转换为相机坐标系的齐次坐标（向右为x轴正方向，向下为y轴正方向）
            x0 = (pixel_x - cx) / fx
            y0 = (pixel_y - cy) / fy
            # 相机坐标系下的齐 homogeneous coordinates
            camera_point = np.array([x0, y0, 1.0])
            
            # 第二步：使用TF2将相机坐标系的两个点转换到地图坐标系
            # 获取从相机坐标系到地图坐标系的变换
            try:
                # 首先尝试获取最新的变换（非阻塞）
                if self.tf_buffer.can_transform('map', self.camera_frame_id, rospy.Time(0), rospy.Duration(0.01)):
                    transform = self.tf_buffer.lookup_transform(
                        'map',  # 目标坐标系
                        self.camera_frame_id,  # 源坐标系
                        rospy.Time(0),  # 最新的变换
                        rospy.Duration(0.1)  # 较短的超时时间
                    )
                else:
                    # 如果最新变换不可用，尝试稍早的时间戳
                    lookup_time = rospy.Time.now() - rospy.Duration(0.2)
                    if self.tf_buffer.can_transform('map', self.camera_frame_id, lookup_time, rospy.Duration(0.01)):
                        transform = self.tf_buffer.lookup_transform(
                            'map',  # 目标坐标系
                            self.camera_frame_id,  # 源坐标系
                            lookup_time,  # 使用稍早的时间戳
                            rospy.Duration(0.1)  # 较短的超时时间
                        )
                    else:
                        rospy.logwarn_throttle(5.0, f"TF变换不可用: map -> {self.camera_frame_id}")
                        return None
                    
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn_throttle(5.0, f"无法获取TF变换: {e}")
                return None
            
            # 检查是否超时
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            if elapsed_time > 0.5:  # 500ms超时
                rospy.logwarn(f"TF查找耗时过长: {elapsed_time:.2f}秒")
            
            # 定义相机坐标系中的两个点
            # 点1：相机光心 (0, 0, 0)
            point1_camera = np.array([0.0, 0.0, 0.0, 1.0])  # 齐次坐标
            # 点2：物体在相机坐标系中的位置 (x0, y0, 1)
            point2_camera = np.array([x0, y0, 1.0, 1.0])  # 齐次坐标
            
            # 将变换转换为4x4矩阵
            transform_matrix = self.transform_to_matrix(transform)
            
            # 将两个点转换到地图坐标系
            point1_map = transform_matrix @ point1_camera
            point2_map = transform_matrix @ point2_camera
            
            # 转换为3D点（去除齐次坐标的最后一维）
            point1_map_3d = point1_map[:3]
            point2_map_3d = point2_map[:3]
            
            # 第三步：计算直线与地面(z=0)的交点
            # 直线方程：P = point1_map_3d + t * (point2_map_3d - point1_map_3d)
            # 地面方程：z = 0
            
            direction = point2_map_3d - point1_map_3d
            
            # 检查直线是否平行于地面
            if abs(direction[2]) < 1e-6:
                rospy.logwarn_throttle(1.0, "直线平行于地面，无法计算交点")
                return None
            
            # 求解参数t使得 point1_map_3d[2] + t * direction[2] = 0
            t = -point1_map_3d[2] / direction[2]
            
            # 检查交点是否在射线的正方向上
            if t < 0:
                rospy.logwarn_throttle(1.0, "交点在相机后方")
                return None
            
            # 计算交点
            intersection_point = point1_map_3d + t * direction
            intersection_point[2] = 0.0  # 确保Z坐标为0
            
            # 记录总处理时间
            total_time = (rospy.Time.now() - start_time).to_sec()
            if total_time > 0.1:  # 100ms以上记录警告
                rospy.logwarn_throttle(2.0, f"坐标转换耗时: {total_time:.3f}秒")
            
            return intersection_point
            
        except Exception as e:
            rospy.logerr(f"坐标转换失败: {e}")
            import traceback
            rospy.logerr(f"错误堆栈: {traceback.format_exc()}")
            return None
    
    def transform_to_matrix(self, transform):
        """将TF2变换转换为4x4变换矩阵"""
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        
        # 从四元数创建旋转矩阵
        quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
        rotation_matrix = quaternion_matrix(quaternion)
        
        # 设置平移
        rotation_matrix[0, 3] = translation.x
        rotation_matrix[1, 3] = translation.y
        rotation_matrix[2, 3] = translation.z
        
        return rotation_matrix

if __name__ == '__main__':
    try:
        detector = RedPatternDetector()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Red pattern detector stopped")
