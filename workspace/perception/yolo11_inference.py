#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
颜色检测参数说明：

检测方法选择：
    detection_method: 'yolo' 或 'color'
    - 'yolo': 使用YOLO模型检测
    - 'color': 使用颜色阈值检测

颜色阈值检测参数：
    min_contour_area: 最小轮廓面积，过滤小的噪声区域
    morphology_kernel_size: 形态学操作核大小，用于去噪
    gaussian_blur_size: 高斯模糊核大小，预处理减少噪声

HSV颜色范围：
    红色 (red): H=[0,10] 或 [170,180], S=[50,255], V=[50,255]
    黄色 (yellow): H=[20,30], S=[50,255], V=[50,255] 
    白色 (white): H=[0,180], S=[0,30], V=[200,255]

使用示例：
    rosrun your_package yolo11_inference.py _detection_method:=color
    rosrun your_package yolo11_inference.py _detection_method:=yolo
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import os
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_matrix
import threading
import struct

class YOLO11InferenceNode:
    def __init__(self):
        """初始化YOLO11推理节点"""
        rospy.init_node('yolo11_inference_node', anonymous=True)
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 获取参数
        self.input_topic = rospy.get_param('~input_topic', '/standard_vtol_0/camera/image_raw')
        self.output_topic = rospy.get_param('~output_topic', '/yolo11/detection_image')
        self.pose_topic = rospy.get_param('~pose_topic', '/standard_vtol_0/mavros/local_position/pose')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/standard_vtol_0/camera/camera_info')
        self.model_path = rospy.get_param('~model_path', 'models/yolo11n_original.pt')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.45)
        
        # 检测方法选择参数
        self.detection_method = rospy.get_param('~detection_method', 'yolo')  # 'yolo' 或 'color'
        
        # 颜色阈值检测参数
        self.color_detection_params = {
            'min_contour_area': rospy.get_param('~min_contour_area', 500),  # 最小轮廓面积
            'morphology_kernel_size': rospy.get_param('~morphology_kernel_size', 5),  # 形态学操作核大小
            'gaussian_blur_size': rospy.get_param('~gaussian_blur_size', 5),  # 高斯模糊核大小
        }
        
        # HSV颜色阈值范围
        self.color_ranges = {
            'red': {
                # 红色在HSV中有两个范围（因为红色在色调环的两端）
                'lower1': np.array([0, 50, 50]),
                'upper1': np.array([10, 255, 255]),
                'lower2': np.array([170, 50, 50]),
                'upper2': np.array([180, 255, 255])
            },
            'yellow': {
                'lower': np.array([20, 50, 50]),
                'upper': np.array([30, 255, 255])
            },
            'white': {
                'lower': np.array([0, 0, 200]),
                'upper': np.array([180, 30, 255])
            }
        }
        
        # 点云发布参数
        self.pointcloud_frame_id = rospy.get_param('~pointcloud_frame_id', 'map')
        self.max_points_per_cloud = rospy.get_param('~max_points_per_cloud', 1000)  # 每个点云最大点数
        self.publish_pointcloud = rospy.get_param('~publish_pointcloud', True)  # 是否发布点云
        
        # 初始化数据存储
        self.current_pose = None
        self.camera_info = None
        self.data_lock = threading.Lock()
        
        # 预初始化TF2系统
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 创建三个目标的点集
        self.target_points = {
            'red': [],
            'yellow': [],
            'white': []
        }
        
        # 相机变换参数
        self.camera_transform_params = self.get_camera_transform_params()
        
        # 根据检测方法加载相应资源
        if self.detection_method == 'yolo':
            # 加载YOLO11模型
            self.load_model()
            # 获取类别名称
            self.class_names = self.model.names
        else:
            # 颜色检测模式，定义类别名称
            self.model = None
            self.class_names = {0: 'red', 1: 'yellow', 2: 'white'}
        
        # 创建发布者和订阅者
        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.image_sub = rospy.Subscriber(self.input_topic, Image, self.image_callback)
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        
        # 创建点云发布者
        self.pointcloud_pubs = {}
        if self.publish_pointcloud:
            for target_name in self.target_points.keys():
                topic_name = f"/yolo11/pointcloud/{target_name}"
                self.pointcloud_pubs[target_name] = rospy.Publisher(topic_name, PointCloud2, queue_size=1)
                rospy.loginfo(f"点云发布话题: {topic_name}")
            
            # 创建定时器，定期发布点云
            self.pointcloud_timer = rospy.Timer(rospy.Duration(0.5), self.pointcloud_timer_callback)  # 2Hz发布
        
        rospy.loginfo(f"YOLO11推理节点已启动")
        rospy.loginfo(f"检测方法: {self.detection_method}")
        rospy.loginfo(f"订阅话题: {self.input_topic}")
        rospy.loginfo(f"发布话题: {self.output_topic}")
        rospy.loginfo(f"位姿话题: {self.pose_topic}")
        rospy.loginfo(f"相机内参话题: {self.camera_info_topic}")
        if self.detection_method == 'yolo':
            rospy.loginfo(f"模型路径: {self.model_path}")
            rospy.loginfo(f"置信度阈值: {self.confidence_threshold}")
            rospy.loginfo(f"IOU阈值: {self.iou_threshold}")
        else:
            rospy.loginfo(f"颜色检测参数: {self.color_detection_params}")
        rospy.loginfo(f"点云坐标系: {self.pointcloud_frame_id}")
        rospy.loginfo(f"每个点云最大点数: {self.max_points_per_cloud}")
        rospy.loginfo("TF2系统已初始化，等待变换数据...")
        
        # 等待TF系统建立
        rospy.sleep(1.0)
        
    def load_model(self):
        """加载YOLO11模型"""
        try:
            # 检查模型文件是否存在
            if not os.path.exists(self.model_path):
                rospy.logwarn(f"模型文件不存在: {self.model_path}，将使用默认的yolo11n.pt")
                self.model_path = 'yolo11n.pt'
            
            # 加载模型
            self.model = YOLO(self.model_path)
            rospy.loginfo(f"成功加载YOLO11模型: {self.model_path}")
            
            # 获取类别名称
            self.class_names = self.model.names
            rospy.loginfo(f"检测类别数量: {len(self.class_names)}")
            
        except Exception as e:
            rospy.logerr(f"加载模型失败: {str(e)}")
            rospy.signal_shutdown("模型加载失败")
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 根据检测方法运行不同的推理
            if self.detection_method == 'yolo':
                # 运行YOLO11推理
                results = self.model(cv_image, 
                                   conf=self.confidence_threshold, 
                                   iou=self.iou_threshold)
                result = results[0]
            else:
                # 运行颜色阈值检测
                detections = self.detect_by_color(cv_image)
                result = self.create_color_detection_result(detections)
            
            # 获取当前位姿和相机内参
            with self.data_lock:
                current_pose = self.current_pose
                current_camera_info = self.camera_info
            
            # 处理检测结果并更新点集
            self.process_detections_and_update_points(result, current_pose, current_camera_info)
            
            # 绘制检测结果
            annotated_image = self.draw_detections(cv_image, result)
            
            # 将处理后的图像转换回ROS消息格式并发布
            output_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            output_msg.header = msg.header  # 保持时间戳信息
            self.image_pub.publish(output_msg)
            
        except Exception as e:
            rospy.logerr(f"图像处理失败: {str(e)}")
    
    def draw_detections(self, image, result):
        """在图像上绘制检测框和标签"""
        annotated_image = image.copy()
        
        # 获取检测结果
        if result.boxes is not None and len(result.boxes) > 0:
            boxes = result.boxes.xyxy.cpu().numpy()  # 边界框坐标
            confidences = result.boxes.conf.cpu().numpy()  # 置信度
            class_ids = result.boxes.cls.cpu().numpy().astype(int)  # 类别ID
            
            # 为每个检测框绘制标注
            for box, conf, class_id in zip(boxes, confidences, class_ids):
                x1, y1, x2, y2 = box.astype(int)
                
                # 获取类别名称
                class_name = self.class_names[class_id] if class_id < len(self.class_names) else f"Class_{class_id}"
                
                # 创建标签文本
                label = f"{class_name}: {conf:.2f}"
                
                # 选择颜色（根据类别ID）
                color = self.get_color(class_id)
                
                # 绘制边界框
                cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, 2)
                
                # 计算标签背景大小
                (text_width, text_height), baseline = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                
                # 绘制标签背景
                cv2.rectangle(annotated_image, 
                            (x1, y1 - text_height - baseline - 5),
                            (x1 + text_width, y1), 
                            color, -1)
                
                # 绘制标签文本
                cv2.putText(annotated_image, label, 
                          (x1, y1 - baseline - 2),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                          (255, 255, 255), 1, cv2.LINE_AA)
            
            # 在图像上显示检测数量信息
            detection_count = len(boxes)
            count_text = f"检测到 {detection_count} 个目标 ({self.detection_method})"
            cv2.putText(annotated_image, count_text, 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, (0, 255, 0), 2, cv2.LINE_AA)
            
            # 显示点集统计信息
            summary = self.get_target_points_summary()
            y_offset = 60
            for target_name, info in summary.items():
                if info['count'] > 0:
                    stats_text = f"{target_name}: {info['count']} 个点"
                    cv2.putText(annotated_image, stats_text, 
                               (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 
                               0.5, (255, 255, 0), 1, cv2.LINE_AA)
                    y_offset += 25
        
        return annotated_image
    
    def get_color(self, class_id):
        """根据类别ID生成颜色"""
        # 使用固定的颜色列表，循环使用
        colors = [
            (255, 0, 0),    # 红色
            (0, 255, 0),    # 绿色
            (0, 0, 255),    # 蓝色
            (255, 255, 0),  # 黄色
            (255, 0, 255),  # 洋红
            (0, 255, 255),  # 青色
            (128, 0, 128),  # 紫色
            (255, 165, 0),  # 橙色
            (255, 192, 203), # 粉色
            (128, 128, 0),  # 橄榄色
        ]
        return colors[class_id % len(colors)]
    
    def pose_callback(self, msg):
        """位姿回调函数"""
        with self.data_lock:
            self.current_pose = msg
    
    def camera_info_callback(self, msg):
        """相机内参回调函数"""
        with self.data_lock:
            self.camera_info = msg
            rospy.loginfo_once("已获取相机内参信息")
    
    def pixel_to_world_coordinate(self, pixel_x, pixel_y, pose, camera_info):
        """将像素坐标转换为世界坐标（假设物体在地面上，Z=0）"""
        if camera_info is None or pose is None:
            return None
        
        try:
            # 第一步：计算物体相对于相机坐标系的齐次坐标
            # 获取相机内参
            fx = camera_info.K[0]  # 焦距x
            fy = camera_info.K[4]  # 焦距y
            cx = camera_info.K[2]  # 光心x
            cy = camera_info.K[5]  # 光心y
            
            # 将像素坐标转换为相机坐标系的齐次坐标（向右为x轴正方向，向下为y轴正方向）
            x0 = (pixel_x - cx) / fx
            y0 = (pixel_y - cy) / fy
            # 相机坐标系下的齐次坐标
            camera_point = np.array([x0, y0, 1.0])
            
            # 第二步：使用TF2将相机坐标系的两个点转换到地图坐标系
            # TF2 buffer和listener已在__init__中初始化
            
            # 获取从相机坐标系到地图坐标系的变换
            try:
                # 使用稍早的时间戳避免重复数据警告
                lookup_time = rospy.Time.now() - rospy.Duration(0.1)
                
                # 首先检查变换是否可用
                if self.tf_buffer.can_transform('map', 'standard_vtol_0/camera_link', lookup_time, rospy.Duration(0.1)):
                    transform = self.tf_buffer.lookup_transform(
                        'map',  # 目标坐标系
                        'standard_vtol_0/camera_link',  # 源坐标系
                        lookup_time,  # 使用稍早的时间戳
                        rospy.Duration(0.1)  # 超时时间
                    )
                else:
                    # 如果没有特定时间的变换，使用最新的
                    transform = self.tf_buffer.lookup_transform(
                        'map',  # 目标坐标系
                        'standard_vtol_0/camera_link',  # 源坐标系
                        rospy.Time(0),  # 最新的变换
                        rospy.Duration(0.5)  # 增加超时时间
                    )
                    
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn_throttle(5.0, f"无法获取TF变换: {e}")
                return None
            
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
            
            return intersection_point
            
        except Exception as e:
            rospy.logerr(f"坐标转换失败: {e}")
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
    
    def process_detections_and_update_points(self, result, pose, camera_info):
        """处理检测结果并更新点集"""
        if result.boxes is None or len(result.boxes) == 0:
            return
        
        if pose is None or camera_info is None:
            rospy.logwarn_throttle(5.0, "缺少位姿或相机内参信息，无法计算物体位置")
            return
        
        boxes = result.boxes.xyxy.cpu().numpy()  # 边界框坐标
        class_ids = result.boxes.cls.cpu().numpy().astype(int)  # 类别ID
        
        # 处理每个检测到的物体
        for box, class_id in zip(boxes, class_ids):
            # 获取类别名称
            if self.detection_method == 'yolo':
                class_name = self.class_names[class_id] if class_id < len(self.class_names) else f"Class_{class_id}"
            else:
                # 颜色检测模式，直接根据class_id映射
                class_name = self.class_names.get(class_id, f"Class_{class_id}")
            
            # 只处理我们关心的三个目标
            if class_name not in ['red', 'yellow', 'white']:
                continue
            
            # 计算检测框中心点的像素坐标
            x1, y1, x2, y2 = box
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            
            # 转换为世界坐标
            world_pos = self.pixel_to_world_coordinate(center_x, center_y, pose, camera_info)
            
            if world_pos is not None:
                # 添加到对应的点集中
                with self.data_lock:
                    self.target_points[class_name].append({
                        'position': world_pos.tolist(),
                        'timestamp': rospy.Time.now().to_sec(),
                        'pixel_coords': [center_x, center_y]
                    })
                
                rospy.loginfo(f"检测到{class_name}，世界坐标: [{world_pos[0]:.2f}, {world_pos[1]:.2f}, {world_pos[2]:.2f}]")
                rospy.loginfo(f"当前{class_name}点集大小: {len(self.target_points[class_name])}")
        
        # 处理完所有检测后，发布点云数据
        self.publish_pointclouds()
    
    def get_target_points_summary(self):
        """获取目标点集摘要信息"""
        with self.data_lock:
            summary = {}
            for target_name, points in self.target_points.items():
                summary[target_name] = {
                    'count': len(points),
                    'latest_position': points[-1]['position'] if points else None,
                    'latest_timestamp': points[-1]['timestamp'] if points else None
                }
            return summary
    
    def clear_target_points(self, target_name=None):
        """清空目标点集"""
        with self.data_lock:
            if target_name is None:
                # 清空所有点集
                for name in self.target_points:
                    self.target_points[name].clear()
                rospy.loginfo("已清空所有目标点集")
            elif target_name in self.target_points:
                self.target_points[target_name].clear()
                rospy.loginfo(f"已清空{target_name}目标点集")
            else:
                rospy.logwarn(f"未知的目标名称: {target_name}")
        
        # 发布更新后的点云（可能是空的）
        self.publish_pointclouds()
    
    def run(self):
        """运行节点"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("节点正在关闭...")

    def get_camera_transform_params(self):
        """获取相机到无人机的变换参数"""
        # 对于朝向正下方的相机，变换关系很简单
        # 相机朝向正下方：相机Z轴对应无人机-Z轴（向下）
        # 相机X轴对应无人机X轴（前向）
        # 相机Y轴对应无人机Y轴（右向）
        
        # 默认配置：相机朝向正下方
        default_rotation = [
            [1, 0, 0],   # 相机X轴 -> 无人机X轴（前向）
            [0, 1, 0],   # 相机Y轴 -> 无人机Y轴（右向）
            [0, 0, -1]   # 相机Z轴 -> 无人机-Z轴（向下）
        ]
        
        # 相机位于无人机正下方0.03m处
        default_translation = [0.0, 0.0, -0.03]  # 相机在无人机下方0.03m
        
        # 从ROS参数获取变换参数（如果有自定义的话）
        camera_rotation = rospy.get_param('~camera_rotation_matrix', default_rotation)
        camera_translation = rospy.get_param('~camera_translation', default_translation)
        
        transform_params = {
            'rotation': np.array(camera_rotation, dtype=np.float64),
            'translation': np.array(camera_translation, dtype=np.float64)
        }
        
        rospy.loginfo("相机变换参数（正下方配置）:")
        rospy.loginfo(f"旋转矩阵:\n{transform_params['rotation']}")
        rospy.loginfo(f"平移向量: {transform_params['translation']}")
        
        return transform_params

    def create_pointcloud2_message(self, points, target_name):
        """创建PointCloud2消息"""
        if not points:
            return None
        
        # 创建PointCloud2消息头
        header = rospy.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.pointcloud_frame_id
        
        # 定义点云字段
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('timestamp', 12, PointField.FLOAT64, 1),  # 添加时间戳字段
            PointField('intensity', 20, PointField.FLOAT32, 1),  # 用于存储置信度或其他信息
        ]
        
        # 限制点数
        if len(points) > self.max_points_per_cloud:
            # 保留最新的点
            points = points[-self.max_points_per_cloud:]
        
        # 准备点云数据
        cloud_data = []
        for point in points:
            position = point['position']
            timestamp = point['timestamp']
            
            # 根据目标类型设置强度值（用于在RViz中区分颜色）
            intensity_map = {'red': 1.0, 'yellow': 2.0, 'white': 3.0}
            intensity = intensity_map.get(target_name, 0.0)
            
            # 打包点数据（x, y, z, timestamp, intensity）
            point_data = struct.pack('fffdf', 
                                   position[0], position[1], position[2],
                                   timestamp, intensity)
            cloud_data.append(point_data)
        
        # 创建PointCloud2消息
        pointcloud = PointCloud2()
        pointcloud.header = header
        pointcloud.height = 1  # 无组织点云
        pointcloud.width = len(cloud_data)
        pointcloud.fields = fields
        pointcloud.is_bigendian = False
        pointcloud.point_step = 24  # 每个点的字节数 (4*3 + 8 + 4 = 24)
        pointcloud.row_step = pointcloud.point_step * pointcloud.width
        pointcloud.data = b''.join(cloud_data)
        pointcloud.is_dense = True  # 没有无效点
        
        return pointcloud
    
    def publish_pointclouds(self):
        """发布所有目标的点云数据"""
        if not self.publish_pointcloud:
            return
        
        with self.data_lock:
            for target_name, points in self.target_points.items():
                if points and target_name in self.pointcloud_pubs:
                    pointcloud_msg = self.create_pointcloud2_message(points, target_name)
                    if pointcloud_msg is not None:
                        self.pointcloud_pubs[target_name].publish(pointcloud_msg)
    
    def pointcloud_timer_callback(self, event):
        """定时发布点云数据"""
        self.publish_pointclouds()
    
    def get_pointcloud_statistics(self):
        """获取点云统计信息"""
        with self.data_lock:
            stats = {}
            for target_name, points in self.target_points.items():
                if points:
                    positions = np.array([point['position'] for point in points])
                    timestamps = [point['timestamp'] for point in points]
                    
                    stats[target_name] = {
                        'count': len(points),
                        'mean_position': np.mean(positions, axis=0).tolist(),
                        'std_position': np.std(positions, axis=0).tolist(),
                        'min_position': np.min(positions, axis=0).tolist(),
                        'max_position': np.max(positions, axis=0).tolist(),
                        'time_span': max(timestamps) - min(timestamps),
                        'latest_timestamp': max(timestamps)
                    }
                else:
                    stats[target_name] = {
                        'count': 0,
                        'mean_position': None,
                        'std_position': None,
                        'min_position': None,
                        'max_position': None,
                        'time_span': 0,
                        'latest_timestamp': None
                    }
            return stats
    
    def prune_old_points(self, max_age_seconds=300):
        """删除过期的点（默认5分钟）"""
        current_time = rospy.Time.now().to_sec()
        removed_count = 0
        
        with self.data_lock:
            for target_name, points in self.target_points.items():
                original_count = len(points)
                # 保留未过期的点
                self.target_points[target_name] = [
                    point for point in points 
                    if (current_time - point['timestamp']) <= max_age_seconds
                ]
                removed_count += original_count - len(self.target_points[target_name])
        
        if removed_count > 0:
            rospy.loginfo(f"删除了 {removed_count} 个过期点")
            # 发布更新后的点云
            self.publish_pointclouds()
    
    def detect_by_color(self, image):
        """使用颜色阈值检测目标"""
        # 转换到HSV颜色空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # 高斯模糊减少噪声
        blur_size = self.color_detection_params['gaussian_blur_size']
        if blur_size > 0:
            hsv = cv2.GaussianBlur(hsv, (blur_size, blur_size), 0)
        
        detections = []
        
        # 检测每种颜色
        for color_name, color_range in self.color_ranges.items():
            mask = self.create_color_mask(hsv, color_range)
            
            # 形态学操作去噪
            kernel_size = self.color_detection_params['morphology_kernel_size']
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 处理每个轮廓
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > self.color_detection_params['min_contour_area']:
                    # 计算边界框
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # 计算置信度（基于轮廓面积和形状特征）
                    confidence = self.calculate_color_confidence(contour, area)
                    
                    # 添加检测结果
                    detections.append({
                        'bbox': [x, y, x + w, y + h],
                        'class_name': color_name,
                        'class_id': list(self.color_ranges.keys()).index(color_name),
                        'confidence': confidence,
                        'contour_area': area
                    })
        
        return detections
    
    def create_color_mask(self, hsv, color_range):
        """创建颜色掩码"""
        if 'lower1' in color_range and 'upper1' in color_range:
            # 红色有两个范围
            mask1 = cv2.inRange(hsv, color_range['lower1'], color_range['upper1'])
            mask2 = cv2.inRange(hsv, color_range['lower2'], color_range['upper2'])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            # 其他颜色只有一个范围
            mask = cv2.inRange(hsv, color_range['lower'], color_range['upper'])
        
        return mask
    
    def calculate_color_confidence(self, contour, area):
        """计算颜色检测的置信度"""
        # 基于轮廓面积和形状特征计算置信度
        # 可以根据实际需要调整这个算法
        
        # 计算轮廓的周长
        perimeter = cv2.arcLength(contour, True)
        
        # 计算紧致度（4π*面积/周长²）
        if perimeter > 0:
            compactness = 4 * np.pi * area / (perimeter * perimeter)
        else:
            compactness = 0
        
        # 计算面积占比得分（面积越大，置信度越高，但有上限）
        area_score = min(area / 10000.0, 1.0)  # 假设10000像素为满分面积
        
        # 计算形状得分（越接近圆形得分越高）
        shape_score = min(compactness * 2, 1.0)  # 完美圆形的紧致度为1
        
        # 综合得分
        confidence = (area_score * 0.7 + shape_score * 0.3)
        confidence = max(0.1, min(confidence, 1.0))  # 限制在0.1-1.0之间
        
        return confidence
    
    def create_color_detection_result(self, detections):
        """将颜色检测结果转换为类似YOLO结果的格式"""
        class ColorDetectionResult:
            def __init__(self, detections):
                if detections:
                    # 转换为numpy数组格式
                    self.boxes = ColorDetectionBoxes(detections)
                else:
                    self.boxes = None
        
        class ColorDetectionBoxes:
            def __init__(self, detections):
                # 提取边界框坐标
                bboxes = [det['bbox'] for det in detections]
                self.xyxy = ColorDetectionTensor(np.array(bboxes, dtype=np.float32))
                
                # 提取置信度
                confidences = [det['confidence'] for det in detections]
                self.conf = ColorDetectionTensor(np.array(confidences, dtype=np.float32))
                
                # 提取类别ID
                class_ids = [det['class_id'] for det in detections]
                self.cls = ColorDetectionTensor(np.array(class_ids, dtype=np.int32))
        
        class ColorDetectionTensor:
            def __init__(self, data):
                self.data = data
            
            def cpu(self):
                return self
            
            def numpy(self):
                return self.data
        
        return ColorDetectionResult(detections)

    def prune_old_points(self, max_age_seconds=300):
        """删除过期的点（默认5分钟）"""
        current_time = rospy.Time.now().to_sec()
        removed_count = 0
        
        with self.data_lock:
            for target_name, points in self.target_points.items():
                original_count = len(points)
                # 保留未过期的点
                self.target_points[target_name] = [
                    point for point in points 
                    if (current_time - point['timestamp']) <= max_age_seconds
                ]
                removed_count += original_count - len(self.target_points[target_name])
        
        if removed_count > 0:
            rospy.loginfo(f"删除了 {removed_count} 个过期点")
            # 发布更新后的点云
            self.publish_pointclouds()

"""
颜色检测参数说明：

检测方法选择：
    detection_method: 'yolo' 或 'color'
    - 'yolo': 使用YOLO模型检测
    - 'color': 使用颜色阈值检测

颜色阈值检测参数：
    min_contour_area: 最小轮廓面积，过滤小的噪声区域
    morphology_kernel_size: 形态学操作核大小，用于去噪
    gaussian_blur_size: 高斯模糊核大小，预处理减少噪声

HSV颜色范围：
    红色 (red): H=[0,10] 或 [170,180], S=[50,255], V=[50,255]
    黄色 (yellow): H=[20,30], S=[50,255], V=[50,255] 
    白色 (white): H=[0,180], S=[0,30], V=[200,255]

使用示例：
    rosrun your_package yolo11_inference.py _detection_method:=color
    rosrun your_package yolo11_inference.py _detection_method:=yolo
"""

def main():
    """主函数"""
    try:
        node = YOLO11InferenceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"节点运行失败: {str(e)}")

if __name__ == '__main__':
    main()