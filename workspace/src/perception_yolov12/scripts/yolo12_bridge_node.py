#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
YOLOv12桥接节点，负责：
1. 订阅相机图像
2. 将图像发送给外部YOLOv12推理脚本
3. 接收检测结果并以ROS消息形式发布

使用示例：
    rosrun perception_yolov12 yolo12_bridge_node.py _image_topic:=/iris_0/camera/image_raw
    rosrun perception_yolov12 yolo12_bridge_node.py _image_topic:=/standard_vtol_0/camera/image_raw
    
    # 使用不同端口（适用于同时运行多个推理节点）
    rosrun perception_yolov12 yolo12_bridge_node.py _image_topic:=/iris_0/camera/image_raw _sender_port:=5555 _receiver_port:=5556
    rosrun perception_yolov12 yolo12_bridge_node.py _image_topic:=/standard_vtol_0/camera/image_raw _sender_port:=5557 _receiver_port:=5558
"""

import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, Point, Pose
from std_msgs.msg import String, Int8, Header
from cv_bridge import CvBridge, CvBridgeError
import json
import zmq
import numpy as np
import time
import threading
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_matrix
import struct

class Yolo12BridgeNode:
    def __init__(self):
        """初始化YOLOv12桥接节点"""
        rospy.init_node('yolo12_bridge_node', anonymous=True)
        
        # 获取机型参数
        self.aircraft_type = rospy.get_param('~aircraft_type', 'standard_vtol')  # 'iris' 或 'standard_vtol'
        
        # 获取ZeroMQ端口参数（用于支持多个推理节点）
        self.sender_port = rospy.get_param('~sender_port', 5555)
        self.receiver_port = rospy.get_param('~receiver_port', 5556)
        
        # 根据机型配置话题
        if self.aircraft_type == 'iris':
            # iris机型的话题配置
            self.image_topic = rospy.get_param('~image_topic', '/iris_0/camera/image_raw')
            self.pose_topic = rospy.get_param('~pose_topic', '/iris_0/mavros/local_position/pose')
            self.camera_info_topic = rospy.get_param('~camera_info_topic', '/iris_0/camera/camera_info')
            self.camera_frame_id = 'iris_0/camera_link'
            self.detections_topic = rospy.get_param('~detections_topic', '/iris_0/yolo12/detections')
            self.annotated_image_topic = rospy.get_param('~annotated_image_topic', '/iris_0/yolo12/annotated_image')
        elif self.aircraft_type == 'standard_vtol':
            # standard_vtol机型的话题配置
            self.image_topic = rospy.get_param('~image_topic', '/standard_vtol_0/camera/image_raw')
            self.pose_topic = rospy.get_param('~pose_topic', '/standard_vtol_0/mavros/local_position/pose')
            self.camera_info_topic = rospy.get_param('~camera_info_topic', '/standard_vtol_0/camera/camera_info')
            self.camera_frame_id = 'standard_vtol_0/camera_link'
            self.detections_topic = rospy.get_param('~detections_topic', '/standard_vtol_0/yolo12/detections')
            self.annotated_image_topic = rospy.get_param('~annotated_image_topic', '/standard_vtol_0/yolo12/annotated_image')
        else:
            rospy.logerr(f"不支持的机型: {self.aircraft_type}，支持的机型: 'iris', 'standard_vtol'")
            rospy.signal_shutdown("不支持的机型")
            return
        
        # 其他通用参数
        self.pointcloud_frame_id = rospy.get_param('~pointcloud_frame_id', 'map')
        self.max_points_per_cloud = rospy.get_param('~max_points_per_cloud', 1000)  # 每个点云最大点数
        self.publish_pointcloud = rospy.get_param('~publish_pointcloud', False)  # 暂时禁用点云发布用于调试
        
        # 初始化数据存储
        self.current_pose = None
        self.camera_info = None
        self.data_lock = threading.Lock()
        
        # VTOL着陆标志状态 (用于控制点云发布)
        self.vtol_land_flag = 0  # 默认为0
        self.vtol_flag_lock = threading.Lock()
        
        # 预初始化TF2系统
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 创建三个目标的点集
        self.target_points = {
            'red': [],
            'yellow': [],
            'white': []
        }
        
        # 相机变换参数 (从原始yolo11代码移植)
        self.camera_transform_params = self.get_camera_transform_params()
        
        # Publishers
        self.detections_pub = rospy.Publisher(self.detections_topic, String, queue_size=10)
        self.annotated_image_pub = rospy.Publisher(self.annotated_image_topic, Image, queue_size=10)
        
        # 创建实时位置发布者
        self.position_pubs = {}
        for target_name in self.target_points.keys():
            topic_name = f"/yolo12/position/{target_name}"
            self.position_pubs[target_name] = rospy.Publisher(topic_name, Point, queue_size=1)
            rospy.loginfo(f"实时位置发布话题: {topic_name}")
        
        # 创建位姿估计发布者（用于发布平均位置）
        self.pose_estimation_pubs = {}
        for target_name in self.target_points.keys():
            topic_name = f"/yolo12/pose_estimation/{target_name}"
            self.pose_estimation_pubs[target_name] = rospy.Publisher(topic_name, Point, queue_size=1)
            rospy.loginfo(f"位姿估计发布话题: {topic_name}")
        
        # 创建像素坐标发布者（用于发布实时像素坐标）
        self.pixel_position_pubs = {}
        for target_name in self.target_points.keys():
            topic_name = f"/yolo12/pixel_position/{target_name}"
            self.pixel_position_pubs[target_name] = rospy.Publisher(topic_name, Point, queue_size=1)
            rospy.loginfo(f"像素坐标发布话题: {topic_name}")
        
        # 创建VTOL目标位姿发布者（仅在VTOL机型时）
        self.vtol_target_pose_pubs = {}
        if self.aircraft_type == 'standard_vtol':
            target_topics = {
                'red': '/zhihang2025/first_man/pose',
                'yellow': '/zhihang2025/second_man/pose',
                'white': '/zhihang2025/third_man/pose'
            }
            for target_name, topic_name in target_topics.items():
                self.vtol_target_pose_pubs[target_name] = rospy.Publisher(topic_name, Pose, queue_size=1)
                rospy.loginfo(f"VTOL目标位姿发布话题: {topic_name}")
        
        # 创建点云发布者
        self.pointcloud_pubs = {}
        if self.publish_pointcloud:
            for target_name in self.target_points.keys():
                topic_name = f"/yolo12/pointcloud/{target_name}"
                self.pointcloud_pubs[target_name] = rospy.Publisher(topic_name, PointCloud2, queue_size=1)
                rospy.loginfo(f"点云发布话题: {topic_name}")
            
            # 创建定时器，定期发布点云
            self.pointcloud_timer = rospy.Timer(rospy.Duration(0.5), self.pointcloud_timer_callback)  # 2Hz发布
            rospy.loginfo("点云定时器已启用，2Hz发布频率")
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # ZeroMQ context and sockets
        self.context = zmq.Context()
        self.sender = self.context.socket(zmq.PUSH)
        
        # 绑定发送端口
        self.sender.bind(f"tcp://127.0.0.1:{self.sender_port}")
        rospy.loginfo(f"图像发送端口: {self.sender_port}")
        
        self.receiver = self.context.socket(zmq.PULL)
        
        # 绑定接收端口
        self.receiver.bind(f"tcp://127.0.0.1:{self.receiver_port}")
        rospy.loginfo(f"结果接收端口: {self.receiver_port}")
        
        # Subscribers
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        
        # 订阅VTOL着陆标志（仅在VTOL机型时）
        if self.aircraft_type == 'standard_vtol':
            self.vtol_flag_sub = rospy.Subscriber('/zhihang2025/vtol_land_sub/done', Int8, self.vtol_flag_callback)
            rospy.loginfo("已订阅VTOL着陆标志话题: /zhihang2025/vtol_land_sub/done")
        
        # Start result listener thread
        self.receive_thread = threading.Thread(target=self.receive_results)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        
        rospy.loginfo(f"Yolo12BridgeNode 初始化成功. 订阅 {self.image_topic}")
        rospy.loginfo(f"机型: {self.aircraft_type}")
        rospy.loginfo(f"订阅话题: {self.image_topic}")
        rospy.loginfo(f"发布话题: {self.annotated_image_topic}")
        rospy.loginfo(f"位姿话题: {self.pose_topic}")
        rospy.loginfo(f"相机内参话题: {self.camera_info_topic}")
        rospy.loginfo(f"相机坐标系: {self.camera_frame_id}")
        rospy.loginfo(f"点云坐标系: {self.pointcloud_frame_id}")
        rospy.loginfo(f"每个点云最大点数: {self.max_points_per_cloud}")
        rospy.loginfo("TF2系统已初始化，等待变换数据...")
        
        # 等待TF系统建立
        rospy.sleep(1.0)
    
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
    
    def pose_callback(self, msg):
        """位姿回调函数，存储当前无人机位置"""
        with self.data_lock:
            self.current_pose = msg

    def camera_info_callback(self, msg):
        """相机内参回调函数"""
        with self.data_lock:
            self.camera_info = msg
            rospy.loginfo_once("已获取相机内参信息")
    
    def vtol_flag_callback(self, msg):
        """VTOL着陆标志回调函数"""
        with self.vtol_flag_lock:
            old_flag = self.vtol_land_flag
            self.vtol_land_flag = msg.data
            if old_flag != self.vtol_land_flag:
                rospy.loginfo(f"VTOL着陆标志更新: {old_flag} -> {self.vtol_land_flag}")
                if self.vtol_land_flag == 2:
                    pass
                elif self.vtol_land_flag == 4:
                    self.calculate_and_publish_average_positions()
                    # 发布目标位姿到VTOL话题
                    self.publish_vtol_target_poses()
                else:
                    pass

            if self.vtol_land_flag == 3:
                self.calculate_and_publish_average_positions()
                # 发布目标位姿到VTOL话题
                self.publish_vtol_target_poses()
    
    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 编码图像为JPEG并发送到外部脚本
            _, jpeg_img = cv2.imencode('.jpg', cv_image)
            img_data = jpeg_img.tobytes()
            
            # 发送图像数据以及头信息
            self.sender.send_multipart([b"image", img_data, bytes(msg.header.frame_id, 'utf-8')])
            
        except CvBridgeError as e:
            rospy.logerr(e)
    
    def receive_results(self):
        """线程函数：接收来自YOLO处理脚本的结果"""
        while not rospy.is_shutdown():
            try:
                # 接收检测结果和标注图像
                frames = self.receiver.recv_multipart()
                
                if len(frames) == 3 and frames[0] == b"result":
                    # 解析检测结果
                    detections_json = frames[1].decode('utf-8')
                    self.detections_pub.publish(detections_json)
                    
                    # 解码并发布标注图像
                    frame_id = frames[2].decode('utf-8')
                    img_data = np.frombuffer(self.receiver.recv(), dtype=np.uint8)
                    annotated_img = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
                    
                    # 将标注图像转换回ROS图像消息
                    img_msg = self.bridge.cv2_to_imgmsg(annotated_img, "bgr8")
                    img_msg.header.stamp = rospy.Time.now()
                    img_msg.header.frame_id = frame_id
                    self.annotated_image_pub.publish(img_msg)
                    
                    # 处理检测结果，更新位置信息
                    self.process_detections(detections_json)
                    
            except Exception as e:
                rospy.logerr(f"接收结果时出错: {e}")
                import traceback
                rospy.logerr(f"错误堆栈: {traceback.format_exc()}")
                time.sleep(0.1)
    
    def process_detections(self, detections_json):
        """处理检测结果，更新位置信息"""
        try:
            detections = json.loads(detections_json)
            
            # 获取当前位姿和相机内参
            with self.data_lock:
                current_pose = self.current_pose
                current_camera_info = self.camera_info
            
            # 检查必要的数据是否可用
            if current_pose is None or current_camera_info is None:
                return
            
            # 按类别处理检测结果
            for detection in detections:
                # 获取目标类别和置信度
                class_name = detection["name"].lower()
                confidence = detection["confidence"]
                
                # 目标类别到颜色的映射
                target_color = None
                if "red" in class_name or "person" in class_name:
                    target_color = "red"
                elif "yellow" in class_name:
                    target_color = "yellow"
                elif "white" in class_name:
                    target_color = "white"
                else:
                    continue  # 不是我们关注的目标
                
                # 获取边界框中心坐标
                x_min, y_min = detection["xmin"], detection["ymin"]
                x_max, y_max = detection["xmax"], detection["ymax"]
                center_x = (x_min + x_max) / 2.0
                center_y = (y_min + y_max) / 2.0
                
                # 发布像素坐标
                pixel_msg = Point(x=center_x, y=center_y, z=0.0)
                self.pixel_position_pubs[target_color].publish(pixel_msg)
                
                # 计算3D位置（使用TF2方法）
                point_3d = self.pixel_to_world_coordinate(center_x, center_y, current_pose, current_camera_info)
                
                if point_3d is not None and self.vtol_land_flag >= 2 and self.vtol_land_flag <= 3:
                    # 添加到对应的点集中（包含时间戳和像素坐标）
                    point_data = {
                        'position': point_3d.tolist(),
                        'timestamp': rospy.Time.now().to_sec(),
                        'pixel_coords': [center_x, center_y]
                    }
                    
                    # 如果达到最大点数，则移除最旧的点
                    if len(self.target_points[target_color]) >= self.max_points_per_cloud:
                        self.target_points[target_color].pop(0)
                    self.target_points[target_color].append(point_data)
                    
                    # 发布实时位置
                    self.publish_realtime_position(target_color, point_3d)
                    
                    # 计算并发布平均位置
                    self.publish_average_position(target_color)
                    
                    # 如果是VTOL且着陆标志已经设置，发布目标位姿
                    if self.aircraft_type == 'standard_vtol':
                        with self.vtol_flag_lock:
                            if self.vtol_land_flag == 1:
                                self.publish_vtol_target_pose(target_color)
                elif point_3d is not None:
                    # 即使不在收集点的状态下，也发布实时位置
                    point_msg = Point(x=point_3d[0], y=point_3d[1], z=point_3d[2])
                    self.position_pubs[target_color].publish(point_msg)
                    
                    # 计算并发布平均位置
                    self.publish_average_position(target_color)
            
            # 根据条件发布点云
            self.conditional_publish_pointclouds()
        
        except Exception as e:
            rospy.logerr(f"处理检测结果时出错: {e}")
            import traceback
            rospy.logerr(f"错误堆栈: {traceback.format_exc()}")
    
    def pixel_to_world_coordinate(self, pixel_x, pixel_y, pose, camera_info):
        """将像素坐标转换为世界坐标（使用TF2系统，假设物体在地面上，Z=0）"""
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
            
            # 检查内参数据是否有效
            if fx == 0 or fy == 0:
                rospy.logwarn_throttle(5.0, "相机内参无效")
                return None
            
            # 将像素坐标转换为相机坐标系的齐次坐标（向右为x轴正方向，向下为y轴正方向）
            x0 = (pixel_x - cx) / fx
            y0 = (pixel_y - cy) / fy
            # 相机坐标系下的齐次坐标
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
    
    def compute_3d_position(self, pixel_x, pixel_y, pose, camera_info):
        """计算像素坐标对应的3D位置（保持兼容性的包装函数）"""
        return self.pixel_to_world_coordinate(pixel_x, pixel_y, pose, camera_info)
    
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
    
    def publish_average_position(self, target_color):
        """计算并发布平均位置"""
        if not self.target_points[target_color]:
            return
        
        # 计算平均位置
        positions = np.array([point['position'] if isinstance(point, dict) else point for point in self.target_points[target_color]])
        avg_position = np.mean(positions, axis=0)
        
        # 创建并发布消息
        point_msg = Point(x=avg_position[0], y=avg_position[1], z=avg_position[2])
        self.pose_estimation_pubs[target_color].publish(point_msg)
    
    def publish_realtime_position(self, target_name, world_pos):
        """发布目标的实时位置"""
        if target_name in self.position_pubs:
            try:
                position_msg = Point()
                position_msg.x = world_pos[0]
                position_msg.y = world_pos[1] 
                position_msg.z = world_pos[2]
                self.position_pubs[target_name].publish(position_msg)
            except Exception as e:
                rospy.logerr(f"发布 {target_name} 实时位置失败: {e}")
        else:
            rospy.logwarn(f"{target_name} 没有对应的实时位置发布者")
    
    def publish_vtol_target_pose(self, target_color):
        """发布VTOL目标位姿"""
        if not self.target_points[target_color] or target_color not in self.vtol_target_pose_pubs:
            return
        
        # 计算平均位置
        positions = np.array([point['position'] if isinstance(point, dict) else point for point in self.target_points[target_color]])
        avg_position = np.mean(positions, axis=0)
        
        # 创建并发布消息
        pose_msg = Pose()
        pose_msg.position.x = avg_position[0]
        pose_msg.position.y = avg_position[1]
        pose_msg.position.z = 0.0  # 目标假定在地面
        
        # 设置默认方向（无旋转）
        pose_msg.orientation.w = 1.0
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        
        # 发布消息
        self.vtol_target_pose_pubs[target_color].publish(pose_msg)
    
    def calculate_and_publish_average_positions(self):
        """计算三个目标的点云平均位置并发布"""
        with self.data_lock:
            for target_name, points in self.target_points.items():
                if points:
                    # 提取所有位置数据
                    positions = np.array([point['position'] if isinstance(point, dict) else point for point in points])
                    
                    # 计算平均位置
                    avg_position = np.mean(positions, axis=0)
                    
                    # 计算标准差（用于评估数据质量）
                    std_position = np.std(positions, axis=0)
                    
                    # 发布平均位置到位姿估计话题
                    if target_name in self.pose_estimation_pubs:
                        avg_msg = Point()
                        avg_msg.x = avg_position[0]
                        avg_msg.y = avg_position[1]
                        avg_msg.z = avg_position[2]
                        
                        self.pose_estimation_pubs[target_name].publish(avg_msg)
    
    def publish_vtol_target_poses(self):
        """发布VTOL目标位姿到指定话题"""
        if self.aircraft_type != 'standard_vtol':
            rospy.logwarn("非VTOL机型，跳过目标位姿发布")
            return
        
        with self.data_lock:
            for target_name, points in self.target_points.items():
                if points and target_name in self.vtol_target_pose_pubs:
                    # 计算平均位置
                    positions = np.array([point['position'] if isinstance(point, dict) else point for point in points])
                    avg_position = np.mean(positions, axis=0)
                    
                    # 创建Pose消息
                    pose_msg = Pose()
                    pose_msg.position.x = avg_position[0]
                    pose_msg.position.y = avg_position[1]
                    pose_msg.position.z = avg_position[2]
                    
                    # 设置默认朝向（单位四元数，表示无旋转）
                    pose_msg.orientation.x = 0.0
                    pose_msg.orientation.y = 0.0
                    pose_msg.orientation.z = 0.0
                    pose_msg.orientation.w = 1.0
                    
                    # 发布到对应话题
                    self.vtol_target_pose_pubs[target_name].publish(pose_msg)
                    
                    # 获取话题名称用于日志
                    topic_mapping = {
                        'red': '/zhihang2025/first_man/pose',
                        'yellow': '/zhihang2025/second_man/pose',
                        'white': '/zhihang2025/third_man/pose'
                    }
                    topic_name = topic_mapping.get(target_name, 'unknown')
                    
                else:
                    if not points:
                        rospy.logwarn(f"{target_name}目标没有点云数据，无法发布位姿")
                    elif target_name not in self.vtol_target_pose_pubs:
                        rospy.logwarn(f"{target_name}目标没有对应的发布者")
    
    def shutdown(self):
        """清理ZMQ连接"""
        self.sender.close()
        self.receiver.close()
        self.context.term()

    def conditional_publish_pointclouds(self):
        """根据条件发布点云数据"""
        if not self.publish_pointcloud:
            return
        
        # 检查是否应该发布点云
        should_publish = self.should_publish_pointcloud()
        
        if should_publish:
            self.publish_pointclouds()
        else:
            with self.vtol_flag_lock:
                vtol_flag = self.vtol_land_flag
    
    def should_publish_pointcloud(self):
        """判断是否应该发布点云数据"""
        # 对于非VTOL机型，始终发布点云
        if self.aircraft_type != 'standard_vtol':
            return True  # 修改：非VTOL机型应该发布点云
        
        # 对于VTOL机型，只有当标志为2时才发布点云
        with self.vtol_flag_lock:
            flag_value = self.vtol_land_flag
            result = flag_value == 2
            return result
    
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
    
    def create_pointcloud2_message(self, points, target_name):
        """创建PointCloud2消息"""
        if not points:
            return None
        
        try:
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
            original_count = len(points)
            if len(points) > self.max_points_per_cloud:
                # 保留最新的点
                points = points[-self.max_points_per_cloud:]

            cloud_data = []
            for i, point in enumerate(points):
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
            
        except Exception as e:
            rospy.logerr(f"创建 {target_name} 点云消息失败: {e}")
            import traceback
            rospy.logerr(f"错误堆栈: {traceback.format_exc()}")
            return None
    
    def pointcloud_timer_callback(self, event):
        """定时发布点云数据"""
        try:
            self.conditional_publish_pointclouds()
        except Exception as e:
            rospy.logerr(f"定时器点云发布失败: {e}")
            import traceback
            rospy.logerr(f"错误堆栈: {traceback.format_exc()}")
    
    def get_target_statistics(self):
        """获取目标统计信息（包括平均位置）"""
        with self.data_lock:
            stats = {}
            for target_name, points in self.target_points.items():
                if points:
                    positions = np.array([point['position'] if isinstance(point, dict) else point for point in points])
                    timestamps = [point['timestamp'] if isinstance(point, dict) else rospy.Time.now().to_sec() for point in points]
                    
                    stats[target_name] = {
                        'count': len(points),
                        'mean_position': np.mean(positions, axis=0).tolist(),
                        'std_position': np.std(positions, axis=0).tolist(),
                        'min_position': np.min(positions, axis=0).tolist(),
                        'max_position': np.max(positions, axis=0).tolist(),
                        'time_span': max(timestamps) - min(timestamps) if timestamps else 0,
                        'latest_timestamp': max(timestamps) if timestamps else None,
                        'oldest_timestamp': min(timestamps) if timestamps else None
                    }
                else:
                    stats[target_name] = {
                        'count': 0,
                        'mean_position': [0.0, 0.0, 0.0],
                        'std_position': [0.0, 0.0, 0.0],
                        'min_position': [0.0, 0.0, 0.0],
                        'max_position': [0.0, 0.0, 0.0],
                        'time_span': 0,
                        'latest_timestamp': None,
                        'oldest_timestamp': None
                    }
            return stats

    def get_target_points_summary(self):
        """获取目标点集摘要信息"""
        with self.data_lock:
            summary = {}
            for target_name, points in self.target_points.items():
                summary[target_name] = {
                    'count': len(points),
                    'latest_position': points[-1]['position'] if points and isinstance(points[-1], dict) else None,
                    'latest_timestamp': points[-1]['timestamp'] if points and isinstance(points[-1], dict) else None
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
        self.conditional_publish_pointclouds()
    
    def get_pointcloud_statistics(self):
        """获取点云统计信息"""
        with self.data_lock:
            stats = {}
            for target_name, points in self.target_points.items():
                if points:
                    positions = np.array([point['position'] if isinstance(point, dict) else point for point in points])
                    timestamps = [point['timestamp'] if isinstance(point, dict) else rospy.Time.now().to_sec() for point in points]
                    
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
                    if isinstance(point, dict) and (current_time - point['timestamp']) <= max_age_seconds
                ]
                removed_count += original_count - len(self.target_points[target_name])
        
        if removed_count > 0:
            rospy.loginfo(f"删除了 {removed_count} 个过期点")
            # 发布更新后的点云
            self.conditional_publish_pointclouds()
    
    def run(self):
        """运行节点"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("节点正在关闭...")
    
    def publish_pixel_position(self, target_name, pixel_x, pixel_y):
        """发布目标的像素坐标位置"""
        if target_name in self.pixel_position_pubs:
            try:
                pixel_msg = Point()
                pixel_msg.x = pixel_x
                pixel_msg.y = pixel_y
                pixel_msg.z = 0.0  # 像素坐标的z值设为0
                self.pixel_position_pubs[target_name].publish(pixel_msg)
            except Exception as e:
                rospy.logerr(f"发布 {target_name} 像素坐标失败: {e}")
        else:
            rospy.logwarn(f"{target_name} 没有对应的像素坐标发布者")

def main():
    """主函数"""
    try:
        node = Yolo12BridgeNode()
        rospy.on_shutdown(node.shutdown)
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"节点运行失败: {str(e)}")

if __name__ == '__main__':
    main()
