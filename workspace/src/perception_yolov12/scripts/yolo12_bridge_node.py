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
    
    def get_camera_transform_params(self):
        """获取相机变换参数"""
        # 以下代码从原始YOLO11代码中移植
        return {
            'iris': {
                'translation': [0.0, 0.0, 0.0],
                'rotation': [1.0, 0.0, 0.0, 0.0]  # 四元数 [w, x, y, z]
            },
            'standard_vtol': {
                'translation': [0.0, 0.0, 0.0],
                'rotation': [1.0, 0.0, 0.0, 0.0]  # 四元数 [w, x, y, z]
            }
        }.get(self.aircraft_type, {
            'translation': [0.0, 0.0, 0.0],
            'rotation': [1.0, 0.0, 0.0, 0.0]
        })
    
    def pose_callback(self, msg):
        """位姿回调函数，存储当前无人机位置"""
        with self.data_lock:
            self.current_pose = msg

    def camera_info_callback(self, msg):
        """相机内参回调函数"""
        with self.data_lock:
            self.camera_info = msg
    
    def vtol_flag_callback(self, msg):
        """VTOL着陆标志回调"""
        with self.vtol_flag_lock:
            self.vtol_land_flag = msg.data
            rospy.loginfo(f"接收到VTOL着陆标志: {self.vtol_land_flag}")
    
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
        """处理检测结果，更新位置信信息"""
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
                
                # 计算3D位置
                point_3d = self.compute_3d_position(center_x, center_y, current_pose, current_camera_info)
                
                if point_3d is not None:
                    # 更新点集
                    if len(self.target_points[target_color]) >= self.max_points_per_cloud:
                        self.target_points[target_color].pop(0)  # 如果达到最大点数，则移除最旧的点
                    self.target_points[target_color].append(point_3d)
                    
                    # 发布实时位置
                    point_msg = Point(x=point_3d[0], y=point_3d[1], z=point_3d[2])
                    self.position_pubs[target_color].publish(point_msg)
                    
                    # 发布像素坐标
                    pixel_msg = Point(x=center_x, y=center_y, z=0.0)
                    self.pixel_position_pubs[target_color].publish(pixel_msg)
                    
                    # 计算并发布平均位置
                    self.publish_average_position(target_color)
                    
                    # 如果是VTOL且着陆标志已经设置，发布目标位姿
                    if self.aircraft_type == 'standard_vtol':
                        with self.vtol_flag_lock:
                            if self.vtol_land_flag == 1:
                                self.publish_vtol_target_pose(target_color)
        
        except Exception as e:
            rospy.logerr(f"处理检测结果时出错: {e}")
            import traceback
            rospy.logerr(f"错误堆栈: {traceback.format_exc()}")
    
    def compute_3d_position(self, pixel_x, pixel_y, pose, camera_info):
        """计算像素坐标对应的3D位置"""
        try:
            # 检查内参数据是否有效
            if camera_info.K[0] == 0:
                rospy.logwarn_throttle(5.0, "相机内参无效")
                return None
            
            # 获取相机内参
            fx = camera_info.K[0]  # 焦距 x
            fy = camera_info.K[4]  # 焦距 y
            cx = camera_info.K[2]  # 主点 x
            cy = camera_info.K[5]  # 主点 y
            
            # 假设目标在地面上
            # 从相机坐标系到无人机坐标系
            camera_transform = self.camera_transform_params
            
            # 计算地面的距离
            altitude = pose.pose.position.z
            
            # 计算归一化的像平面坐标
            x_norm = (pixel_x - cx) / fx
            y_norm = (pixel_y - cy) / fy
            
            # 计算从相机到目标的向量
            # 假设z轴指向相机前方，y轴向下
            vec_camera = np.array([x_norm, y_norm, 1.0])
            
            # 获取无人机的方向四元数
            q = [
                pose.pose.orientation.w,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z
            ]
            
            # 计算旋转矩阵
            rotation_matrix = quaternion_matrix(q)[:3, :3]
            
            # 将相机坐标系中的向量旋转到世界坐标系
            vec_world = rotation_matrix.dot(vec_camera)
            
            # 归一化向量
            vec_world = vec_world / np.linalg.norm(vec_world)
            
            # 计算射线与地面的交点
            # 假设地面是z=0平面
            if vec_world[2] >= 0:
                rospy.logdebug_throttle(5.0, "目标在相机上方，无法计算地面交点")
                return None
            
            # 计算缩放因子
            scale = -altitude / vec_world[2]
            
            # 计算地面交点
            ground_point = np.array([
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z
            ]) + scale * vec_world
            
            return ground_point
            
        except Exception as e:
            rospy.logerr(f"计算3D位置时出错: {e}")
            import traceback
            rospy.logerr(f"错误堆栈: {traceback.format_exc()}")
            return None
    
    def publish_average_position(self, target_color):
        """计算并发布平均位置"""
        if not self.target_points[target_color]:
            return
        
        # 计算平均位置
        points = np.array(self.target_points[target_color])
        avg_position = np.mean(points, axis=0)
        
        # 创建并发布消息
        point_msg = Point(x=avg_position[0], y=avg_position[1], z=avg_position[2])
        self.pose_estimation_pubs[target_color].publish(point_msg)
    
    def publish_vtol_target_pose(self, target_color):
        """发布VTOL目标位姿"""
        if not self.target_points[target_color] or target_color not in self.vtol_target_pose_pubs:
            return
        
        # 计算平均位置
        points = np.array(self.target_points[target_color])
        avg_position = np.mean(points, axis=0)
        
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
    
    def shutdown(self):
        """清理ZMQ连接"""
        self.sender.close()
        self.receiver.close()
        self.context.term()

if __name__ == '__main__':
    try:
        node = Yolo12BridgeNode()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
