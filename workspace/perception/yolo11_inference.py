#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import os
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_matrix
import threading

class YOLO11InferenceNode:
    def __init__(self):
        """初始化YOLO11推理节点"""
        rospy.init_node('yolo11_inference_node', anonymous=True)
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 获取参数
        self.input_topic = rospy.get_param('~input_topic', '/camera/image_raw')
        self.output_topic = rospy.get_param('~output_topic', '/yolo11/detection_image')
        self.pose_topic = rospy.get_param('~pose_topic', '/iris_0/mavros/local_position/pose')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/iris_0/camera/camera_info')
        self.model_path = rospy.get_param('~model_path', 'yolo11n.pt')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.45)
        
        # 初始化数据存储
        self.current_pose = None
        self.camera_info = None
        self.data_lock = threading.Lock()
        
        # 创建三个目标的点集
        self.target_points = {
            'red': [],
            'yellow': [],
            'white': []
        }
        
        # 相机变换参数
        self.camera_transform_params = self.get_camera_transform_params()
        
        # 加载YOLO11模型
        self.load_model()
        
        # 创建发布者和订阅者
        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.image_sub = rospy.Subscriber(self.input_topic, Image, self.image_callback)
        self.pose_sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_callback)
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        
        rospy.loginfo(f"YOLO11推理节点已启动")
        rospy.loginfo(f"订阅话题: {self.input_topic}")
        rospy.loginfo(f"发布话题: {self.output_topic}")
        rospy.loginfo(f"位姿话题: {self.pose_topic}")
        rospy.loginfo(f"相机内参话题: {self.camera_info_topic}")
        rospy.loginfo(f"模型路径: {self.model_path}")
        rospy.loginfo(f"置信度阈值: {self.confidence_threshold}")
        rospy.loginfo(f"IOU阈值: {self.iou_threshold}")
        
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
            
            # 运行YOLO11推理
            results = self.model(cv_image, 
                               conf=self.confidence_threshold, 
                               iou=self.iou_threshold)
            
            # 获取当前位姿和相机内参
            with self.data_lock:
                current_pose = self.current_pose
                current_camera_info = self.camera_info
            
            # 处理检测结果并更新点集
            self.process_detections_and_update_points(results[0], current_pose, current_camera_info)
            
            # 绘制检测结果
            annotated_image = self.draw_detections(cv_image, results[0])
            
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
            count_text = f"检测到 {detection_count} 个目标"
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
        
        # 获取相机内参
        fx = camera_info.K[0]  # 焦距x
        fy = camera_info.K[4]  # 焦距y
        cx = camera_info.K[2]  # 光心x
        cy = camera_info.K[5]  # 光心y
        
        # 将像素坐标转换为归一化相机坐标
        x_norm = (pixel_x - cx) / fx
        y_norm = (pixel_y - cy) / fy
        
        # 第一步：从像素坐标到相机坐标系
        # 相机坐标系下的射线方向（假设相机朝向+Z方向，符合ROS标准）
        ray_camera = np.array([x_norm, y_norm, 1.0])
        ray_camera = ray_camera / np.linalg.norm(ray_camera)
        
        # 第二步：从相机坐标系到无人机坐标系（body frame）
        # 使用配置的相机变换参数
        R_camera_to_body = self.camera_transform_params['rotation']
        t_camera_to_body = self.camera_transform_params['translation']
        
        # 将相机射线转换到无人机坐标系
        ray_body = R_camera_to_body @ ray_camera
        
        # 第三步：从无人机坐标系到世界坐标系
        # 获取无人机位姿信息
        drone_pos = pose.pose.position
        drone_orientation = pose.pose.orientation
        
        # 将四元数转换为旋转矩阵（无人机到世界坐标系的变换）
        quaternion = [drone_orientation.x, drone_orientation.y, 
                     drone_orientation.z, drone_orientation.w]
        R_body_to_world = quaternion_matrix(quaternion)[:3, :3]
        
        # 无人机在世界坐标系中的位置
        drone_pos_world = np.array([drone_pos.x, drone_pos.y, drone_pos.z])
        
        # 计算相机在世界坐标系中的位置
        camera_pos_world = drone_pos_world + R_body_to_world @ t_camera_to_body
        
        # 将射线转换到世界坐标系
        ray_world = R_body_to_world @ ray_body
        
        # 第四步：计算射线与地面的交点（假设地面Z=0）
        # 射线方程: P = camera_pos_world + t * ray_world
        # 地面方程: Z = 0
        # 求解: camera_pos_world[2] + t * ray_world[2] = 0
        if abs(ray_world[2]) < 1e-6:  # 射线平行于地面
            return None
        
        t = -camera_pos_world[2] / ray_world[2]
        if t < 0:  # 交点在相机后方
            return None
        
        # 计算世界坐标
        world_pos = camera_pos_world + t * ray_world
        world_pos[2] = 0.0  # 确保Z坐标为0（地面高度）
        
        return world_pos
    
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
            class_name = self.class_names[class_id] if class_id < len(self.class_names) else f"Class_{class_id}"
            
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