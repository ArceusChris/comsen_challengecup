#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import os

class YOLO11InferenceNode:
    def __init__(self):
        """初始化YOLO11推理节点"""
        rospy.init_node('yolo11_inference_node', anonymous=True)
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 获取参数
        self.input_topic = rospy.get_param('~input_topic', '/camera/image_raw')
        self.output_topic = rospy.get_param('~output_topic', '/yolo11/detection_image')
        self.model_path = rospy.get_param('~model_path', 'yolo11n.pt')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.iou_threshold = rospy.get_param('~iou_threshold', 0.45)
        
        # 加载YOLO11模型
        self.load_model()
        
        # 创建发布者和订阅者
        self.image_pub = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.image_sub = rospy.Subscriber(self.input_topic, Image, self.image_callback)
        
        rospy.loginfo(f"YOLO11推理节点已启动")
        rospy.loginfo(f"订阅话题: {self.input_topic}")
        rospy.loginfo(f"发布话题: {self.output_topic}")
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
    
    def run(self):
        """运行节点"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("节点正在关闭...")

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