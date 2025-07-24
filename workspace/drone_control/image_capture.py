#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
图像采集脚本
从/iris_0/camera/image_raw话题获取图像，以6fps频率保存到dataset文件夹

用法:
    python image_capture.py

功能:
    - 订阅/iris_0/camera/image_raw话题
    - 以6fps频率保存图像到dataset文件夹
    - 图像文件名格式: image_YYYYMMDD_HHMMSS_NNNN.jpg
    - 支持Ctrl+C优雅退出
"""

import rospy
import cv2
import os
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from datetime import datetime

class ImageCapture:
    def __init__(self, output_dir="dataset", capture_fps=6.0):
        """
        初始化图像采集器
        
        Args:
            output_dir (str): 输出目录
            capture_fps (float): 采集帧率
        """
        self.output_dir = output_dir
        self.capture_fps = capture_fps
        self.capture_interval = 1.0 / capture_fps  # 采集间隔(秒)
        
        # 创建输出目录
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            print(f"Created directory: {self.output_dir}")
        
        # CV Bridge用于ROS图像转换
        self.bridge = CvBridge()
        
        # 最新接收的图像
        self.latest_image = None
        self.image_lock = threading.Lock()
        
        # 图像计数器
        self.image_count = 0
        
        # 上次保存时间
        self.last_save_time = 0.0
        
        # 初始化ROS节点
        rospy.init_node('image_capture_node', anonymous=True)
        
        # 订阅相机话题
        self.image_sub = rospy.Subscriber('/iris_0/camera/image_raw', Image, self.image_callback)
        
        print(f"Image capture initialized:")
        print(f"  - Topic: /iris_0/camera/image_raw")
        print(f"  - Output directory: {self.output_dir}")
        print(f"  - Capture rate: {capture_fps} fps")
        print(f"  - Capture interval: {self.capture_interval:.3f} seconds")
        print("Waiting for images...")
        
    def image_callback(self, msg):
        """
        图像话题回调函数
        
        Args:
            msg (sensor_msgs/Image): 图像消息
        """
        try:
            # 转换ROS图像到OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 线程安全地更新最新图像
            with self.image_lock:
                self.latest_image = cv_image.copy()
                
            # 检查是否需要保存图像
            current_time = rospy.get_time()
            if current_time - self.last_save_time >= self.capture_interval:
                self.save_image(cv_image)
                self.last_save_time = current_time
                
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def save_image(self, cv_image):
        """
        保存图像到磁盘
        
        Args:
            cv_image: OpenCV图像
        """
        try:
            # 生成文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"image_{timestamp}_{self.image_count:04d}.jpg"
            filepath = os.path.join(self.output_dir, filename)
            
            # 保存图像
            success = cv2.imwrite(filepath, cv_image)
            
            if success:
                self.image_count += 1
                print(f"Saved: {filename} (Count: {self.image_count})")
            else:
                rospy.logwarn(f"Failed to save image: {filename}")
                
        except Exception as e:
            rospy.logerr(f"Error saving image: {e}")
    
    def get_stats(self):
        """获取统计信息"""
        return {
            'images_saved': self.image_count,
            'output_dir': self.output_dir,
            'capture_fps': self.capture_fps,
            'has_image': self.latest_image is not None
        }
    
    def run(self):
        """运行图像采集"""
        try:
            # 等待话题连接
            rospy.loginfo("Waiting for camera topic...")
            rospy.wait_for_message('/iris_0/camera/image_raw', Image, timeout=10.0)
            rospy.loginfo("Camera topic connected!")
            
            # 保持节点运行
            rospy.spin()
            
        except rospy.ROSInterruptException:
            print("\nReceived interrupt signal")
        except Exception as e:
            rospy.logerr(f"Error in image capture: {e}")
        finally:
            stats = self.get_stats()
            print(f"\nImage capture stopped.")
            print(f"Total images saved: {stats['images_saved']}")
            print(f"Images saved to: {stats['output_dir']}")

def main():
    """主函数"""
    try:
        # 创建图像采集器
        capture = ImageCapture(output_dir="dataset", capture_fps=6.0)
        
        # 运行采集
        capture.run()
        
    except KeyboardInterrupt:
        print("\nStopping image capture...")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    main()
