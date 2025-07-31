#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ROS1 Python脚本：从指定话题订阅图像并每隔1秒保存一张图片
使用方法：
    python3 image_saver.py --topic /iris_0/camera/image_raw --output_dir ./dataset
"""

import rospy
import cv2
import os
import argparse
import threading
from datetime import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageSaver:
    def __init__(self, topic_name, output_dir, save_interval=1.0):
        """
        初始化图像保存器
        
        Args:
            topic_name (str): 图像话题名称
            output_dir (str): 图片保存目录
            save_interval (float): 保存间隔（秒）
        """
        self.topic_name = topic_name
        self.output_dir = output_dir
        self.save_interval = save_interval
        
        # 创建输出目录
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            rospy.loginfo(f"Created output directory: {self.output_dir}")
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 图像缓存
        self.latest_image = None
        self.image_lock = threading.Lock()
        
        # 初始化ROS节点
        rospy.init_node('image_saver_node', anonymous=True)
        
        # 订阅图像话题
        self.image_subscriber = rospy.Subscriber(
            self.topic_name, 
            Image, 
            self.image_callback,
            queue_size=1
        )
        
        rospy.loginfo(f"Subscribing to topic: {self.topic_name}")
        rospy.loginfo(f"Saving images to: {self.output_dir}")
        rospy.loginfo(f"Save interval: {self.save_interval} seconds")
        
        # 启动保存线程
        self.save_thread = threading.Thread(target=self.save_images_periodically)
        self.save_thread.daemon = True
        self.save_thread.start()
    
    def image_callback(self, msg):
        """
        图像话题回调函数
        
        Args:
            msg (sensor_msgs.msg.Image): 接收到的图像消息
        """
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 线程安全地更新最新图像
            with self.image_lock:
                self.latest_image = cv_image
                
        except Exception as e:
            rospy.logerr(f"Error converting image: {e}")
    
    def save_images_periodically(self):
        """
        定期保存图像的线程函数
        """
        image_count = 0
        
        while not rospy.is_shutdown():
            try:
                # 等待指定的时间间隔
                rospy.sleep(self.save_interval)
                
                # 获取当前最新图像
                with self.image_lock:
                    if self.latest_image is not None:
                        current_image = self.latest_image.copy()
                    else:
                        rospy.logwarn("No image received yet")
                        continue
                
                # 生成文件名（使用时间戳）
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # 精确到毫秒
                filename = f"image_{image_count:06d}_{timestamp}.jpg"
                filepath = os.path.join(self.output_dir, filename)
                
                # 保存图像
                success = cv2.imwrite(filepath, current_image)
                
                if success:
                    image_count += 1
                    rospy.loginfo(f"Saved image {image_count}: {filename}")
                else:
                    rospy.logerr(f"Failed to save image: {filename}")
                    
            except Exception as e:
                rospy.logerr(f"Error in save thread: {e}")
    
    def run(self):
        """
        运行节点
        """
        try:
            rospy.loginfo("Image saver node is running. Press Ctrl+C to stop.")
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down image saver node...")
        except Exception as e:
            rospy.logerr(f"Error in main loop: {e}")


def main():
    """
    主函数
    """
    parser = argparse.ArgumentParser(description='ROS1 Image Saver')
    parser.add_argument(
        '--topic', 
        type=str, 
        default='/camera/image_raw',
        help='Image topic name (default: /camera/image_raw)'
    )
    parser.add_argument(
        '--output_dir', 
        type=str, 
        default='./saved_images',
        help='Output directory for saved images (default: ./saved_images)'
    )
    parser.add_argument(
        '--interval', 
        type=float, 
        default=1.0,
        help='Save interval in seconds (default: 1.0)'
    )
    
    args = parser.parse_args()
    
    try:
        # 创建并运行图像保存器
        image_saver = ImageSaver(args.topic, args.output_dir, args.interval)
        image_saver.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted")
    except Exception as e:
        rospy.logerr(f"Error: {e}")


if __name__ == '__main__':
    main()
