#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class LandingTestClient:
    def __init__(self):
        rospy.init_node('landing_test_client', anonymous=True)
        
        self.bridge = CvBridge()
        
        # 订阅检测结果
        self.target_sub = rospy.Subscriber('/landing_target', PointStamped, self.target_callback)
        self.center_sub = rospy.Subscriber('/landing_platform/center', Point, self.center_callback)
        
        # 订阅调试图像
        self.debug_sub = rospy.Subscriber('/landing_platform/debug_image', Image, self.debug_callback)
        self.simple_debug_sub = rospy.Subscriber('/landing_debug', Image, self.simple_debug_callback)
        
        # 存储最新的检测结果
        self.latest_target = None
        self.latest_center = None
        
        rospy.loginfo("Landing test client started")
    
    def target_callback(self, msg):
        """
        简化版检测器结果回调
        """
        self.latest_target = msg
        rospy.loginfo(f"Simple detector - Target at: ({msg.point.x:.1f}, {msg.point.y:.1f}), "
                     f"Confidence: {msg.point.z:.2f}")
    
    def center_callback(self, msg):
        """
        详细版检测器结果回调
        """
        self.latest_center = msg
        rospy.loginfo(f"Detailed detector - Center at: ({msg.x:.1f}, {msg.y:.1f}), "
                     f"Confidence: {msg.z:.2f}")
    
    def debug_callback(self, msg):
        """
        详细版调试图像回调
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Detailed Detection Debug", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error displaying detailed debug image: {e}")
    
    def simple_debug_callback(self, msg):
        """
        简化版调试图像回调
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Simple Detection Debug", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error displaying simple debug image: {e}")
    
    def print_status(self):
        """
        打印当前检测状态
        """
        rospy.loginfo("=== Landing Detection Status ===")
        
        if self.latest_target:
            rospy.loginfo(f"Simple Detector: ({self.latest_target.point.x:.1f}, "
                         f"{self.latest_target.point.y:.1f}) - "
                         f"Conf: {self.latest_target.point.z:.2f}")
        else:
            rospy.loginfo("Simple Detector: No target detected")
        
        if self.latest_center:
            rospy.loginfo(f"Detailed Detector: ({self.latest_center.x:.1f}, "
                         f"{self.latest_center.y:.1f}) - "
                         f"Conf: {self.latest_center.z:.2f}")
        else:
            rospy.loginfo("Detailed Detector: No target detected")
        
        rospy.loginfo("=================================")
    
    def run(self):
        """
        运行测试客户端
        """
        rate = rospy.Rate(1)  # 1Hz
        
        while not rospy.is_shutdown():
            self.print_status()
            rate.sleep()

if __name__ == '__main__':
    try:
        client = LandingTestClient()
        client.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Landing test client stopped")
    finally:
        cv2.destroyAllWindows()
