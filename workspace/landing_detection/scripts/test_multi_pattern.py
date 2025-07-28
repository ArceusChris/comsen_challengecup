#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class MultiPatternTestClient:
    def __init__(self):
        rospy.init_node('multi_pattern_test_client', anonymous=True)
        
        self.bridge = CvBridge()
        
        # 订阅检测结果
        self.camo_sub = rospy.Subscriber('/landing_target_camo', PointStamped, self.camo_callback)
        self.red_sub = rospy.Subscriber('/landing_target_red', PointStamped, self.red_callback)
        
        # 订阅调试图像
        self.camo_debug_sub = rospy.Subscriber('/landing_debug_camo', Image, self.camo_debug_callback)
        self.red_debug_sub = rospy.Subscriber('/landing_debug_red', Image, self.red_debug_callback)
        
        # 存储最新的检测结果
        self.latest_camo = None
        self.latest_red = None
        
        rospy.loginfo("Multi-pattern test client started")
    
    def camo_callback(self, msg):
        """迷彩图案检测结果回调"""
        self.latest_camo = msg
        rospy.loginfo(f"[CAMOUFLAGE] Detected at ({msg.point.x:.1f}, {msg.point.y:.1f}) with confidence {msg.point.z:.2f}")
    
    def red_callback(self, msg):
        """红色图案检测结果回调"""
        self.latest_red = msg
        rospy.loginfo(f"[RED] Detected at ({msg.point.x:.1f}, {msg.point.y:.1f}) with confidence {msg.point.z:.2f}")
    
    def camo_debug_callback(self, msg):
        """迷彩图案调试图像回调"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Camouflage Pattern Debug", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error displaying camouflage debug image: {e}")
    
    def red_debug_callback(self, msg):
        """红色图案调试图像回调"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Red Pattern Debug", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error displaying red debug image: {e}")
    
    def get_best_detection(self):
        """获取最佳检测结果"""
        candidates = []
        
        if self.latest_camo:
            candidates.append(('camouflage', self.latest_camo))
        
        if self.latest_red:
            candidates.append(('red', self.latest_red))
        
        if not candidates:
            return None, None
        
        # 选择置信度最高的检测结果
        best_type, best_result = max(candidates, key=lambda x: x[1].point.z)
        
        return best_type, best_result
    
    def run(self):
        """运行测试客户端"""
        rate = rospy.Rate(1)  # 1Hz
        
        while not rospy.is_shutdown():
            best_type, best_result = self.get_best_detection()
            
            if best_result:
                rospy.loginfo(f"[BEST] {best_type.upper()} pattern at ({best_result.point.x:.1f}, {best_result.point.y:.1f}) "
                             f"confidence: {best_result.point.z:.2f}")
            else:
                rospy.logwarn("[BEST] No landing platform detected")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        client = MultiPatternTestClient()
        client.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Multi-pattern test client stopped")
    finally:
        cv2.destroyAllWindows()
