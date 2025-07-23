#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
检测方法切换脚本
用于在YOLO检测和颜色阈值检测之间切换
"""

import rospy
import sys
from std_msgs.msg import String

def switch_detection_method(method):
    """切换检测方法"""
    if method not in ['yolo', 'color']:
        print(f"错误：不支持的检测方法 '{method}'，请使用 'yolo' 或 'color'")
        return False
    
    try:
        # 设置参数
        rospy.set_param('/yolo11_inference_node/detection_method', method)
        print(f"检测方法已切换为: {method}")
        
        # 发布切换信息（可选）
        pub = rospy.Publisher('/yolo11/detection_method_switch', String, queue_size=1)
        rospy.sleep(0.5)  # 等待发布者建立连接
        pub.publish(String(data=method))
        
        return True
    except Exception as e:
        print(f"切换失败: {str(e)}")
        return False

def main():
    """主函数"""
    rospy.init_node('detection_method_switch', anonymous=True)
    
    if len(sys.argv) != 2:
        print("使用方法: python3 switch_detection_method.py [yolo|color]")
        print("示例:")
        print("  python3 switch_detection_method.py yolo   # 切换到YOLO检测")
        print("  python3 switch_detection_method.py color  # 切换到颜色检测")
        sys.exit(1)
    
    method = sys.argv[1].lower()
    
    if switch_detection_method(method):
        print("切换成功！")
        print("注意：需要重启yolo11_inference_node节点以使更改生效")
    else:
        sys.exit(1)

if __name__ == '__main__':
    main()
