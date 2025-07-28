#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
启动YOLO11推理节点的示例脚本
使用自定义参数启动节点
"""

import rospy
import sys
import os

def launch_yolo_node():
    """启动YOLO11推理节点"""
    
    # 设置基本参数
    rospy.set_param('/yolo11_inference_node/input_topic', '/iris_0/camera/image_raw')
    rospy.set_param('/yolo11_inference_node/output_topic', '/yolo11/detection_image')
    rospy.set_param('/yolo11_inference_node/pose_topic', '/iris_0/mavros/local_position/pose')
    rospy.set_param('/yolo11_inference_node/camera_info_topic', '/iris_0/camera/camera_info')
    rospy.set_param('/yolo11_inference_node/model_path', 'yolo11n.pt')
    rospy.set_param('/yolo11_inference_node/confidence_threshold', 0.5)
    rospy.set_param('/yolo11_inference_node/iou_threshold', 0.45)
    
    # 设置相机变换参数（相机朝向正下方配置）
    camera_rotation = [
        [1, 0, 0],   # 相机X轴 -> 无人机X轴（前向）
        [0, 1, 0],   # 相机Y轴 -> 无人机Y轴（右向）
        [0, 0, -1]   # 相机Z轴 -> 无人机-Z轴（向下）
    ]
    camera_translation = [0.0, 0.0, -0.03]  # 相机位于无人机正下方0.03m处
    
    rospy.set_param('/yolo11_inference_node/camera_rotation_matrix', camera_rotation)
    rospy.set_param('/yolo11_inference_node/camera_translation', camera_translation)
    
    # 设置点云发布参数
    rospy.set_param('/yolo11_inference_node/publish_pointcloud', True)
    rospy.set_param('/yolo11_inference_node/pointcloud_frame_id', 'map')
    rospy.set_param('/yolo11_inference_node/max_points_per_cloud', 1000)
    
    print("YOLO11推理节点参数已设置")
    print("输入话题: /iris_0/camera/image_raw")
    print("输出话题: /yolo11/detection_image")
    print("位姿话题: /iris_0/mavros/local_position/pose")
    print("相机内参话题: /iris_0/camera/camera_info")
    print("模型路径: yolo11n.pt")
    print("置信度阈值: 0.5")
    print("IOU阈值: 0.45")
    print("\n相机变换参数（朝向正下方）:")
    print("旋转矩阵:", camera_rotation)
    print("平移向量:", camera_translation)
    print("相机位置：无人机正下方0.03m处")
    print("\n点云发布设置:")
    print("发布点云: True")
    print("点云坐标系: map")
    print("每个点云最大点数: 1000")
    print("点云话题:")
    print("  /yolo11/pointcloud/red")
    print("  /yolo11/pointcloud/yellow") 
    print("  /yolo11/pointcloud/white")
    print("\n如需自定义其他配置，可使用:")
    print("python3 camera_transform_config.py")
    print("\n请在另一个终端中运行:")
    print("cd /root/workspace/perception")
    print("python3 yolo11_inference.py")

if __name__ == '__main__':
    launch_yolo_node()
