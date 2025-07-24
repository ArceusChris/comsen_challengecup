#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point, Quaternion
import math
import sys

def gazebo_teleport_drone(vehicle_type, vehicle_id, x, y, z, yaw=0.0):
    """在Gazebo中瞬移无人机"""
    rospy.init_node('gazebo_teleport', anonymous=True)
    
    # 创建服务客户端
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    try:
        # 创建模型状态消息
        model_state = ModelState()
        model_state.model_name = f"{vehicle_type}_{vehicle_id}"
        
        # 设置位置
        model_state.pose.position.x = x
        model_state.pose.position.y = y
        model_state.pose.position.z = z
        
        # 设置姿态
        quat_z = math.sin(yaw / 2.0)
        quat_w = math.cos(yaw / 2.0)
        model_state.pose.orientation.x = 0.0
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = quat_z
        model_state.pose.orientation.w = quat_w
        
        # 清零速度
        model_state.twist.linear.x = 0.0
        model_state.twist.linear.y = 0.0
        model_state.twist.linear.z = 0.0
        model_state.twist.angular.x = 0.0
        model_state.twist.angular.y = 0.0
        model_state.twist.angular.z = 0.0
        
        model_state.reference_frame = 'world'
        
        # 执行瞬移
        response = set_model_state(model_state)
        if response.success:
            print(f"Successfully teleported {vehicle_type}_{vehicle_id} to ({x}, {y}, {z})")
        else:
            print(f"Teleport failed: {response.status_message}")
            
    except Exception as e:
        print(f"Error: {e}")

if __name__ == '__main__':
    if len(sys.argv) < 6:
        print("Usage: python gazebo_teleport.py <vehicle_type> <vehicle_id> <x> <y> <z> [yaw]")
        print("Example: python gazebo_teleport.py iris 0 1492.49 31.83 3.2 0.0")
        sys.exit(1)
    
    vehicle_type = sys.argv[1]
    vehicle_id = sys.argv[2]
    x = float(sys.argv[3])
    y = float(sys.argv[4])
    z = float(sys.argv[5])
    yaw = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
    
    gazebo_teleport_drone(vehicle_type, vehicle_id, x, y, z, yaw)