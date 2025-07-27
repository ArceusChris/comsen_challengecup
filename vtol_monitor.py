#!/usr/bin/env python3
"""
简单的ROS话题监听器，用于检查VTOL状态
"""

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

def position_callback(msg):
    pos = msg.pose.position
    print(f"位置: x={pos.x:.1f}, y={pos.y:.1f}, z={pos.z:.1f}")

def state_callback(msg):
    print(f"状态: 连接={msg.connected}, 解锁={msg.armed}, 模式={msg.mode}")

def main():
    rospy.init_node('vtol_monitor', anonymous=True)
    
    vtol_ns = "/standard_vtol_0"
    
    # 订阅话题
    pos_sub = rospy.Subscriber(f'{vtol_ns}/mavros/local_position/pose', PoseStamped, position_callback)
    state_sub = rospy.Subscriber(f'{vtol_ns}/mavros/state', State, state_callback)
    
    print("监听VTOL状态...")
    print("按Ctrl+C退出")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("退出监听")

if __name__ == "__main__":
    main()
