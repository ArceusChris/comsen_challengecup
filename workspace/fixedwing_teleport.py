#!/usr/bin/env python
"""
固定翼无人机瞬移控制脚本
支持瞬移并设置适当的初始速度以维持升力
"""
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point, Quaternion, Twist
import math
import sys

def fixedwing_teleport(vehicle_type, vehicle_id, x, y, z, yaw=0.0, forward_speed=15.0):
    """
    瞬移固定翼无人机并设置适当的前进速度
    
    Args:
        vehicle_type: 固定翼类型 (如: standard_vtol, plane, fw)
        vehicle_id: 无人机ID
        x, y, z: 目标位置坐标
        yaw: 偏航角 (弧度)
        forward_speed: 前进速度 (m/s)，固定翼需要保持前进速度
    """
    rospy.init_node('fixedwing_teleport', anonymous=True)
    
    # 创建服务客户端
    set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    try:
        # 等待服务可用
        rospy.wait_for_service('/gazebo/set_model_state', timeout=5.0)
        
        # 创建模型状态消息
        model_state = ModelState()
        model_state.model_name = f"{vehicle_type}_{vehicle_id}"
        
        # 设置位置
        model_state.pose.position.x = x
        model_state.pose.position.y = y
        model_state.pose.position.z = z
        
        # 设置姿态 (偏航角)
        quat_z = math.sin(yaw / 2.0)
        quat_w = math.cos(yaw / 2.0)
        model_state.pose.orientation.x = 0.0
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = quat_z
        model_state.pose.orientation.w = quat_w
        
        # 设置速度 - 固定翼需要前进速度来维持升力
        # 速度方向与偏航角对齐
        model_state.twist.linear.x = forward_speed * math.cos(yaw)
        model_state.twist.linear.y = forward_speed * math.sin(yaw)
        model_state.twist.linear.z = 0.0
        model_state.twist.angular.x = 0.0
        model_state.twist.angular.y = 0.0
        model_state.twist.angular.z = 0.0
        
        model_state.reference_frame = 'world'
        
        # 执行瞬移
        print(f"Teleporting {vehicle_type}_{vehicle_id} to ({x}, {y}, {z}) with speed {forward_speed} m/s...")
        response = set_model_state(model_state)
        
        if response.success:
            print(f"✅ Successfully teleported {vehicle_type}_{vehicle_id}")
            print(f"   Position: ({x:.2f}, {y:.2f}, {z:.2f})")
            print(f"   Yaw: {math.degrees(yaw):.1f}°")
            print(f"   Forward speed: {forward_speed} m/s")
        else:
            print(f"❌ Teleport failed: {response.status_message}")
            
    except rospy.ROSException as e:
        print(f"❌ ROS Error: {e}")
        print("Make sure Gazebo simulation is running!")
    except Exception as e:
        print(f"❌ Error: {e}")

def show_usage():
    """显示使用方法"""
    print("固定翼无人机瞬移控制脚本")
    print("=" * 40)
    print("Usage: python fixedwing_teleport.py <vehicle_type> <vehicle_id> <x> <y> <z> [yaw] [speed]")
    print()
    print("参数说明:")
    print("  vehicle_type  - 固定翼类型 (如: standard_vtol, plane, fw)")
    print("  vehicle_id    - 无人机ID (通常为0)")
    print("  x, y, z       - 目标位置坐标 (米)")
    print("  yaw           - 偏航角 (度，可选，默认0)")
    print("  speed         - 前进速度 (m/s，可选，默认15)")
    print()
    print("示例:")
    print("  python fixedwing_teleport.py standard_vtol 0 1000 500 100")
    print("  python fixedwing_teleport.py plane 0 2000 1000 150 45 20")
    print("  python fixedwing_teleport.py fw 0 500 -200 80 -90 12")
    print()
    print("常用固定翼类型:")
    print("  - standard_vtol: 垂直起降固定翼")
    print("  - plane: 标准固定翼")
    print("  - fw: 固定翼简称")

if __name__ == '__main__':
    if len(sys.argv) < 6:
        show_usage()
        sys.exit(1)
    
    try:
        vehicle_type = sys.argv[1]
        vehicle_id = sys.argv[2]
        x = float(sys.argv[3])
        y = float(sys.argv[4])
        z = float(sys.argv[5])
        
        # 偏航角 (度转弧度)
        yaw_deg = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
        yaw = math.radians(yaw_deg)
        
        # 前进速度
        forward_speed = float(sys.argv[7]) if len(sys.argv) > 7 else 15.0
        
        fixedwing_teleport(vehicle_type, vehicle_id, x, y, z, yaw, forward_speed)
        
    except ValueError as e:
        print(f"❌ 参数错误: {e}")
        print("请确保坐标、角度和速度都是数字!")
        show_usage()
    except KeyboardInterrupt:
        print("\n操作已取消")
