#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
无人机控制测试脚本
演示如何使用drone_control.py脚本控制无人机
"""

import rospy
from std_msgs.msg import String, Float32MultiArray
import time
import sys

def test_drone_control():
    """测试无人机控制功能"""
    
    rospy.init_node('drone_control_test')
    
    # 发布器
    vel_cmd_pub = rospy.Publisher('/vel_cmd', Float32MultiArray, queue_size=1)
    drone_cmd_pub = rospy.Publisher('/drone_cmd', String, queue_size=1)
    
    print("无人机控制测试脚本")
    print("确保已经启动了 drone_control.py")
    print("=" * 40)
    
    # 等待发布器连接
    rospy.sleep(2.0)
    
    try:
        print("测试1: 自动起飞流程")
        cmd_msg = String()
        cmd_msg.data = "AUTO_TAKEOFF"
        drone_cmd_pub.publish(cmd_msg)
        print("发送自动起飞命令...")
        rospy.sleep(10.0)  # 等待起飞完成
        
        print("\n测试2: 发送速度命令 - 前进")
        vel_msg = Float32MultiArray()
        vel_msg.data = [1.0, 0.0, 0.0, 0.0]  # 前进1m/s
        vel_cmd_pub.publish(vel_msg)
        print("发送前进命令: [1.0, 0.0, 0.0, 0.0]")
        rospy.sleep(3.0)
        
        print("\n测试3: 发送速度命令 - 左移")
        vel_msg.data = [0.0, 1.0, 0.0, 0.0]  # 左移1m/s
        vel_cmd_pub.publish(vel_msg)
        print("发送左移命令: [0.0, 1.0, 0.0, 0.0]")
        rospy.sleep(3.0)
        
        print("\n测试4: 发送速度命令 - 旋转")
        vel_msg.data = [0.0, 0.0, 0.0, 0.5]  # 旋转0.5rad/s
        vel_cmd_pub.publish(vel_msg)
        print("发送旋转命令: [0.0, 0.0, 0.0, 0.5]")
        rospy.sleep(3.0)
        
        print("\n测试5: 悬停")
        cmd_msg.data = "HOVER"
        drone_cmd_pub.publish(cmd_msg)
        print("发送悬停命令...")
        rospy.sleep(3.0)
        
        print("\n测试6: 自动降落")
        cmd_msg.data = "AUTO_LAND"
        drone_cmd_pub.publish(cmd_msg)
        print("发送自动降落命令...")
        rospy.sleep(15.0)
        
        print("\n测试完成！")
        
    except KeyboardInterrupt:
        print("\n测试被用户中断")
        # 紧急停止
        cmd_msg = String()
        cmd_msg.data = "EMERGENCY"
        drone_cmd_pub.publish(cmd_msg)
        print("发送紧急停止命令")

def manual_control():
    """手动控制模式"""
    rospy.init_node('drone_manual_control')
    
    vel_cmd_pub = rospy.Publisher('/vel_cmd', Float32MultiArray, queue_size=1)
    drone_cmd_pub = rospy.Publisher('/drone_cmd', String, queue_size=1)
    
    print("手动控制模式")
    print("=" * 40)
    print("可用命令:")
    print("1 - 自动起飞")
    print("2 - 前进")
    print("3 - 后退")
    print("4 - 左移")
    print("5 - 右移")
    print("6 - 上升")
    print("7 - 下降")
    print("8 - 左转")
    print("9 - 右转")
    print("0 - 悬停")
    print("h - 返航")
    print("l - 自动降落")
    print("e - 紧急停止")
    print("q - 退出")
    print("=" * 40)
    
    rospy.sleep(1.0)
    
    try:
        while not rospy.is_shutdown():
            cmd = input("请输入命令: ").strip().lower()
            
            vel_msg = Float32MultiArray()
            cmd_msg = String()
            
            if cmd == '1':
                cmd_msg.data = "AUTO_TAKEOFF"
                drone_cmd_pub.publish(cmd_msg)
                print("执行自动起飞...")
            elif cmd == '2':
                vel_msg.data = [10.0, 0.0, 0.0, 0.0]
                vel_cmd_pub.publish(vel_msg)
                print("前进")
            elif cmd == '3':
                vel_msg.data = [-1.0, 0.0, 0.0, 0.0]
                vel_cmd_pub.publish(vel_msg)
                print("后退")
            elif cmd == '4':
                vel_msg.data = [0.0, 1.0, 0.0, 0.0]
                vel_cmd_pub.publish(vel_msg)
                print("左移")
            elif cmd == '5':
                vel_msg.data = [0.0, -1.0, 0.0, 0.0]
                vel_cmd_pub.publish(vel_msg)
                print("右移")
            elif cmd == '6':
                vel_msg.data = [0.0, 0.0, 0.5, 0.0]
                vel_cmd_pub.publish(vel_msg)
                print("上升")
            elif cmd == '7':
                vel_msg.data = [0.0, 0.0, -0.5, 0.0]
                vel_cmd_pub.publish(vel_msg)
                print("下降")
            elif cmd == '8':
                vel_msg.data = [0.0, 0.0, 0.0, 0.5]
                vel_cmd_pub.publish(vel_msg)
                print("左转")
            elif cmd == '9':
                vel_msg.data = [0.0, 0.0, 0.0, -0.5]
                vel_cmd_pub.publish(vel_msg)
                print("右转")
            elif cmd == '0':
                cmd_msg.data = "HOVER"
                drone_cmd_pub.publish(cmd_msg)
                print("悬停")
            elif cmd == 'h':
                cmd_msg.data = "RTL"
                drone_cmd_pub.publish(cmd_msg)
                print("返航")
            elif cmd == 'l':
                cmd_msg.data = "AUTO_LAND"
                drone_cmd_pub.publish(cmd_msg)
                print("自动降落")
            elif cmd == 'e':
                cmd_msg.data = "EMERGENCY"
                drone_cmd_pub.publish(cmd_msg)
                print("紧急停止")
            elif cmd == 'q':
                print("退出")
                break
            else:
                print("未知命令")
    
    except KeyboardInterrupt:
        print("\n手动控制已退出")

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'test':
        test_drone_control()
    else:
        manual_control()
