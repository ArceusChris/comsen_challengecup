#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from gazebo_msgs.srv import GetModelState
import sys

def get_person_positions():
    """获取所有人物的坐标信息"""
    
    # 初始化ROS节点
    rospy.init_node('quick_position_check', anonymous=True)
    
    # 要查询的模型列表
    models = ['person_yellow', 'person_white', 'person_red']
    
    # 创建服务客户端
    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    
    print("="*60)
    print("人物坐标查询结果")
    print("="*60)
    
    for model_name in models:
        try:
            response = get_model_state(model_name, "")
            if response.success:
                x = response.pose.position.x
                y = response.pose.position.y
                z = response.pose.position.z
                print("{:15s}: x={:8.3f}, y={:8.3f}, z={:8.3f}".format(
                    model_name, x, y, z))
            else:
                print("{:15s}: 模型不存在或无法获取".format(model_name))
        except rospy.ServiceException as e:
            print("{:15s}: 服务调用失败 - {}".format(model_name, e))
    
    print("="*60)

if __name__ == '__main__':
    try:
        get_person_positions()
    except rospy.ROSInterruptException:
        print("程序被中断")
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print("发生错误: {}".format(e))
