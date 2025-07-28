#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose
import time

class PersonPositionMonitor:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('person_position_monitor', anonymous=True)
        
        # 要监控的人物模型名称
        self.target_models = ['person_yellow', 'person_white', 'person_red']
        
        # 创建服务客户端
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        
        # 订阅模型状态话题（备用方法）
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        
        # 存储最新的模型状态
        self.latest_model_states = {}
        
        # 发布各个人物的坐标信息
        self.yellow_pub = rospy.Publisher('/person_yellow/position', Pose, queue_size=1)
        self.white_pub = rospy.Publisher('/person_white/position', Pose, queue_size=1)
        self.red_pub = rospy.Publisher('/person_red/position', Pose, queue_size=1)
        
        print("人物坐标监控器已启动...")
        print("监控目标: {}".format(', '.join(self.target_models)))
        
    def model_states_callback(self, msg):
        """接收模型状态回调函数"""
        for i, name in enumerate(msg.name):
            if name in self.target_models:
                self.latest_model_states[name] = msg.pose[i]
                
    def get_position_by_service(self, model_name):
        """通过服务获取模型位置"""
        try:
            response = self.get_model_state(model_name, "")
            if response.success:
                return response.pose
            else:
                rospy.logwarn("无法获取模型 {} 的状态: {}".format(model_name, response.status_message))
                return None
        except rospy.ServiceException as e:
            rospy.logwarn("服务调用失败: {}".format(e))
            return None
            
    def get_position_by_topic(self, model_name):
        """通过话题获取模型位置"""
        return self.latest_model_states.get(model_name, None)
        
    def print_positions(self):
        """打印所有人物的位置信息"""
        print("\n" + "="*60)
        print("时间: {}".format(time.strftime("%Y-%m-%d %H:%M:%S")))
        print("-"*60)
        
        for model_name in self.target_models:
            # 优先使用话题数据，如果没有则使用服务
            pose = self.get_position_by_topic(model_name)
            if pose is None:
                pose = self.get_position_by_service(model_name)
            
            if pose is not None:
                x = pose.position.x
                y = pose.position.y
                z = pose.position.z
                print("{:15s}: x={:8.3f}, y={:8.3f}, z={:8.3f}".format(
                    model_name, x, y, z))
                
                # 发布到对应话题
                if model_name == 'person_yellow':
                    self.yellow_pub.publish(pose)
                elif model_name == 'person_white':
                    self.white_pub.publish(pose)
                elif model_name == 'person_red':
                    self.red_pub.publish(pose)
            else:
                print("{:15s}: 未找到或无法获取坐标".format(model_name))
                
    def run_continuous_monitor(self):
        """持续监控模式"""
        rate = rospy.Rate(2)  # 2Hz更新频率
        
        print("\n开始持续监控模式 (按 Ctrl+C 退出)...")
        
        try:
            while not rospy.is_shutdown():
                self.print_positions()
                rate.sleep()
        except KeyboardInterrupt:
            print("\n监控已停止。")
            
    def run_single_query(self):
        """单次查询模式"""
        print("\n单次查询结果:")
        self.print_positions()

def main():
    try:
        monitor = PersonPositionMonitor()
        
        # 等待一下让话题连接建立
        rospy.sleep(1.0)
        
        print("\n选择运行模式:")
        print("1. 持续监控 (实时显示坐标)")
        print("2. 单次查询")
        
        try:
            choice = input("请输入选择 (1 或 2): ").strip()
        except:
            choice = "2"  # 默认单次查询
            
        if choice == "1":
            monitor.run_continuous_monitor()
        elif choice == "2":
            monitor.run_single_query()
        else:
            print("无效选择，执行单次查询...")
            monitor.run_single_query()
            
    except rospy.ROSInterruptException:
        print("程序被中断。")

if __name__ == '__main__':
    main()
