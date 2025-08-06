#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Pose, PoseStamped
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
        
        # 创建VTOL目标位姿发布者
        self.vtol_target_pose_pubs = {}
        self.target_topics = {
            'red': '/zhihang2025/first_man/pose',
            'yellow': '/zhihang2025/second_man/pose',
            'white': '/zhihang2025/third_man/pose'
        }
        
        for target_name, topic_name in self.target_topics.items():
            self.vtol_target_pose_pubs[target_name] = rospy.Publisher(topic_name, PoseStamped, queue_size=1)
            rospy.loginfo(f"VTOL目标位姿发布话题: {topic_name}")
        
        # 为了兼容性，保留旧的话题发布者
        self.yellow_pub = rospy.Publisher('/person_yellow/position', Pose, queue_size=1)
        self.white_pub = rospy.Publisher('/person_white/position', Pose, queue_size=1)
        self.red_pub = rospy.Publisher('/person_red/position', Pose, queue_size=1)
        
        print("人物坐标监控器已启动...")
        print("监控目标: {}".format(', '.join(self.target_models)))
        print("发布话题:")
        for target_name, topic_name in self.target_topics.items():
            print(f"  person_{target_name} -> {topic_name}")
        
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
                    # 发布到VTOL目标话题
                    pose_stamped = PoseStamped()
                    pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.header.frame_id = "map"
                    pose_stamped.pose = pose
                    self.vtol_target_pose_pubs['yellow'].publish(pose_stamped)
                elif model_name == 'person_white':
                    self.white_pub.publish(pose)
                    # 发布到VTOL目标话题
                    pose_stamped = PoseStamped()
                    pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.header.frame_id = "map"
                    pose_stamped.pose = pose
                    self.vtol_target_pose_pubs['white'].publish(pose_stamped)
                elif model_name == 'person_red':
                    self.red_pub.publish(pose)
                    # 发布到VTOL目标话题
                    pose_stamped = PoseStamped()
                    pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.header.frame_id = "map"
                    pose_stamped.pose = pose
                    self.vtol_target_pose_pubs['red'].publish(pose_stamped)
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
            
def main():
    try:
        monitor = PersonPositionMonitor()
        
        # 等待一下让话题连接建立
        rospy.sleep(1.0)
        
        print("\n启动持续监控模式 (按 Ctrl+C 退出)...")
        monitor.run_continuous_monitor()
            
    except rospy.ROSInterruptException:
        print("程序被中断。")

if __name__ == '__main__':
    main()
