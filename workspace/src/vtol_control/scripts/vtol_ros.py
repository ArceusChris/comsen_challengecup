#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
VTOL ROS通信模块
专门处理ROS话题订阅、发布和相关通信功能
与飞行控制逻辑分离，提高代码模块化程度
'''

import rospy
import math
import threading
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String, Int8


class VTOLROSCommunicator:
    """VTOL ROS通信管理器"""
    
    def __init__(self, vehicle_type="standard_vtol", vehicle_id="0"):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        
        # 当前状态
        self.current_position = None
        self.current_yaw = 0
        
        # 位置指令持续发布
        self.target_pose = Pose()
        self.should_publish = False
        
        # Condition状态管理
        self.current_condition = 0xAA  # 初始化为0xAA
        self.last_sent_condition = None
        
        # 回调函数
        self.position_callback = None
        self.condition_callback = None
        
        # ROS发布者和订阅者
        self.local_pose_sub = None
        self.vtol_condition_sub = None
        self.cmd_pub = None
        self.cmd_pose_pub = None
        self.condition_pub = None
        
        print(f"初始化VTOL ROS通信器: {self.vehicle_type}_{self.vehicle_id}")

    def init_ros_communication(self):
        """初始化ROS通信（节点应已在主函数中初始化）"""
        # 订阅者
        self.local_pose_sub = rospy.Subscriber(
            f"{self.vehicle_type}_{self.vehicle_id}/mavros/local_position/pose", 
            PoseStamped, self._local_pose_callback, queue_size=1)
        
        # Condition话题订阅者
        self.vtol_condition_sub = rospy.Subscriber(
            '/zhihang2025/vtol_land_sub/done', 
            Int8, self._done_callback, queue_size=1)
        
        # 发布者
        self.cmd_pub = rospy.Publisher(
            f"/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd", 
            String, queue_size=10)
        
        self.cmd_pose_pub = rospy.Publisher(
            f"/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd_pose_enu", 
            Pose, queue_size=10)
        
        # Condition状态发布者
        self.condition_pub = rospy.Publisher(
            '/zhihang2025/vtol_land_sub/done', 
            Int8, queue_size=10)
        
        # 发送初始condition状态
        self.publish_condition(self.current_condition)
        
        print("ROS通信初始化完成")

    def _local_pose_callback(self, msg):
        """位置回调函数（内部）"""
        self.current_position = msg.pose.position
        if self.position_callback:
            self.position_callback(msg.pose.position)

    def _done_callback(self, msg):
        """Condition话题回调函数（内部）"""
        received_condition = msg.data
        print(f"收到Condition: 0x{received_condition:02X}")
        if self.condition_callback:
            self.condition_callback(received_condition)

    def set_position_callback(self, callback_func):
        """设置位置更新回调函数"""
        self.position_callback = callback_func

    def set_condition_callback(self, callback_func):
        """设置condition接收回调函数"""
        self.condition_callback = callback_func

    def publish_condition(self, condition_value):
        """发布condition状态"""
        if condition_value != self.last_sent_condition:
            msg = Int8()
            msg.data = condition_value
            self.condition_pub.publish(msg)
            self.last_sent_condition = condition_value
            print(f"发送Condition: 0x{condition_value:02X}")

    def send_command(self, cmd_str):
        """发送xtdrone命令"""
        cmd_msg = String()
        cmd_msg.data = cmd_str
        self.cmd_pub.publish(cmd_msg)
        print(f"发送命令: {cmd_str}")

    def set_target_pose(self, x, y, z, yaw=0.0):
        """设置目标位置并开始持续发布"""
        self.target_pose.position.x = x
        self.target_pose.position.y = y
        self.target_pose.position.z = z
        self.target_pose.orientation.x = 0.0
        self.target_pose.orientation.y = 0.0
        self.target_pose.orientation.z = math.sin(yaw / 2.0)
        self.target_pose.orientation.w = math.cos(yaw / 2.0)
        
        if not self.should_publish:
            self.should_publish = True
            publish_thread = threading.Thread(target=self._continuous_publish)
            publish_thread.daemon = True
            publish_thread.start()
            print(f"开始持续发布位置指令: x={x:.1f}, y={y:.1f}, z={z:.1f}")
        else:
            print(f"更新目标位置: x={x:.1f}, y={y:.1f}, z={z:.1f}")

    def stop_publishing(self):
        """停止持续发布位置指令"""
        self.should_publish = False
        print("停止位置指令发布")

    def _continuous_publish(self):
        """持续发布位置指令 - 闭环控制版本"""
        rate = rospy.Rate(50)  # 提高到50Hz获得更好的控制性能
        last_distance = float('inf')
        stable_count = 0
        
        while self.should_publish and not rospy.is_shutdown():
            try:
                # 发布当前目标位置
                self.cmd_pose_pub.publish(self.target_pose)
                
                # 实时监控距离变化
                if self.current_position is not None:
                    current_distance = math.sqrt(
                        (self.target_pose.position.x - self.current_position.x)**2 +
                        (self.target_pose.position.y - self.current_position.y)**2 +
                        (self.target_pose.position.z - self.current_position.z)**2
                    )
                    
                    # 检测是否接近目标
                    if current_distance < 25.0:  # 25米内认为接近
                        stable_count += 1
                        if stable_count > 100:  # 连续2秒(50Hz*2s=100)保持接近
                            rospy.loginfo(f"目标位置稳定到达，距离: {current_distance:.1f}m")
                    else:
                        stable_count = 0
                    
                    last_distance = current_distance
                
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def get_distance_to_target(self, target_x, target_y, target_z):
        """计算到目标点的距离"""
        if self.current_position is None:
            return float('inf')
        
        return math.sqrt(
            (target_x - self.current_position.x)**2 +
            (target_y - self.current_position.y)**2 +
            (target_z - self.current_position.z)**2
        )

    def get_current_position(self):
        """获取当前位置"""
        return self.current_position

    def is_ros_ok(self):
        """检查ROS状态"""
        return not rospy.is_shutdown()

    def wait_for_position(self, timeout=10):
        """等待获取位置信息"""
        import time
        start_time = time.time()
        while self.current_position is None and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        return self.current_position is not None

    def shutdown(self):
        """关闭ROS通信"""
        self.stop_publishing()
        print("ROS通信器关闭")


def test_ros_communication():
    """测试ROS通信功能"""
    print("测试VTOL ROS通信模块")
    
    try:
        # 初始化ROS节点
        rospy.init_node('vtol_ros_test', anonymous=True)
        
        # 创建通信器
        communicator = VTOLROSCommunicator()
        
        # 初始化通信
        communicator.init_ros_communication()
        
        # 等待位置信息
        if communicator.wait_for_position(timeout=5):
            pos = communicator.get_current_position()
            print(f"✅ 成功获取位置: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f})")
        else:
            print("⚠️ 未能获取位置信息")
        
        # 测试命令发送
        communicator.send_command("ARM")
        
        # 测试condition发布
        communicator.publish_condition(0x01)
        
        print("✅ ROS通信测试完成")
        
    except Exception as e:
        print(f"❌ ROS通信测试失败: {e}")


if __name__ == "__main__":
    test_ros_communication()
