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
        self.condition_timer = None  # 定时器用于定期发布condition
        
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
        
        # 启动20Hz定时器，持续发布condition状态
        self.start_condition_timer()
        
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
        """更新并发布condition状态"""
        self.current_condition = condition_value
        # 不再直接发布，而是通过定时器统一发布

    def _publish_condition_timer_callback(self, event):
        """定时器回调函数，20Hz发布当前condition状态"""
        if self.condition_pub is not None:
            msg = Int8()
            msg.data = self.current_condition
            self.condition_pub.publish(msg)
            # 只在condition改变时打印，避免过多输出
            if self.current_condition != self.last_sent_condition:
                print(f"发送Condition: 0x{self.current_condition:02X} (持续20Hz发布)")
                self.last_sent_condition = self.current_condition

    def start_condition_timer(self):
        """启动20Hz condition发布定时器"""
        if self.condition_timer is None:
            self.condition_timer = rospy.Timer(
                rospy.Duration(0.05),  # 20Hz = 1/0.05s
                self._publish_condition_timer_callback
            )
            print("✅ 启动20Hz Condition定时发布器")

    def stop_condition_timer(self):
        """停止condition定时发布"""
        if self.condition_timer is not None:
            self.condition_timer.shutdown()
            self.condition_timer = None
            print("停止Condition定时发布器")

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
        self.stop_condition_timer()
        print("ROS通信器关闭")


class PersonPositionReader:
    """人员位置读取器 - 从ROS话题获取人员位置"""
    
    def __init__(self):
        self.positions = {}  # 存储人员位置 {name: (x, y, z)}
        self.subscribers = []
        self.position_lock = threading.Lock()
        
        print("初始化人员位置读取器")
    
    def start_listening(self):
        """开始监听人员位置话题"""
        # 监听三个人员的位置话题 - 修正话题名称
        person_topics = [
            ("/person_yellow/position", "Person_Yellow"),
            ("/person_white/position", "Person_White"), 
            ("/person_red/position", "Person_Red")
        ]
        
        for topic, name in person_topics:
            try:
                # 使用Pose消息类型而不是PoseStamped
                from geometry_msgs.msg import Pose
                sub = rospy.Subscriber(
                    topic, 
                    Pose, 
                    lambda msg, person_name=name: self._position_callback(msg, person_name),
                    queue_size=1
                )
                self.subscribers.append(sub)
                print(f"✅ 开始监听 {name} 位置话题: {topic}")
            except Exception as e:
                print(f"❌ 监听 {name} 位置话题失败: {e}")
    
    def _position_callback(self, msg, person_name):
        """位置回调函数 - 适配Pose消息"""
        with self.position_lock:
            x = msg.position.x
            y = msg.position.y
            z = msg.position.z
            self.positions[person_name] = (x, y, z)
            # 只在首次接收时打印
            if len(self.positions) <= 3:
                print(f"📍 接收到 {person_name} 位置: ({x:.1f}, {y:.1f}, {z:.1f})")
    
    def update_positions_once(self):
        """主动更新一次人员位置 - 只在任务3完成后调用一次"""
        print("🔄 主动更新人员位置...")
        
        # 开始监听（如果还没有开始）
        if not self.subscribers:
            self.start_listening()
        
        # 等待获取位置数据
        import time
        timeout = 5.0
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            with self.position_lock:
                if len(self.positions) >= 3:
                    print(f"✅ 成功获取{len(self.positions)}个人员位置")
                    for name, (x, y, z) in self.positions.items():
                        print(f"   {name}: ({x:.1f}, {y:.1f}, {z:.1f})")
                    return True
            time.sleep(0.1)
        
        # 如果无法获取真实数据，使用模拟数据
        print(f"⚠️ 超时，只获取到{len(self.positions)}个人员位置，使用模拟数据")
        self._generate_mock_positions()
        return len(self.positions) > 0
    
    def _generate_mock_positions(self):
        """生成模拟的人员位置数据"""
        print("🎭 生成模拟人员位置数据...")
        mock_positions = {
            "Person_Yellow": (1495, 249, 0),  # y坐标最小
            "Person_White": (1495, 250, 0),      # y坐标中等  
            "Person_Red": (1495, 251, 0)       # y坐标最大
        }
        
        with self.position_lock:
            self.positions.update(mock_positions)
            
        print("📍 模拟人员位置:")
        for name, (x, y, z) in mock_positions.items():
            print(f"   {name}: ({x:.1f}, {y:.1f}, {z:.1f})")
    
    def get_sorted_positions(self):
        """获取按y坐标排序的人员位置列表"""
        with self.position_lock:
            if not self.positions:
                print("❌ 没有人员位置数据")
                return []
            
            # 按y坐标从小到大排序
            sorted_positions = []
            for name, (x, y, z) in self.positions.items():
                sorted_positions.append((x, y, z, name))
            
            sorted_positions.sort(key=lambda pos: pos[1])  # 按y坐标排序
            
            print(f"📍 按y坐标排序的人员位置:")
            for i, (x, y, z, name) in enumerate(sorted_positions, 1):
                print(f"   {i}. {name}: ({x:.1f}, {y:.1f}, {z:.1f})")
            
            return sorted_positions
    
    def shutdown(self):
        """关闭人员位置读取器"""
        print("关闭人员位置读取器...")
        for sub in self.subscribers:
            sub.unregister()
        self.subscribers.clear()
        self.positions.clear()


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
