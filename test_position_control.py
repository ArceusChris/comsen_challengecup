#!/usr/bin/env python3
"""
测试位置控制和无人机响应
专门验证无人机是否能跟随位置指令
"""

import time
import math
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import NavSatFix

class PositionControlTest:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('position_control_test', anonymous=True)
        print("位置控制测试初始化...")
        
        # 状态变量
        self.current_state = State()
        self.current_position = None
        self.target_position = PoseStamped()
        
        # 使用VTOL无人机的命名空间
        vtol_ns = "/standard_vtol_0"
        
        # 发布器
        self.position_pub = rospy.Publisher(f'{vtol_ns}/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        
        # 订阅器
        self.state_sub = rospy.Subscriber(f'{vtol_ns}/mavros/state', State, self.state_callback)
        self.position_sub = rospy.Subscriber(f'{vtol_ns}/mavros/local_position/pose', PoseStamped, self.position_callback)
        
        # 服务客户端
        self.arming_client = rospy.ServiceProxy(f'{vtol_ns}/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy(f'{vtol_ns}/mavros/set_mode', SetMode)
        
        # 等待连接
        print("等待MAVROS连接...")
        while not self.current_state.connected:
            rospy.sleep(0.1)
        print("✅ MAVROS连接成功")
        
        # 初始化目标位置
        self.target_position.header.frame_id = "map"
        self.target_position.pose.position.x = 0
        self.target_position.pose.position.y = 0
        self.target_position.pose.position.z = 5
        self.target_position.pose.orientation.w = 1.0
        
        # 开始发布初始位置指令
        for _ in range(100):
            self.position_pub.publish(self.target_position)
            rospy.sleep(0.05)
    
    def state_callback(self, msg):
        self.current_state = msg
    
    def position_callback(self, msg):
        self.current_position = msg.pose.position
    
    def set_position(self, x, y, z):
        """设置目标位置"""
        self.target_position.header.stamp = rospy.Time.now()
        self.target_position.pose.position.x = x
        self.target_position.pose.position.y = y
        self.target_position.pose.position.z = z
        self.position_pub.publish(self.target_position)
    
    def get_distance_to_target(self, target_x, target_y, target_z):
        """计算到目标的距离"""
        if not self.current_position:
            return float('inf')
        
        dx = self.current_position.x - target_x
        dy = self.current_position.y - target_y
        dz = self.current_position.z - target_z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def set_mode(self, mode):
        """设置飞行模式"""
        try:
            response = self.set_mode_client(custom_mode=mode)
            if response.mode_sent:
                print(f"✅ 成功设置模式: {mode}")
                return True
            else:
                print(f"❌ 设置模式失败: {mode}")
                return False
        except Exception as e:
            print(f"❌ 设置模式异常: {e}")
            return False
    
    def arm(self):
        """解锁无人机"""
        try:
            response = self.arming_client(True)
            if response.success:
                print("✅ 无人机解锁成功")
                return True
            else:
                print("❌ 无人机解锁失败")
                return False
        except Exception as e:
            print(f"❌ 解锁异常: {e}")
            return False
    
    def test_basic_positioning(self):
        """测试基本位置控制"""
        print("\n🧪 基本位置控制测试")
        print("="*50)
        
        if not self.current_position:
            print("❌ 无法获取当前位置")
            return False
        
        print(f"当前位置: ({self.current_position.x:.1f}, {self.current_position.y:.1f}, {self.current_position.z:.1f})")
        
        # 1. 设置OFFBOARD模式
        print("设置OFFBOARD模式...")
        if not self.set_mode("OFFBOARD"):
            return False
        
        time.sleep(2)
        
        # 2. 解锁
        print("解锁无人机...")
        if not self.arm():
            return False
        
        time.sleep(2)
        
        # 3. 测试基本起飞
        print("测试起飞到10米高度...")
        start_x = self.current_position.x
        start_y = self.current_position.y
        
        for height in [5, 10, 15, 20]:
            print(f"目标高度: {height}m")
            target_x, target_y, target_z = start_x, start_y, height
            
            # 持续发布位置指令30秒
            start_time = time.time()
            while time.time() - start_time < 30:
                self.set_position(target_x, target_y, target_z)
                rospy.sleep(0.1)  # 10Hz发布频率
                
                # 每2秒检查一次距离
                if int((time.time() - start_time) * 10) % 20 == 0:
                    distance = self.get_distance_to_target(target_x, target_y, target_z)
                    print(f"  时间: {time.time() - start_time:.1f}s, 距离目标: {distance:.1f}m")
                    print(f"  当前位置: ({self.current_position.x:.1f}, {self.current_position.y:.1f}, {self.current_position.z:.1f})")
                    
                    # 如果足够接近，提前结束
                    if distance < 3.0:
                        print(f"  ✅ 到达高度 {height}m")
                        break
        
        return True
    
    def test_horizontal_movement(self):
        """测试水平移动"""
        print("\n🧪 水平移动测试")
        print("="*50)
        
        # 测试目标点
        test_points = [
            (50, 0, 20),    # 东50米
            (50, 50, 20),   # 东北
            (0, 50, 20),    # 北50米
            (-50, 50, 20),  # 西北
            (-50, 0, 20),   # 西50米
            (-50, -50, 20), # 西南
            (0, -50, 20),   # 南50米
            (50, -50, 20),  # 东南
            (0, 0, 20),     # 回到原点
        ]
        
        for i, (target_x, target_y, target_z) in enumerate(test_points):
            print(f"\n目标点 {i+1}: ({target_x}, {target_y}, {target_z})")
            
            # 持续发布位置指令60秒
            start_time = time.time()
            min_distance = float('inf')
            
            while time.time() - start_time < 60:
                self.set_position(target_x, target_y, target_z)
                rospy.sleep(0.1)  # 10Hz
                
                # 每3秒检查一次
                if int((time.time() - start_time) * 10) % 30 == 0:
                    distance = self.get_distance_to_target(target_x, target_y, target_z)
                    min_distance = min(min_distance, distance)
                    
                    print(f"  时间: {time.time() - start_time:.1f}s, 距离: {distance:.1f}m, 最小距离: {min_distance:.1f}m")
                    print(f"  位置: ({self.current_position.x:.1f}, {self.current_position.y:.1f}, {self.current_position.z:.1f})")
                    
                    # 如果足够接近，等待一段时间后移动到下一个点
                    if distance < 10.0:
                        print(f"  ✅ 接近目标点 {i+1}")
                        break
            
            print(f"  📊 目标点 {i+1} 完成，最小距离: {min_distance:.1f}m")
    
    def test_mode_switching(self):
        """测试模式切换"""
        print("\n🧪 模式切换测试")
        print("="*50)
        
        modes = ["OFFBOARD", "plane", "multirotor", "OFFBOARD"]
        
        for mode in modes:
            print(f"切换到模式: {mode}")
            self.set_mode(mode)
            time.sleep(5)
            
            # 在每个模式下发布位置指令
            for _ in range(50):  # 5秒
                self.set_position(0, 0, 20)
                rospy.sleep(0.1)
            
            if self.current_position:
                print(f"  当前位置: ({self.current_position.x:.1f}, {self.current_position.y:.1f}, {self.current_position.z:.1f})")

def main():
    try:
        test = PositionControlTest()
        
        print("🚀 开始位置控制测试")
        
        # 等待位置信息
        print("等待位置信息...")
        while not test.current_position:
            rospy.sleep(0.1)
        
        # 运行测试
        test.test_basic_positioning()
        test.test_horizontal_movement()
        test.test_mode_switching()
        
        print("\n🎉 测试完成")
        
    except KeyboardInterrupt:
        print("\n用户中断测试")
    except Exception as e:
        print(f"\n❌ 测试异常: {e}")

if __name__ == "__main__":
    main()
