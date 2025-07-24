#!/usr/bin/env python
"""
固定翼无人机控制脚本
支持MAVROS控制、瞬移、状态监控等功能
"""
import rospy
from mavros_msgs.msg import State, OverrideRCIn
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import math
import sys
import time

class FixedWingController:
    def __init__(self, vehicle_id=0):
        self.vehicle_id = vehicle_id
        self.namespace = f"mavros"  # 根据实际情况可能需要调整
        
        # 初始化ROS节点
        rospy.init_node('fixedwing_controller', anonymous=True)
        
        # 状态变量
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.current_gps = NavSatFix()
        
        # 订阅者
        self.state_sub = rospy.Subscriber(f"{self.namespace}/state", State, self.state_callback)
        self.pose_sub = rospy.Subscriber(f"{self.namespace}/local_position/pose", PoseStamped, self.pose_callback)
        self.gps_sub = rospy.Subscriber(f"{self.namespace}/global_position/global", NavSatFix, self.gps_callback)
        
        # 发布者
        self.rc_override_pub = rospy.Publisher(f"{self.namespace}/rc/override", OverrideRCIn, queue_size=10)
        self.velocity_pub = rospy.Publisher(f"{self.namespace}/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        
        # 服务客户端
        self.arming_client = rospy.ServiceProxy(f"{self.namespace}/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy(f"{self.namespace}/set_mode", SetMode)
        self.takeoff_client = rospy.ServiceProxy(f"{self.namespace}/cmd/takeoff", CommandTOL)
        
        # Gazebo瞬移服务
        self.set_model_state_client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        print(f"固定翼控制器已初始化 (ID: {vehicle_id})")
        
    def state_callback(self, msg):
        self.current_state = msg
        
    def pose_callback(self, msg):
        self.current_pose = msg
        
    def gps_callback(self, msg):
        self.current_gps = msg
    
    def wait_for_connection(self, timeout=30):
        """等待与飞控连接"""
        print("等待与飞控连接...")
        rate = rospy.Rate(1)
        start_time = time.time()
        
        while not rospy.is_shutdown() and not self.current_state.connected:
            if time.time() - start_time > timeout:
                print("❌ 连接超时!")
                return False
            rate.sleep()
            
        print("✅ 已连接到飞控")
        return True
    
    def set_mode(self, mode):
        """设置飞行模式"""
        try:
            response = self.set_mode_client(custom_mode=mode)
            if response.mode_sent:
                print(f"✅ 模式已设置为: {mode}")
                return True
            else:
                print(f"❌ 设置模式失败: {mode}")
                return False
        except rospy.ServiceException as e:
            print(f"❌ 设置模式服务调用失败: {e}")
            return False
    
    def arm(self):
        """解锁无人机"""
        try:
            response = self.arming_client(value=True)
            if response.success:
                print("✅ 无人机已解锁")
                return True
            else:
                print("❌ 解锁失败")
                return False
        except rospy.ServiceException as e:
            print(f"❌ 解锁服务调用失败: {e}")
            return False
    
    def disarm(self):
        """上锁无人机"""
        try:
            response = self.arming_client(value=False)
            if response.success:
                print("✅ 无人机已上锁")
                return True
            else:
                print("❌ 上锁失败")
                return False
        except rospy.ServiceException as e:
            print(f"❌ 上锁服务调用失败: {e}")
            return False
    
    def send_rc_override(self, throttle=1500, aileron=1500, elevator=1500, rudder=1500):
        """
        发送RC遥控器覆盖指令
        固定翼控制通道:
        - throttle: 油门 (1000-2000, 1500为中点)
        - aileron: 副翼 (控制滚转)
        - elevator: 升降舵 (控制俯仰)
        - rudder: 方向舵 (控制偏航)
        """
        rc_msg = OverrideRCIn()
        rc_msg.channels = [0] * 18  # 初始化18个通道
        rc_msg.channels[0] = aileron   # 通道1: 副翼
        rc_msg.channels[1] = elevator  # 通道2: 升降舵
        rc_msg.channels[2] = throttle  # 通道3: 油门
        rc_msg.channels[3] = rudder    # 通道4: 方向舵
        
        self.rc_override_pub.publish(rc_msg)
    
    def takeoff_sequence(self, altitude=50):
        """固定翼起飞序列"""
        print("开始固定翼起飞序列...")
        
        # 1. 设置为手动模式
        if not self.set_mode("MANUAL"):
            return False
        
        # 2. 解锁
        if not self.arm():
            return False
        
        # 3. 增加油门进行起飞
        print("增加油门进行起飞...")
        for i in range(10):
            throttle = 1500 + i * 50  # 逐步增加油门
            self.send_rc_override(throttle=throttle)
            time.sleep(0.5)
        
        # 4. 切换到自动模式
        time.sleep(2)
        self.set_mode("AUTO")
        
        print("✅ 起飞序列完成")
        return True
    
    def land(self):
        """降落"""
        print("开始降落...")
        return self.set_mode("RTL")  # Return to Launch
    
    def gazebo_teleport(self, x, y, z, yaw=0.0, forward_speed=15.0):
        """在Gazebo中瞬移固定翼"""
        try:
            model_state = ModelState()
            model_state.model_name = f"iris_{self.vehicle_id}"  # 根据实际模型名称调整
            
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
            
            # 设置前进速度
            model_state.twist.linear.x = forward_speed * math.cos(yaw)
            model_state.twist.linear.y = forward_speed * math.sin(yaw)
            model_state.twist.linear.z = 0.0
            model_state.twist.angular.x = 0.0
            model_state.twist.angular.y = 0.0
            model_state.twist.angular.z = 0.0
            
            model_state.reference_frame = 'world'
            
            response = self.set_model_state_client(model_state)
            if response.success:
                print(f"✅ 瞬移成功: ({x}, {y}, {z}), 速度: {forward_speed} m/s")
                return True
            else:
                print(f"❌ 瞬移失败: {response.status_message}")
                return False
                
        except Exception as e:
            print(f"❌ 瞬移错误: {e}")
            return False
    
    def get_status(self):
        """获取无人机状态"""
        print("\n=== 固定翼状态 ===")
        print(f"连接状态: {'✅ 已连接' if self.current_state.connected else '❌ 未连接'}")
        print(f"解锁状态: {'✅ 已解锁' if self.current_state.armed else '🔒 已上锁'}")
        print(f"飞行模式: {self.current_state.mode}")
        
        if self.current_pose.header.stamp.secs > 0:
            pos = self.current_pose.pose.position
            print(f"位置: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}")
        
        if self.current_gps.header.stamp.secs > 0:
            print(f"GPS: lat={self.current_gps.latitude:.6f}, lon={self.current_gps.longitude:.6f}")
        
        print("==================\n")

def main():
    if len(sys.argv) < 2:
        print("固定翼无人机控制脚本")
        print("Usage: python fixedwing_control.py <command> [args...]")
        print("\n可用命令:")
        print("  status                    - 显示状态")
        print("  takeoff [altitude]        - 起飞")
        print("  land                      - 降落")
        print("  arm                       - 解锁")
        print("  disarm                    - 上锁")
        print("  mode <mode_name>          - 设置飞行模式")
        print("  teleport <x> <y> <z> [yaw] [speed] - Gazebo瞬移")
        print("  rc <throttle> <aileron> <elevator> <rudder> - RC控制")
        print("\n示例:")
        print("  python fixedwing_control.py status")
        print("  python fixedwing_control.py takeoff 100")
        print("  python fixedwing_control.py teleport 1000 500 80 45 20")
        print("  python fixedwing_control.py mode AUTO")
        sys.exit(1)
    
    controller = FixedWingController()
    
    # 等待连接 (可选)
    # controller.wait_for_connection()
    
    command = sys.argv[1].lower()
    
    if command == "status":
        controller.get_status()
        
    elif command == "takeoff":
        altitude = float(sys.argv[2]) if len(sys.argv) > 2 else 50
        controller.takeoff_sequence(altitude)
        
    elif command == "land":
        controller.land()
        
    elif command == "arm":
        controller.arm()
        
    elif command == "disarm":
        controller.disarm()
        
    elif command == "mode":
        if len(sys.argv) < 3:
            print("请指定飞行模式")
            sys.exit(1)
        mode = sys.argv[2]
        controller.set_mode(mode)
        
    elif command == "teleport":
        if len(sys.argv) < 5:
            print("瞬移需要坐标参数: x y z [yaw] [speed]")
            sys.exit(1)
        x = float(sys.argv[2])
        y = float(sys.argv[3])
        z = float(sys.argv[4])
        yaw = math.radians(float(sys.argv[5])) if len(sys.argv) > 5 else 0.0
        speed = float(sys.argv[6]) if len(sys.argv) > 6 else 15.0
        controller.gazebo_teleport(x, y, z, yaw, speed)
        
    elif command == "rc":
        if len(sys.argv) < 6:
            print("RC控制需要4个参数: throttle aileron elevator rudder")
            sys.exit(1)
        throttle = int(sys.argv[2])
        aileron = int(sys.argv[3])
        elevator = int(sys.argv[4])
        rudder = int(sys.argv[5])
        controller.send_rc_override(throttle, aileron, elevator, rudder)
        print(f"RC指令已发送: T={throttle}, A={aileron}, E={elevator}, R={rudder}")
        
    else:
        print(f"未知命令: {command}")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n操作已取消")
    except Exception as e:
        print(f"错误: {e}")
