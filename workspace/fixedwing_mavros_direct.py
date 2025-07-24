#!/usr/bin/env python3
"""
固定翼MAVROS直接控制脚本
绕过xtdrone，直接使用MAVROS控制固定翼
"""

import rospy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import TwistStamped
import sys

class FixedwingMavrosController:
    def __init__(self, vehicle_id=0):
        self.vehicle_id = vehicle_id
        self.namespace = f"standard_vtol_{vehicle_id}"
        
        rospy.init_node('fixedwing_mavros_controller', anonymous=True)
        
        # 状态
        self.current_state = State()
        
        # 订阅者
        self.state_sub = rospy.Subscriber(f"/{self.namespace}/mavros/state", State, self.state_callback)
        
        # 发布者
        self.velocity_pub = rospy.Publisher(f"/{self.namespace}/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
        
        # 服务客户端
        self.arming_client = rospy.ServiceProxy(f"/{self.namespace}/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy(f"/{self.namespace}/mavros/set_mode", SetMode)
        
        # 速度命令
        self.velocity_cmd = TwistStamped()
        
        print(f"固定翼MAVROS控制器已初始化: {self.namespace}")
        
    def state_callback(self, msg):
        self.current_state = msg
    
    def wait_for_connection(self, timeout=30):
        """等待与飞控连接"""
        print("等待与飞控连接...")
        rate = rospy.Rate(1)
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            if self.current_state.connected:
                print("✅ 已连接到飞控")
                return True
            
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                print("❌ 连接超时")
                return False
                
            rate.sleep()
        return False
    
    def set_offboard_mode(self):
        """设置OFFBOARD模式"""
        try:
            response = self.set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            if response.mode_sent:
                print("✅ OFFBOARD模式设置成功")
                return True
            else:
                print("❌ OFFBOARD模式设置失败")
                return False
        except Exception as e:
            print(f"❌ 设置模式时出错: {e}")
            return False
    
    def arm(self):
        """解锁飞机"""
        try:
            response = self.arming_client(value=True)
            if response.success:
                print("✅ 飞机解锁成功")
                return True
            else:
                print(f"❌ 飞机解锁失败: result={response.result}")
                return False
        except Exception as e:
            print(f"❌ 解锁时出错: {e}")
            return False
    
    def disarm(self):
        """上锁飞机"""
        try:
            response = self.arming_client(value=False)
            if response.success:
                print("✅ 飞机上锁成功")
                return True
            else:
                print(f"❌ 飞机上锁失败: result={response.result}")
                return False
        except Exception as e:
            print(f"❌ 上锁时出错: {e}")
            return False
    
    def set_velocity(self, forward, climb, yaw_rate):
        """设置速度命令"""
        self.velocity_cmd.header.stamp = rospy.Time.now()
        self.velocity_cmd.twist.linear.x = forward
        self.velocity_cmd.twist.linear.y = 0.0
        self.velocity_cmd.twist.linear.z = climb
        self.velocity_cmd.twist.angular.x = 0.0
        self.velocity_cmd.twist.angular.y = 0.0
        self.velocity_cmd.twist.angular.z = yaw_rate
        
        self.velocity_pub.publish(self.velocity_cmd)
    
    def auto_takeoff(self):
        """自动起飞流程"""
        print("开始固定翼自动起飞流程...")
        
        # 等待连接
        if not self.wait_for_connection():
            return False
        
        # 设置起飞速度
        print("设置起飞速度: 前进15m/s, 爬升2m/s")
        rate = rospy.Rate(20)  # 20Hz
        
        # 先发布一段时间的速度命令
        for i in range(40):  # 2秒
            self.set_velocity(15.0, 2.0, 0.0)
            rate.sleep()
        
        # 设置OFFBOARD模式
        print("设置OFFBOARD模式...")
        if not self.set_offboard_mode():
            return False
        
        # 继续发布速度命令
        for i in range(20):  # 1秒
            self.set_velocity(15.0, 2.0, 0.0)
            rate.sleep()
        
        # 解锁
        print("解锁飞机...")
        if not self.arm():
            return False
        
        print("✅ 固定翼起飞流程完成！")
        return True
    
    def print_status(self):
        """打印状态"""
        print(f"""
========== 固定翼状态 ==========
连接: {'✅' if self.current_state.connected else '❌'}
解锁: {'✅' if self.current_state.armed else '❌'}
模式: {self.current_state.mode}
引导: {'✅' if self.current_state.guided else '❌'}
系统状态: {self.current_state.system_status}
==============================
""")
    
    def spin(self):
        """主循环"""
        rate = rospy.Rate(20)  # 20Hz
        print("开始控制循环...")
        print("发送速度命令: 前进15m/s, 爬升2m/s")
        
        try:
            while not rospy.is_shutdown():
                # 持续发布速度命令
                self.set_velocity(15.0, 2.0, 0.0)
                rate.sleep()
        except KeyboardInterrupt:
            print("\n停止控制...")
            # 停止速度命令
            self.set_velocity(0.0, 0.0, 0.0)
            rospy.sleep(1)

def main():
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
    else:
        command = "takeoff"
    
    controller = FixedwingMavrosController(0)
    
    print("固定翼MAVROS直接控制")
    print("=" * 40)
    
    if command == "status":
        controller.print_status()
    elif command == "takeoff":
        success = controller.auto_takeoff()
        if success:
            print("起飞成功，开始持续控制...")
            controller.spin()
        else:
            print("起飞失败")
    elif command == "arm":
        controller.arm()
    elif command == "disarm":
        controller.disarm()
    elif command == "offboard":
        controller.set_offboard_mode()
    else:
        print("可用命令: takeoff, status, arm, disarm, offboard")

if __name__ == '__main__':
    main()
