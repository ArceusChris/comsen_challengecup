#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import select
import termios
import tty
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import PointStamped, PoseStamped
from mavros_msgs.msg import State

class PrecisionLandingTestClient:
    """
    精准降落测试客户端 - 提供交互式控制和监控
    """
    
    def __init__(self):
        rospy.init_node('precision_landing_test_client', anonymous=True)
        
        # 状态变量
        self.landing_state = "UNKNOWN"
        self.target_detected = False
        self.target_position = PointStamped()
        self.drone_pose = PoseStamped()
        self.drone_state = State()
        self.landing_progress = 0.0
        self.precision_controller_enabled = False
        
        # 订阅状态信息
        self.state_sub = rospy.Subscriber('/precision_landing/state', String, self.landing_state_callback)
        self.progress_sub = rospy.Subscriber('/precision_landing/progress', Float64, self.progress_callback)
        self.status_sub = rospy.Subscriber('/precision_landing/status', String, self.status_callback)
        
        # 订阅目标检测
        self.target_sub_basic = rospy.Subscriber('/landing_target', PointStamped, self.target_callback)
        self.target_sub_camo = rospy.Subscriber('/landing_target_camo', PointStamped, self.target_callback)
        self.target_sub_red = rospy.Subscriber('/landing_target_red', PointStamped, self.target_callback)
        
        # 订阅无人机状态
        self.drone_pose_sub = rospy.Subscriber('/iris_0/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.drone_state_sub = rospy.Subscriber('/iris_0/mavros/state', State, self.drone_state_callback)
        
        # 发布控制命令
        self.start_pub = rospy.Publisher('/precision_landing/start', Bool, queue_size=1)
        self.abort_pub = rospy.Publisher('/precision_landing/abort', Bool, queue_size=1)
        self.enable_pub = rospy.Publisher('/precision_landing/enable', Bool, queue_size=1)
        
        # 终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        
        rospy.loginfo("Precision Landing Test Client initialized")
        
    def landing_state_callback(self, msg):
        """降落状态回调"""
        self.landing_state = msg.data
        
    def progress_callback(self, msg):
        """进度回调"""
        self.landing_progress = msg.data
        
    def status_callback(self, msg):
        """状态回调"""
        pass  # 状态信息已在主循环中显示
        
    def target_callback(self, msg):
        """目标检测回调"""
        self.target_position = msg
        self.target_detected = True
        
    def pose_callback(self, msg):
        """位置回调"""
        self.drone_pose = msg
        
    def drone_state_callback(self, msg):
        """无人机状态回调"""
        self.drone_state = msg
        
    def get_key(self):
        """获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def print_status(self):
        """打印状态信息"""
        print("\033[2J\033[H")  # 清屏
        print("=" * 80)
        print("                    PRECISION LANDING TEST CLIENT")
        print("=" * 80)
        print()
        
        # 无人机状态
        print("DRONE STATUS:")
        print(f"  Position: x={self.drone_pose.pose.position.x:.2f}, "
              f"y={self.drone_pose.pose.position.y:.2f}, "
              f"z={self.drone_pose.pose.position.z:.2f}")
        print(f"  Connected: {self.drone_state.connected}")
        print(f"  Armed: {self.drone_state.armed}")
        print(f"  Mode: {self.drone_state.mode}")
        print()
        
        # 目标检测状态
        print("TARGET DETECTION:")
        if self.target_detected:
            print(f"  Target detected at: ({self.target_position.point.x:.1f}, {self.target_position.point.y:.1f})")
            print(f"  Confidence: {self.target_position.point.z:.2f}")
            
            # 计算像素误差
            center_x, center_y = 320, 240
            error_x = self.target_position.point.x - center_x
            error_y = self.target_position.point.y - center_y
            error_magnitude = (error_x**2 + error_y**2)**0.5
            print(f"  Center error: {error_magnitude:.1f} pixels")
        else:
            print("  No target detected")
        print()
        
        # 降落状态
        print("LANDING STATUS:")
        print(f"  State: {self.landing_state}")
        print(f"  Progress: {self.landing_progress:.1%}")
        print(f"  Precision Controller: {'ENABLED' if self.precision_controller_enabled else 'DISABLED'}")
        print()
        
        # 控制说明
        print("CONTROLS:")
        print("  s - Start landing sequence")
        print("  a - Abort landing")
        print("  e - Enable precision controller")
        print("  d - Disable precision controller")
        print("  r - Reset/Return to IDLE")
        print("  q - Quit")
        print()
        print("Press any key to send command...")
        
    def send_start_command(self):
        """发送开始降落命令"""
        msg = Bool()
        msg.data = True
        self.start_pub.publish(msg)
        rospy.loginfo("Start landing command sent")
        
    def send_abort_command(self):
        """发送中止降落命令"""
        msg = Bool()
        msg.data = True
        self.abort_pub.publish(msg)
        rospy.loginfo("Abort landing command sent")
        
    def send_enable_command(self, enable):
        """发送使能精准控制命令"""
        msg = Bool()
        msg.data = enable
        self.enable_pub.publish(msg)
        self.precision_controller_enabled = enable
        rospy.loginfo(f"Precision controller {'enabled' if enable else 'disabled'}")
        
    def run(self):
        """主循环"""
        rospy.loginfo("Test client started - use keyboard to control")
        
        rate = rospy.Rate(5)  # 5Hz显示更新
        
        try:
            while not rospy.is_shutdown():
                # 显示状态
                self.print_status()
                
                # 检查键盘输入
                key = self.get_key()
                
                if key == 's':
                    self.send_start_command()
                elif key == 'a':
                    self.send_abort_command()
                elif key == 'e':
                    self.send_enable_command(True)
                elif key == 'd':
                    self.send_enable_command(False)
                elif key == 'r':
                    # 重置到IDLE状态
                    self.send_abort_command()
                    rospy.sleep(0.5)
                    self.landing_state = "IDLE"
                elif key == 'q':
                    break
                elif key == '\x03':  # Ctrl+C
                    break
                    
                rate.sleep()
                
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("\nTest client stopped")

if __name__ == '__main__':
    try:
        client = PrecisionLandingTestClient()
        client.run()
    except rospy.ROSInterruptException:
        print("Test client interrupted")
