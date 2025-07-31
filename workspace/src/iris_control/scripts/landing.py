#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
视觉降落控制器 - 增强版PID控制
通过下视摄像头获取降落平台的视觉信息，控制无人机精确降落到目标位置
支持多种降落目标检测算法：/landing_target_camo 和 /landing_target_red

PID控制增强功能：
1. 独立的X、Y、Z轴PID控制器
2. 积分项防饱和处理
3. 输出限幅保护
4. 动态参数调整
5. PID调试信息输出
6. 状态切换时PID重置

使用方法：
rosrun iris_control landing.py [target_topic]
例如：rosrun iris_control landing.py landing_target_red
"""

import rospy
import math
import numpy as np
import sys
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from std_msgs.msg import String, Bool, Float64
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

class PIDController:
    """
    PID控制器类
    """
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, output_limits=(-1.0, 1.0), windup_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.windup_limit = windup_limit
        
        # 内部状态
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
        
    def update(self, error, current_time=None):
        """
        更新PID控制器
        Args:
            error: 当前误差
            current_time: 当前时间戳，如果为None则使用rospy.Time.now()
        Returns:
            控制输出
        """
        if current_time is None:
            current_time = rospy.Time.now()
            
        if self.prev_time is None:
            self.prev_time = current_time
            dt = 0.05  # 默认时间间隔
        else:
            dt = (current_time - self.prev_time).to_sec()
            if dt <= 0:
                dt = 0.05
                
        # 比例项
        proportional = self.kp * error
        
        # 积分项
        self.integral += error * dt
        # 积分限幅防止积分饱和
        self.integral = max(-self.windup_limit, min(self.windup_limit, self.integral))
        integral = self.ki * self.integral
        
        # 微分项
        if dt > 0:
            derivative = self.kd * (error - self.prev_error) / dt
        else:
            derivative = 0.0
            
        # 总输出
        output = proportional + integral + derivative
        
        # 输出限幅
        output = max(self.output_limits[0], min(self.output_limits[1], output))
        
        # 更新历史值
        self.prev_error = error
        self.prev_time = current_time
        
        return output
    
    def reset(self):
        """重置PID控制器"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

class VisualLandingController:
    """
    基于视觉的精确降落控制器
    """
    
    def __init__(self, target_topic="landing_target_camo"):
        rospy.init_node('visual_landing_controller', anonymous=True)
        
        # 解析命令行参数
        if len(sys.argv) > 1:
            target_topic = sys.argv[1]
        
        # 无人机参数
        self.drone_id = 0
        self.drone_type = "iris"
        
        # 相机参数
        self.image_width = 640
        self.image_height = 480
        self.camera_center_x = self.image_width / 2
        self.camera_center_y = self.image_height / 2
        
        # PID控制参数
        # X轴PID参数
        self.kp_x = 0.8      # X轴比例增益
        self.ki_x = 0.1      # X轴积分增益
        self.kd_x = 0.15     # X轴微分增益
        
        # Y轴PID参数
        self.kp_y = 0.8      # Y轴比例增益
        self.ki_y = 0.1      # Y轴积分增益
        self.kd_y = 0.15     # Y轴微分增益
        
        # Z轴PID参数
        self.kp_z = 0.6      # Z轴比例增益
        self.ki_z = 0.05     # Z轴积分增益
        self.kd_z = 0.12     # Z轴微分增益
        
        # 降落参数
        self.landing_threshold = 30    # 像素误差阈值(当误差小于此值时开始降落)
        self.min_altitude = 0.8        # 最小安全高度
        self.descent_rate = 0.25       # 降落速率 m/s
        self.max_vel_xy = 1.5          # XY方向最大速度
        self.max_vel_z = 0.8           # Z方向最大速度
        self.target_timeout = 3.0      # 目标丢失超时时间(秒)
        
        # 状态变量
        self.current_state = State()
        self.current_pose = PoseStamped()
        self.target_detected = False
        self.target_position = PointStamped()
        self.landing_enabled = False
        self.altitude = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_z = 0.0
        self.last_target_time = rospy.Time.now()
        
        # PID控制器实例
        self.pid_x = PIDController(
            kp=self.kp_x, ki=self.ki_x, kd=self.kd_x,
            output_limits=(-self.max_vel_xy, self.max_vel_xy),
            windup_limit=1.0
        )
        self.pid_y = PIDController(
            kp=self.kp_y, ki=self.ki_y, kd=self.kd_y,
            output_limits=(-self.max_vel_xy, self.max_vel_xy),
            windup_limit=1.0
        )
        self.pid_z = PIDController(
            kp=self.kp_z, ki=self.ki_z, kd=self.kd_z,
            output_limits=(-self.max_vel_z, self.max_vel_z),
            windup_limit=0.5
        )
        
        # 降落状态机
        self.STATES = {
            'SEARCHING': 0,    # 搜索目标
            'TRACKING': 1,     # 跟踪目标
            'DESCENDING': 2,   # 下降中
            'LANDING': 3,      # 执行降落
            'LANDED': 4        # 已降落
        }
        self.current_landing_state = self.STATES['SEARCHING']
        
        # 订阅话题
        self.setup_subscribers(target_topic)
        
        # 发布话题
        self.setup_publishers()
        
        # 服务客户端
        self.setup_services()
        
        # 控制循环
        self.control_rate = rospy.Rate(20)  # 20Hz控制频率
        
        rospy.loginfo(f"Visual Landing Controller initialized with target topic: /{target_topic}")
        
    def setup_subscribers(self, target_topic):
        """设置订阅者"""
        # 无人机状态
        self.state_sub = rospy.Subscriber(
            f'/iris_{self.drone_id}/mavros/state', 
            State, 
            self.state_callback
        )
        
        # 位置信息
        self.pose_sub = rospy.Subscriber(
            f'/iris_{self.drone_id}/mavros/local_position/pose', 
            PoseStamped, 
            self.pose_callback
        )
        
        # 降落目标检测结果
        self.target_sub = rospy.Subscriber(
            f'/{target_topic}', 
            PointStamped, 
            self.target_callback
        )
        
        # 降落使能命令
        self.landing_cmd_sub = rospy.Subscriber(
            '/visual_landing/enable', 
            Bool, 
            self.landing_enable_callback
        )
        
    def setup_publishers(self):
        """设置发布者"""
        # 速度控制命令
        self.cmd_vel_pub = rospy.Publisher(
            f'/xtdrone/{self.drone_type}_{self.drone_id}/cmd_vel_flu', 
            Twist, 
            queue_size=1
        )
        
        # 指令发布
        self.cmd_pub = rospy.Publisher(
            f'/xtdrone/{self.drone_type}_{self.drone_id}/cmd', 
            String, 
            queue_size=1
        )
        
        # 状态信息发布
        self.status_pub = rospy.Publisher(
            '/visual_landing/status', 
            String, 
            queue_size=1
        )
        
        # 误差信息发布
        self.error_pub = rospy.Publisher(
            '/visual_landing/error', 
            PointStamped, 
            queue_size=1
        )
        
        # PID调试信息发布
        self.pid_debug_pub = rospy.Publisher(
            '/visual_landing/pid_debug', 
            String, 
            queue_size=1
        )
        
    def setup_services(self):
        """设置服务客户端"""
        self.arming_client = rospy.ServiceProxy(
            f'/iris_{self.drone_id}/mavros/cmd/arming', 
            CommandBool
        )
        
        self.set_mode_client = rospy.ServiceProxy(
            f'/iris_{self.drone_id}/mavros/set_mode', 
            SetMode
        )
        
        self.land_client = rospy.ServiceProxy(
            f'/iris_{self.drone_id}/mavros/cmd/land', 
            CommandTOL
        )
        
    def state_callback(self, msg):
        """无人机状态回调"""
        self.current_state = msg
        
    def pose_callback(self, msg):
        """位置回调"""
        self.current_pose = msg
        self.altitude = msg.pose.position.z
        
    def target_callback(self, msg):
        """目标检测回调"""
        self.target_position = msg
        self.target_detected = True
        self.last_target_time = rospy.Time.now()
        
    def landing_enable_callback(self, msg):
        """降落使能回调"""
        self.landing_enabled = msg.data
        if self.landing_enabled:
            self.current_landing_state = self.STATES['SEARCHING']
            # 重置PID控制器
            self.pid_x.reset()
            self.pid_y.reset()
            self.pid_z.reset()
            rospy.loginfo("Visual landing ENABLED - PID controllers reset")
        else:
            self.current_landing_state = self.STATES['SEARCHING']
            rospy.loginfo("Visual landing DISABLED")
    
    def calculate_pixel_error(self):
        """计算像素误差"""
        if not self.target_detected:
            return None, None, None
            
        # 计算目标中心与图像中心的偏差
        error_x = self.target_position.point.x - self.camera_center_x
        error_y = self.target_position.point.y - self.camera_center_y
        
        # 计算总误差距离
        error_distance = math.sqrt(error_x**2 + error_y**2)
        
        return error_x, error_y, error_distance
    
    def pixel_to_velocity_pid(self, error_x, error_y):
        """使用PID控制器将像素误差转换为速度命令"""
        current_time = rospy.Time.now()
        
        # 将像素误差归一化到 [-1, 1] 范围
        norm_error_x = error_x / (self.image_width / 2)
        norm_error_y = error_y / (self.image_height / 2)
        
        # 使用PID控制器计算速度
        vel_x = -self.pid_x.update(norm_error_x, current_time)
        vel_y = self.pid_y.update(norm_error_y, current_time)
        
        return vel_x, vel_y
    
    def altitude_to_velocity_pid(self, target_altitude):
        """使用PID控制器将高度误差转换为垂直速度命令"""
        current_time = rospy.Time.now()
        altitude_error = self.altitude - target_altitude
        
        # 使用PID控制器计算垂直速度
        vel_z = -self.pid_z.update(altitude_error, current_time)
        
        return vel_z
    
    def check_target_timeout(self):
        """检查目标丢失超时"""
        current_time = rospy.Time.now()
        if (current_time - self.last_target_time).to_sec() > self.target_timeout:
            self.target_detected = False
            return True
        return False
    
    def publish_status(self, status_msg):
        """发布状态信息"""
        status = String()
        status.data = status_msg
        self.status_pub.publish(status)
        
    def publish_error(self, error_x, error_y, error_distance):
        """发布误差信息"""
        error_msg = PointStamped()
        error_msg.header.stamp = rospy.Time.now()
        error_msg.point.x = error_x
        error_msg.point.y = error_y
        error_msg.point.z = error_distance
        self.error_pub.publish(error_msg)
    
    def publish_pid_debug(self, vel_x, vel_y, vel_z, error_x, error_y):
        """发布PID调试信息"""
        debug_msg = String()
        debug_msg.data = f"PID_Debug: vel_x={vel_x:.3f}, vel_y={vel_y:.3f}, vel_z={vel_z:.3f}, " \
                        f"error_x={error_x:.1f}px, error_y={error_y:.1f}px, alt={self.altitude:.2f}m"
        self.pid_debug_pub.publish(debug_msg)
    
    def update_pid_parameters(self, kp_x=None, ki_x=None, kd_x=None, 
                              kp_y=None, ki_y=None, kd_y=None,
                              kp_z=None, ki_z=None, kd_z=None):
        """动态更新PID参数"""
        if kp_x is not None:
            self.pid_x.kp = kp_x
        if ki_x is not None:
            self.pid_x.ki = ki_x
        if kd_x is not None:
            self.pid_x.kd = kd_x
            
        if kp_y is not None:
            self.pid_y.kp = kp_y
        if ki_y is not None:
            self.pid_y.ki = ki_y
        if kd_y is not None:
            self.pid_y.kd = kd_y
            
        if kp_z is not None:
            self.pid_z.kp = kp_z
        if ki_z is not None:
            self.pid_z.ki = ki_z
        if kd_z is not None:
            self.pid_z.kd = kd_z
            
        rospy.loginfo("PID parameters updated")
    
    def execute_landing_state_machine(self):
        """执行降落状态机"""
        if not self.landing_enabled:
            return
            
        # 检查目标超时
        target_timeout = self.check_target_timeout()
        
        # 计算像素误差
        error_x, error_y, error_distance = self.calculate_pixel_error()
        
        # 创建速度命令
        cmd_vel = Twist()
        
        # 状态机逻辑
        if self.current_landing_state == self.STATES['SEARCHING']:
            self.publish_status("SEARCHING for target")
            
            if self.target_detected and not target_timeout:
                self.current_landing_state = self.STATES['TRACKING']
                rospy.loginfo("Target detected, switching to TRACKING")
                
        elif self.current_landing_state == self.STATES['TRACKING']:
            self.publish_status("TRACKING target")
            
            if target_timeout:
                self.current_landing_state = self.STATES['SEARCHING']
                # 重置PID控制器
                self.pid_x.reset()
                self.pid_y.reset()
                rospy.logwarn("Target lost, switching to SEARCHING")
                return
                
            if error_distance is not None:
                # 发布误差信息
                self.publish_error(error_x, error_y, error_distance)
                
                # 使用PID控制器计算XY方向速度
                vel_x, vel_y = self.pixel_to_velocity_pid(error_x, error_y)
                cmd_vel.linear.x = vel_x
                cmd_vel.linear.y = vel_y
                
                # 发布PID调试信息
                self.publish_pid_debug(vel_x, vel_y, 0.0, error_x, error_y)
                
                # 判断是否可以开始下降
                if error_distance < self.landing_threshold and self.altitude > self.min_altitude:
                    self.current_landing_state = self.STATES['DESCENDING']
                    rospy.loginfo("Target centered, switching to DESCENDING")
                    
        elif self.current_landing_state == self.STATES['DESCENDING']:
            self.publish_status("DESCENDING to target")
            
            if target_timeout:
                self.current_landing_state = self.STATES['SEARCHING']
                # 重置PID控制器
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_z.reset()
                rospy.logwarn("Target lost during descent, switching to SEARCHING")
                return
                
            if error_distance is not None:
                # 发布误差信息
                self.publish_error(error_x, error_y, error_distance)
                
                # 使用PID控制器计算XY方向速度（下降时降低响应强度）
                vel_x, vel_y = self.pixel_to_velocity_pid(error_x, error_y)
                cmd_vel.linear.x = vel_x * 0.6  # 下降时减少XY方向的修正幅度
                cmd_vel.linear.y = vel_y * 0.6
                
                # 使用PID控制器控制下降速度，目标高度逐渐降低
                target_altitude = max(self.min_altitude, self.altitude - 0.1)  # 逐步下降
                vel_z = self.altitude_to_velocity_pid(target_altitude)
                
                # 限制下降速度
                vel_z = max(-self.descent_rate, min(0.0, vel_z))
                cmd_vel.linear.z = vel_z
                
                # 发布PID调试信息
                self.publish_pid_debug(cmd_vel.linear.x, cmd_vel.linear.y, vel_z, error_x, error_y)
                
                # 判断是否需要最终降落
                if self.altitude <= self.min_altitude:
                    self.current_landing_state = self.STATES['LANDING']
                    rospy.loginfo("Reached minimum altitude, executing final LANDING")
                    
        elif self.current_landing_state == self.STATES['LANDING']:
            self.publish_status("LANDING")
            
            # 执行最终降落
            try:
                land_response = self.land_client(altitude=0)
                if land_response.success:
                    self.current_landing_state = self.STATES['LANDED']
                    rospy.loginfo("Landing command sent successfully")
                else:
                    rospy.logerr("Landing command failed")
            except rospy.ServiceException as e:
                rospy.logerr(f"Landing service call failed: {e}")
                
        elif self.current_landing_state == self.STATES['LANDED']:
            self.publish_status("LANDED")
            self.landing_enabled = False
            return
            
        # 发布速度命令
        if self.current_landing_state in [self.STATES['TRACKING'], self.STATES['DESCENDING']]:
            self.cmd_vel_pub.publish(cmd_vel)
            # 发布PID调试信息
            self.publish_pid_debug(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z, error_x, error_y)
    
    def run(self):
        """主控制循环"""
        rospy.loginfo("Visual Landing Controller started")
        
        while not rospy.is_shutdown():
            try:
                self.execute_landing_state_machine()
                self.control_rate.sleep()
            except rospy.ROSInterruptException:
                break
            except Exception as e:
                rospy.logerr(f"Error in control loop: {e}")
                
        rospy.loginfo("Visual Landing Controller stopped")

def main():
    """主函数"""
    try:
        # 获取命令行参数
        target_topic = "landing_target_camo"  # 默认话题
        if len(sys.argv) > 1:
            target_topic = sys.argv[1]
            
        # 创建控制器
        controller = VisualLandingController(target_topic)
        
        # 运行控制器
        controller.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Visual Landing Controller interrupted")
    except Exception as e:
        rospy.logerr(f"Failed to start Visual Landing Controller: {e}")

if __name__ == '__main__':
    main()