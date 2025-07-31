#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
from enum import Enum
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from std_msgs.msg import String, Bool, Float64
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL

class LandingState(Enum):
    """降落状态枚举"""
    IDLE = "IDLE"
    SEARCHING = "SEARCHING"
    APPROACHING = "APPROACHING"
    ALIGNING = "ALIGNING"
    DESCENDING = "DESCENDING"
    FINAL_DESCENT = "FINAL_DESCENT"
    LANDED = "LANDED"
    ABORTED = "ABORTED"

class LandingStateMachine:
    """
    降落状态机 - 管理精准降落的整个流程
    """
    
    def __init__(self):
        rospy.init_node('landing_state_machine', anonymous=True)
        
        # 无人机参数
        self.drone_id = 0
        self.drone_type = "iris"
        
        # 状态参数
        self.current_state = LandingState.IDLE
        self.previous_state = LandingState.IDLE
        
        # 控制参数
        self.search_altitude = 10.0      # 搜索高度
        self.approach_altitude = 5.0     # 接近高度
        self.alignment_altitude = 3.0    # 对准高度
        self.final_altitude = 1.0        # 最终下降高度
        self.landing_threshold = 30      # 对准阈值(像素)
        self.search_timeout = 30.0       # 搜索超时时间
        self.approach_timeout = 15.0     # 接近超时时间
        
        # 状态变量
        self.target_detected = False
        self.target_position = PointStamped()
        self.current_pose = PoseStamped()
        self.vehicle_state = State()
        self.altitude = 0.0
        self.state_start_time = rospy.Time.now()
        self.last_target_time = rospy.Time.now()
        
        # 订阅话题
        self.state_sub = rospy.Subscriber(f'/iris_{self.drone_id}/mavros/state', State, self.vehicle_state_callback)
        self.pose_sub = rospy.Subscriber(f'/iris_{self.drone_id}/mavros/local_position/pose', PoseStamped, self.pose_callback)
        
        # 订阅目标检测结果
        self.target_sub_basic = rospy.Subscriber('/landing_target', PointStamped, self.target_callback)
        self.target_sub_camo = rospy.Subscriber('/landing_target_camo', PointStamped, self.target_callback)
        self.target_sub_red = rospy.Subscriber('/landing_target_red', PointStamped, self.target_callback)
        
        # 外部命令订阅
        self.start_landing_sub = rospy.Subscriber('/precision_landing/start', Bool, self.start_landing_callback)
        self.abort_landing_sub = rospy.Subscriber('/precision_landing/abort', Bool, self.abort_landing_callback)
        
        # 发布控制命令
        self.cmd_vel_pub = rospy.Publisher(f'/xtdrone/{self.drone_type}_{self.drone_id}/cmd_vel_flu', Twist, queue_size=1)
        self.cmd_pub = rospy.Publisher(f'/xtdrone/{self.drone_type}_{self.drone_id}/cmd', String, queue_size=1)
        
        # 发布状态信息
        self.state_pub = rospy.Publisher('/precision_landing/state', String, queue_size=1)
        self.enable_pub = rospy.Publisher('/precision_landing/enable', Bool, queue_size=1)
        self.progress_pub = rospy.Publisher('/precision_landing/progress', Float64, queue_size=1)
        
        # 服务客户端
        self.set_mode_client = rospy.ServiceProxy(f'/iris_{self.drone_id}/mavros/set_mode', SetMode)
        self.land_client = rospy.ServiceProxy(f'/iris_{self.drone_id}/mavros/cmd/land', CommandTOL)
        
        # 控制循环
        self.rate = rospy.Rate(10)  # 10Hz
        
        rospy.loginfo("Landing State Machine initialized")
        
    def vehicle_state_callback(self, msg):
        """飞行器状态回调"""
        self.vehicle_state = msg
        
    def pose_callback(self, msg):
        """位置回调"""
        self.current_pose = msg
        self.altitude = msg.pose.position.z
        
    def target_callback(self, msg):
        """目标检测回调"""
        self.target_position = msg
        self.target_detected = True
        self.last_target_time = rospy.Time.now()
        
    def start_landing_callback(self, msg):
        """开始降落回调"""
        if msg.data and self.current_state == LandingState.IDLE:
            self.transition_to_state(LandingState.SEARCHING)
            rospy.loginfo("Landing sequence initiated")
            
    def abort_landing_callback(self, msg):
        """中止降落回调"""
        if msg.data:
            self.transition_to_state(LandingState.ABORTED)
            rospy.logwarn("Landing sequence aborted")
            
    def transition_to_state(self, new_state):
        """状态转换"""
        if self.current_state != new_state:
            rospy.loginfo(f"State transition: {self.current_state.value} -> {new_state.value}")
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_start_time = rospy.Time.now()
            
            # 发布状态
            state_msg = String()
            state_msg.data = new_state.value
            self.state_pub.publish(state_msg)
            
    def get_state_duration(self):
        """获取当前状态持续时间"""
        return (rospy.Time.now() - self.state_start_time).to_sec()
        
    def get_target_age(self):
        """获取目标检测数据的年龄"""
        return (rospy.Time.now() - self.last_target_time).to_sec()
        
    def publish_progress(self):
        """发布降落进度"""
        if self.current_state == LandingState.IDLE:
            progress = 0.0
        elif self.current_state == LandingState.SEARCHING:
            progress = 0.1
        elif self.current_state == LandingState.APPROACHING:
            progress = 0.3
        elif self.current_state == LandingState.ALIGNING:
            progress = 0.5
        elif self.current_state == LandingState.DESCENDING:
            # 基于高度计算进度
            if self.altitude > self.final_altitude:
                progress = 0.7 + 0.2 * (self.alignment_altitude - self.altitude) / (self.alignment_altitude - self.final_altitude)
            else:
                progress = 0.9
        elif self.current_state == LandingState.FINAL_DESCENT:
            progress = 0.95
        elif self.current_state == LandingState.LANDED:
            progress = 1.0
        else:
            progress = 0.0
            
        progress_msg = Float64()
        progress_msg.data = progress
        self.progress_pub.publish(progress_msg)
        
    def control_searching_state(self):
        """搜索状态控制"""
        # 控制无人机到搜索高度
        if abs(self.altitude - self.search_altitude) > 0.5:
            vel_z = 0.5 if self.altitude < self.search_altitude else -0.5
            self.publish_velocity_command(0, 0, vel_z, 0)
            return
            
        # 搜索模式：缓慢螺旋搜索
        duration = self.get_state_duration()
        search_radius = 2.0
        angular_speed = 0.3
        
        vel_x = search_radius * angular_speed * math.cos(angular_speed * duration)
        vel_y = search_radius * angular_speed * math.sin(angular_speed * duration)
        
        self.publish_velocity_command(vel_x, vel_y, 0, angular_speed)
        
        # 检查是否找到目标
        if self.target_detected and self.get_target_age() < 1.0:
            self.transition_to_state(LandingState.APPROACHING)
            
        # 搜索超时
        elif duration > self.search_timeout:
            rospy.logwarn("Search timeout, aborting landing")
            self.transition_to_state(LandingState.ABORTED)
            
    def control_approaching_state(self):
        """接近状态控制"""
        # 检查目标是否丢失
        if not self.target_detected or self.get_target_age() > 2.0:
            rospy.logwarn("Target lost during approach, returning to search")
            self.transition_to_state(LandingState.SEARCHING)
            return
            
        # 控制高度到接近高度
        if abs(self.altitude - self.approach_altitude) > 0.5:
            vel_z = 0.3 if self.altitude < self.approach_altitude else -0.3
            self.publish_velocity_command(0, 0, vel_z, 0)
            return
            
        # 粗略对准目标
        pixel_error_x = self.target_position.point.x - 320  # 假设图像宽度640
        pixel_error_y = self.target_position.point.y - 240  # 假设图像高度480
        
        vel_x = -pixel_error_y * 0.003  # 简单比例控制
        vel_y = -pixel_error_x * 0.003
        
        self.publish_velocity_command(vel_x, vel_y, 0, 0)
        
        # 检查是否可以进入对准状态
        error_magnitude = math.sqrt(pixel_error_x**2 + pixel_error_y**2)
        if error_magnitude < 100:  # 粗略对准阈值
            self.transition_to_state(LandingState.ALIGNING)
            
        # 接近超时
        if self.get_state_duration() > self.approach_timeout:
            rospy.logwarn("Approach timeout, returning to search")
            self.transition_to_state(LandingState.SEARCHING)
            
    def control_aligning_state(self):
        """对准状态控制"""
        # 检查目标是否丢失
        if not self.target_detected or self.get_target_age() > 1.0:
            rospy.logwarn("Target lost during alignment, returning to approach")
            self.transition_to_state(LandingState.APPROACHING)
            return
            
        # 控制高度到对准高度
        if abs(self.altitude - self.alignment_altitude) > 0.3:
            vel_z = 0.2 if self.altitude < self.alignment_altitude else -0.2
            self.publish_velocity_command(0, 0, vel_z, 0)
            return
            
        # 启用精准降落控制器
        enable_msg = Bool()
        enable_msg.data = True
        self.enable_pub.publish(enable_msg)
        
        # 检查对准精度
        pixel_error_x = self.target_position.point.x - 320
        pixel_error_y = self.target_position.point.y - 240
        error_magnitude = math.sqrt(pixel_error_x**2 + pixel_error_y**2)
        
        if error_magnitude < self.landing_threshold:
            self.transition_to_state(LandingState.DESCENDING)
            
    def control_descending_state(self):
        """下降状态控制"""
        # 检查目标是否丢失
        if not self.target_detected or self.get_target_age() > 1.0:
            rospy.logwarn("Target lost during descent, aborting")
            self.transition_to_state(LandingState.ABORTED)
            return
            
        # 精准降落控制器已经在工作
        # 只需要监控进度
        if self.altitude < self.final_altitude:
            self.transition_to_state(LandingState.FINAL_DESCENT)
            
    def control_final_descent_state(self):
        """最终下降状态控制"""
        # 禁用精准降落控制器，使用内置降落
        enable_msg = Bool()
        enable_msg.data = False
        self.enable_pub.publish(enable_msg)
        
        # 执行最终降落
        cmd_msg = String()
        cmd_msg.data = "AUTO.LAND"
        self.cmd_pub.publish(cmd_msg)
        
        # 检查是否着陆
        if self.altitude < 0.2 or not self.vehicle_state.armed:
            self.transition_to_state(LandingState.LANDED)
            
    def control_aborted_state(self):
        """中止状态控制"""
        # 禁用精准降落控制器
        enable_msg = Bool()
        enable_msg.data = False
        self.enable_pub.publish(enable_msg)
        
        # 悬停
        self.publish_velocity_command(0, 0, 0, 0)
        
        # 可以手动重新开始或者RTL
        rospy.logwarn("Landing aborted - awaiting manual intervention")
        
    def control_landed_state(self):
        """着陆状态控制"""
        # 禁用精准降落控制器
        enable_msg = Bool()
        enable_msg.data = False
        self.enable_pub.publish(enable_msg)
        
        rospy.loginfo("Landing completed successfully!")
        
    def publish_velocity_command(self, vel_x, vel_y, vel_z, vel_yaw):
        """发布速度命令"""
        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = vel_y
        twist.linear.z = vel_z
        twist.angular.z = vel_yaw
        
        self.cmd_vel_pub.publish(twist)
        
    def run(self):
        """主循环"""
        rospy.loginfo("Landing State Machine started")
        
        while not rospy.is_shutdown():
            # 发布进度
            self.publish_progress()
            
            # 状态机控制逻辑
            if self.current_state == LandingState.IDLE:
                pass  # 等待外部命令
                
            elif self.current_state == LandingState.SEARCHING:
                self.control_searching_state()
                
            elif self.current_state == LandingState.APPROACHING:
                self.control_approaching_state()
                
            elif self.current_state == LandingState.ALIGNING:
                self.control_aligning_state()
                
            elif self.current_state == LandingState.DESCENDING:
                self.control_descending_state()
                
            elif self.current_state == LandingState.FINAL_DESCENT:
                self.control_final_descent_state()
                
            elif self.current_state == LandingState.ABORTED:
                self.control_aborted_state()
                
            elif self.current_state == LandingState.LANDED:
                self.control_landed_state()
                
            self.rate.sleep()
            
        rospy.loginfo("Landing State Machine stopped")

if __name__ == '__main__':
    try:
        state_machine = LandingStateMachine()
        state_machine.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Landing State Machine interrupted")
