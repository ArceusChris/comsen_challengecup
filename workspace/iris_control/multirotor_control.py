#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
多旋翼无人机控制节点 - 集成视觉降落功能

该模块提供了完整的多旋翼无人机控制功能，包括：
1. 基础飞行控制（起飞、移动、悬停、降落）
2. 路径规划和导航
3. 视觉精确降落控制
4. PID控制算法
5. 任务执行

视觉降落功能支持多种目标类型：
- landing_target_camo: 迷彩降落目标
- landing_target_red: 红色降落目标
- landing_target_custom: 自定义降落目标

使用方法：
1. 基础模式：
   python3 multirotor_control.py <multirotor_type> <multirotor_id> <control_type>

2. 视觉降落演示：
   python3 multirotor_control.py <multirotor_type> <multirotor_id> <control_type> demo_visual_landing [target_type]

3. 在代码中使用视觉降落：
   multirotor_control.visual_landing(target_type="landing_target_camo")
   
   或者在execute_mission中启用：
   multirotor_control.execute_mission(use_visual_landing=True, visual_target_type="landing_target_red")
"""

from drone_control import DroneController
import sys
import math
import time
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist, PointStamped, Pose
from std_msgs.msg import Int8, String, Bool, Float64
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from vtol_Astar import VTOLAstarPlanner
MAX_LINEAR = 20.0

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

class MultirotorControl:
    def __init__(self, controller):
        self.controller = controller
        self.MAX_LINEAR = MAX_LINEAR
        
        # 视觉降落相关参数
        self.image_width = 640
        self.image_height = 480
        self.camera_center_x = self.image_width / 2
        self.camera_center_y = self.image_height / 2
        
        # PID控制参数
        self.kp_x = 0.8      # X轴比例增益
        self.ki_x = 0.1      # X轴积分增益
        self.kd_x = 0.15     # X轴微分增益
        
        self.kp_y = 0.8      # Y轴比例增益
        self.ki_y = 0.1      # Y轴积分增益
        self.kd_y = 0.15     # Y轴微分增益
        
        self.kp_z = 0.6      # Z轴比例增益
        self.ki_z = 0.05     # Z轴积分增益
        self.kd_z = 0.12     # Z轴微分增益
        
        # 降落参数
        self.landing_threshold = 30    # 像素误差阈值
        self.min_altitude = 0.8        # 最小安全高度
        self.descent_rate = 0.25       # 降落速率 m/s
        self.max_vel_xy = 1.5          # XY方向最大速度
        self.max_vel_z = 0.8           # Z方向最大速度
        self.target_timeout = 3.0      # 目标丢失超时时间(秒)
        
        # 视觉降落状态变量
        self.target_detected = False
        self.target_position = PointStamped()
        self.last_target_time = rospy.Time.now()
        
        # 降落状态机
        self.LANDING_STATES = {
            'SEARCHING': 0,    # 搜索目标
            'TRACKING': 1,     # 跟踪目标
            'DESCENDING': 2,   # 下降中
            'LANDING': 3,      # 执行降落
            'LANDED': 4        # 已降落
        }
        self.current_landing_state = self.LANDING_STATES['SEARCHING']
        
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
        
        # 视觉降落相关话题订阅器
        self.target_sub = None
        self.landing_enabled = False

    def takeoff(self, altitude=20):
        """无人机起飞到指定高度"""
        print(f"开始起飞到{altitude}米高度...")
        self.controller.auto_takeoff_sequence()
        
        flag = True  # 修复：添加缺失的flag变量
        last_check_time = time.time()
        check_interval = 0.5  # 0.5秒检查一次高度
        rate = rospy.Rate(10)  # 10Hz循环频率

        while flag and not rospy.is_shutdown():
            current_time = time.time()

            if current_time - last_check_time >= check_interval:
                position = self.controller.get_position_xyz()
                if position is None:
                    print("等待位置信息...")
                else:  
                    x, y, z = position
                    print(f"当前高度: {z:.2f}m")
                    if z > altitude:
                        self.controller.current_twist.linear.z = 0.0
                        self.controller.set_hover_mode()
                        print("达到目标高度，准备水平移动")
                        flag = False
                
                last_check_time = current_time
            
            rate.sleep()  
        
        return True


    def land(self, altitude=0.65):
        """无人机基础降落到指定高度（非视觉降落）"""
        print(f"开始基础降落到{altitude}米高度...")
        self.controller.auto_landing_sequence()
        
        flag = True
        last_check_time = time.time()
        check_interval = 0.5  # 0.5秒检查一次高度
        rate = rospy.Rate(10)  # 10Hz循环频率       
        
        # 添加速度控制变量
        last_update_time = time.time()
        update_interval = 0.5  # 0.5秒更新一次速度
    
        while flag and not rospy.is_shutdown():
            current_time = time.time()

            # 检查是否到了更新速度的时间
            if current_time - last_update_time >= update_interval:
                position = self.controller.get_position_xyz()
                if position is None:
                    print("等待位置信息...")
                else:  
                    x, y, z = position
                    
                    # 计算当前高度与目标高度的差值
                    height_diff = z - altitude
                    print(f"当前高度: {z:.2f}m, 目标高度: {altitude:.2f}m, 高度差: {height_diff:.2f}m")
                    
                    if height_diff <= 0.1:  # 接近目标高度
                        self.controller.current_twist.linear.z = 0.0
                        self.controller.set_hover_mode()
                        print("达到目标降落高度，准备悬停")
                        flag = False
                    else:
                        # 应用速度衰减函数，与go_to_position中相同的逻辑
                        if height_diff > 0.01:
                            # 使用1米作为开始减速的距离
                            speed_factor = min(1.0, height_diff / 1.0)  # 1米内开始减速
                            if speed_factor != 1.0:
                                speed_factor = 1.0 / math.exp(1.0 / speed_factor)  # 使用指数衰减函数
                            
                            # 设置下降速度（负值表示下降）
                            descent_speed = -self.MAX_LINEAR * speed_factor * 0.5  # 降落速度为最大速度的一半
                            self.controller.current_twist.linear.z = descent_speed
                            
                            print(f"更新下降速度: vz={descent_speed:.2f}, 速度系数: {speed_factor:.3f}")
                        else:
                            self.controller.current_twist.linear.z = 0.0
                
                last_update_time = current_time
            
            # 兼容原有的时间检查逻辑（保持0.5秒的状态输出）
            if current_time - last_check_time >= check_interval:
                last_check_time = current_time
            
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                print("收到ROS关闭信号，退出降落循环")
                break
    
        return True

    def go_to_position(self, target_position, stop=True):
        """
        控制无人机移动到指定位置
        该函数分为2种情况：
        1.无人机在悬停
        2.无人机正在运动
        target_position应该为[x, y]
        """
        if not self.controller.is_armed:
            self.controller.arm()

        pose_info, _ = self.controller.get_current_position()
        if pose_info is None:
            print("无法获取当前位置信息")
            return False
        
        position = self.controller.get_position_xyz()
        if position is None:
            print("无法获取位置坐标")
            return False
        
        x, y, z = position
        dx = target_position[0] - x
        dy = target_position[1] - y
        # 避免除零错误
        distance = math.sqrt(dx * dx + dy * dy + 1e-6)  # 添加一个小值以避免除零错误
        if distance < 0.1:
            print("已接近目标位置")
            return True
        
        print(f'当前无人机位置: x={x}, y={y}, z={z}')
        self.controller.current_twist.linear.x = self.MAX_LINEAR * dx / distance
        self.controller.current_twist.linear.y = self.MAX_LINEAR * dy / distance
        self.controller.current_twist.linear.z = 0.0
        self.controller.current_twist.angular.z = 0.0

        done = False
        
        # 添加计时器变量
        last_update_time = time.time()
        update_interval = 0.5  # 0.5秒更新一次速度
        rate = rospy.Rate(20)

        while not done and not rospy.is_shutdown():
            current_time = time.time()
            
            # 检查是否到了更新速度的时间
            if current_time - last_update_time >= update_interval:
                position = self.controller.get_position_xyz()
                if position is None:
                    print("位置信息丢失")
                    break
                    
                x, y, z = position
                dx = target_position[0] - x
                dy = target_position[1] - y
                
                # 计算当前距离
                distance = math.sqrt(dx * dx + dy * dy)
                print(f'当前无人机位置: x={x:.2f}, y={y:.2f}, z={z:.2f}, 距离目标: {distance:.2f}m')
                
                # 检查是否到达目标
                if distance < 1.0 and stop:  # 修正到达判断逻辑
                    print(f"无人机到达目标位置: x={x:.2f}, y={y:.2f}, z={z:.2f}")
                    self.controller.current_twist.linear.x = 0.0
                    self.controller.current_twist.linear.y = 0.0
                    self.controller.current_twist.linear.z = 0.0
                    self.controller.current_twist.angular.z = 0.0
                    done = True  # 停止运动

                elif distance < 3.0 and not stop:
                    done = True  # 如果不需要停止，且距离小于3米，则认为到达目标
                else:
                    # 更新速度命令（避免除零错误）
                    if distance > 0.01:
                        speed_factor = min(1.0, distance / 5.0)  # 5米内开始减速
                        if speed_factor != 1.0:
                            speed_factor = 1.0 / math.exp(1.0 / speed_factor)  # 使用指数衰减函数
                        self.controller.current_twist.linear.x = self.MAX_LINEAR * speed_factor * dx / distance
                        self.controller.current_twist.linear.y = self.MAX_LINEAR * speed_factor * dy / distance
                        print(f"更新速度: vx={self.controller.current_twist.linear.x:.2f}, vy={self.controller.current_twist.linear.y:.2f}")
                    else:
                        self.controller.current_twist.linear.x = 0.0
                        self.controller.current_twist.linear.y = 0.0
                
                # 更新最后一次更新时间
                last_update_time = current_time
            
            # 控制循环频率
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                print("收到ROS关闭信号，退出位置控制循环")
                break
        
        return done

    def go_to_target(self, target_position):
        """移动到目标位置的便捷方法"""
        print(f"开始移动到目标位置 {target_position}...")
        success = self.go_to_position(target_position)
        self.controller.set_hover_mode()
        if success:
            self.controller.current_iris_status.data += 1
        return success

    def execute_mission(self, altitude_high=20, altitude_low=0.65, target_position=[1495, -105], use_visual_landing=False, visual_target_type="landing_target_camo"):
        """执行完整任务：起飞 -> 移动到目标 -> 悬停 -> 降落
        Args:
            altitude_high: 起飞高度
            altitude_low: 降落高度（仅在非视觉降落时使用）
            target_position: 目标位置
            use_visual_landing: 是否使用视觉降落
            visual_target_type: 视觉降落目标类型
        """
        print("开始执行任务...")
        
        # 步骤1：起飞
        takeoff_success = self.takeoff(altitude_high)
        if not takeoff_success:
            print("起飞失败")
            return False
        
        # 步骤2：移动到目标位置
        move_success = self.go_to_target(target_position)

        # 步骤3：降落
        if use_visual_landing:
            print("使用视觉降落")
            landing_success = self.visual_landing(target_type=visual_target_type)
        else:
            print("使用基础降落")
            landing_success = self.land(altitude_low)

        self.controller.set_hover_mode()
        if not landing_success:
            print("降落失败")
            return False
        else:
            print("任务执行成功")
            return True

    def visual_landing(self, target_type="landing_target_camo"):
        """
        基于视觉的精确降落控制器
        Args:
            target_type: 降落目标类型，可选值：
                        "landing_target_camo" - 迷彩降落目标
                        "landing_target_red" - 红色降落目标
                        "landing_target_custom" - 自定义降落目标
        """
        print(f"开始视觉降落，目标类型: {target_type}")
        
        # 设置目标话题订阅
        self._setup_visual_landing_subscriber(target_type)
        
        # 重置PID控制器和状态
        self._reset_landing_state()
        
        # 启用视觉降落
        self.landing_enabled = True
        
        # 主控制循环
        rate = rospy.Rate(20)  # 20Hz控制频率
        
        while self.landing_enabled and not rospy.is_shutdown():
            try:
                self._execute_visual_landing_state_machine()
                
                # 检查是否完成降落
                if self.current_landing_state == self.LANDING_STATES['LANDED']:
                    print("视觉降落完成")
                    break
                    
                rate.sleep()
            except rospy.ROSInterruptException:
                print("收到ROS关闭信号，退出视觉降落")
                break
            except Exception as e:
                print(f"视觉降落过程中出现错误: {e}")
                break
        
        # 清理订阅器
        if self.target_sub:
            self.target_sub.unregister()
            self.target_sub = None
        
        return self.current_landing_state == self.LANDING_STATES['LANDED']
    
    def _setup_visual_landing_subscriber(self, target_type):
        """设置视觉降落目标话题订阅器"""
        if self.target_sub:
            self.target_sub.unregister()
        
        self.target_sub = rospy.Subscriber(
            f'/{target_type}', 
            PointStamped, 
            self._target_callback
        )
        print(f"已订阅视觉目标话题: /{target_type}")
    
    def _target_callback(self, msg):
        """目标检测回调函数"""
        self.target_position = msg
        self.target_detected = True
        self.last_target_time = rospy.Time.now()
    
    def _reset_landing_state(self):
        """重置降落状态和PID控制器"""
        self.current_landing_state = self.LANDING_STATES['SEARCHING']
        self.target_detected = False
        self.pid_x.reset()
        self.pid_y.reset()
        self.pid_z.reset()
        print("视觉降落状态已重置")
    
    def _calculate_pixel_error(self):
        """计算像素误差"""
        if not self.target_detected:
            return None, None, None
            
        # 计算目标中心与图像中心的偏差
        error_x = self.target_position.point.x - self.camera_center_x
        error_y = self.target_position.point.y - self.camera_center_y
        
        # 计算总误差距离
        error_distance = math.sqrt(error_x**2 + error_y**2)
        
        return error_x, error_y, error_distance
    
    def _pixel_to_velocity_pid(self, error_x, error_y):
        """使用PID控制器将像素误差转换为速度命令"""
        current_time = rospy.Time.now()
        
        # 将像素误差归一化到 [-1, 1] 范围
        norm_error_x = error_x / (self.image_width / 2)
        norm_error_y = error_y / (self.image_height / 2)
        
        # 使用PID控制器计算速度
        vel_x = -self.pid_x.update(norm_error_x, current_time)
        vel_y = self.pid_y.update(norm_error_y, current_time)
        
        return vel_x, vel_y
    
    def _altitude_to_velocity_pid(self, target_altitude):
        """使用PID控制器将高度误差转换为垂直速度命令"""
        current_time = rospy.Time.now()
        position = self.controller.get_position_xyz()
        if position is None:
            return 0.0
        
        current_altitude = position[2]
        altitude_error = current_altitude - target_altitude
        
        # 使用PID控制器计算垂直速度
        vel_z = -self.pid_z.update(altitude_error, current_time)
        
        return vel_z
    
    def _check_target_timeout(self):
        """检查目标丢失超时"""
        current_time = rospy.Time.now()
        if (current_time - self.last_target_time).to_sec() > self.target_timeout:
            self.target_detected = False
            return True
        return False
    
    def _execute_visual_landing_state_machine(self):
        """执行视觉降落状态机"""
        if not self.landing_enabled:
            return
            
        # 检查目标超时
        target_timeout = self._check_target_timeout()
        
        # 计算像素误差
        error_x, error_y, error_distance = self._calculate_pixel_error()
        
        # 获取当前高度
        position = self.controller.get_position_xyz()
        if position is None:
            print("无法获取当前位置信息")
            return
        current_altitude = position[2]
        
        # 状态机逻辑
        if self.current_landing_state == self.LANDING_STATES['SEARCHING']:
            print("搜索降落目标中...")
            
            if self.target_detected and not target_timeout:
                self.current_landing_state = self.LANDING_STATES['TRACKING']
                print("检测到目标，切换到跟踪状态")
                
        elif self.current_landing_state == self.LANDING_STATES['TRACKING']:
            print(f"跟踪目标中，误差距离: {error_distance:.1f}px")
            
            if target_timeout:
                self.current_landing_state = self.LANDING_STATES['SEARCHING']
                self.pid_x.reset()
                self.pid_y.reset()
                print("目标丢失，切换到搜索状态")
                return
                
            if error_distance is not None:
                # 使用PID控制器计算XY方向速度
                vel_x, vel_y = self._pixel_to_velocity_pid(error_x, error_y)
                
                # 应用速度命令
                self.controller.current_twist.linear.x = vel_x
                self.controller.current_twist.linear.y = vel_y
                self.controller.current_twist.linear.z = 0.0
                
                print(f"跟踪调整: vx={vel_x:.3f}, vy={vel_y:.3f}, 误差: x={error_x:.1f}px, y={error_y:.1f}px")
                
                # 判断是否可以开始下降
                if error_distance < self.landing_threshold and current_altitude > self.min_altitude:
                    self.current_landing_state = self.LANDING_STATES['DESCENDING']
                    print("目标居中，开始下降")
                    
        elif self.current_landing_state == self.LANDING_STATES['DESCENDING']:
            print(f"下降中，高度: {current_altitude:.2f}m，目标误差: {error_distance:.1f}px")
            
            if target_timeout:
                self.current_landing_state = self.LANDING_STATES['SEARCHING']
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_z.reset()
                print("下降过程中目标丢失，切换到搜索状态")
                return
                
            if error_distance is not None:
                # 使用PID控制器计算XY方向速度（下降时降低响应强度）
                vel_x, vel_y = self._pixel_to_velocity_pid(error_x, error_y)
                vel_x *= 0.6  # 下降时减少XY方向的修正幅度
                vel_y *= 0.6
                
                # 使用PID控制器控制下降速度
                target_altitude = max(self.min_altitude, current_altitude - 0.1)
                vel_z = self._altitude_to_velocity_pid(target_altitude)
                
                # 限制下降速度
                vel_z = max(-self.descent_rate, min(0.0, vel_z))
                
                # 应用速度命令
                self.controller.current_twist.linear.x = vel_x
                self.controller.current_twist.linear.y = vel_y
                self.controller.current_twist.linear.z = vel_z
                
                print(f"下降调整: vx={vel_x:.3f}, vy={vel_y:.3f}, vz={vel_z:.3f}")
                
                # 判断是否需要最终降落
                if current_altitude <= self.min_altitude:
                    self.current_landing_state = self.LANDING_STATES['LANDING']
                    print("到达最小安全高度，执行最终降落")
                    
        elif self.current_landing_state == self.LANDING_STATES['LANDING']:
            print("执行最终降落")
            
            # 停止所有运动
            self.controller.current_twist.linear.x = 0.0
            self.controller.current_twist.linear.y = 0.0
            self.controller.current_twist.linear.z = 0.0
            
            # 执行自动降落序列
            self.controller.auto_landing_sequence()
            
            # 等待降落完成
            time.sleep(2.0)
            
            self.current_landing_state = self.LANDING_STATES['LANDED']
            print("降落完成")
                
        elif self.current_landing_state == self.LANDING_STATES['LANDED']:
            self.landing_enabled = False
            return

class PoseNode:
    def __init__(self):
        self.first_target_pose = ()
        self.third_target_pose = ()
        self.done = False
        # 危重
        self.first_pose_sub = rospy.Subscriber('/zhihang2025/first_man/pose', Pose, self.first_pose_callback)
        # 健康
        self.third_pose_sub = rospy.Subscriber('/zhihang2025/third_man/pose', Pose, self.third_pose_callback)

        self.vtol_land_sub = rospy.Subscriber('/zhihang2025/vtol_land_sub/done', Int8, self.done_callback)

    def first_pose_callback(self, msg):
        """处理人的位置的信息"""
        position = msg.pose.position
        self.first_target_pose = (position.x, position.y)  # 假设高度为20米
        print(f"接收到第一人的位置信息: x={position.x}, y={position.y}, z={position.z}")

    def third_pose_callback(self, msg):
        """处理第三人的位置的信息"""
        position = msg.pose.position
        self.third_target_pose = (position.x, position.y)  # 假设高度为20米
        print(f"接收到第三人的位置信息: x={position.x}, y={position.y}, z={position.z}")

    def done_callback(self, msg):
        self.done = (msg.data == 5)
        if self.done:
            print("固定翼飞机已降落")
        else:
            print("固定翼飞机还在起飞")

def demo_visual_landing():
    """演示视觉降落功能的示例函数"""
    if len(sys.argv) < 4:
        print("用法: python3 multirotor_control.py <multirotor_type> <multirotor_id> <control_type> [demo_visual_landing]")
        return

    multirotor_type = sys.argv[1]
    multirotor_id = int(sys.argv[2])
    control_type = sys.argv[3]

    # 创建控制器和控制类
    controller = DroneController(multirotor_type, multirotor_id, control_type)
    multirotor_control = MultirotorControl(controller)
    
    print("=== 视觉降落演示 ===")
    
    # 步骤1：起飞到合适高度
    print("1. 起飞到20米高度...")
    multirotor_control.takeoff(altitude=20)
    
    # 步骤2：移动到降落区域上方
    print("2. 移动到降落区域上方...")
    landing_area_position = [0, 0]  # 可以根据需要调整位置
    multirotor_control.go_to_position(landing_area_position, stop=True)
    
    # 步骤3：执行视觉降落
    print("3. 开始视觉降落...")
    
    # 可以选择不同的目标类型：
    # "landing_target_camo" - 迷彩目标
    # "landing_target_red" - 红色目标
    # "landing_target_custom" - 自定义目标
    target_type = "landing_target_camo"  # 默认使用迷彩目标
    
    # 如果命令行提供了目标类型参数
    if len(sys.argv) > 4:
        target_type = sys.argv[4]
    
    success = multirotor_control.visual_landing(target_type=target_type)
    
    if success:
        print("4. 视觉降落成功完成！")
        multirotor_control.controller.current_iris_status.data = 8  # 任务完成状态
    else:
        print("4. 视觉降落失败，执行基础降落...")
        multirotor_control.land(altitude=0.65)
    
    print("=== 视觉降落演示完成 ===")

def main():
    # 检查是否是演示模式
    if len(sys.argv) > 4 and "demo_visual_landing" in sys.argv:
        demo_visual_landing()
        return
    
    if len(sys.argv) < 4:
        print("用法: python3 multirotor_control.py <multirotor_type> <multirotor_id> <control_type>")
        print("视觉降落演示: python3 multirotor_control.py <multirotor_type> <multirotor_id> <control_type> demo_visual_landing [target_type]")
        print("支持的目标类型: landing_target_camo, landing_target_red, landing_target_custom")
        return

    multirotor_type = sys.argv[1]
    multirotor_id = int(sys.argv[2])
    control_type = sys.argv[3]

    # 创建控制器和控制类
    controller = DroneController(multirotor_type, multirotor_id, control_type)
    multirotor_control = MultirotorControl(controller)
    
    node = PoseNode()
    while not node.done:
        rospy.sleep(0.1)

    original_poses = [(2, 3),node.third_target_pose[-1], node.first_target_pose[-1] , (0, 1)]
    path_planner = VTOLAstarPlanner()

    multirotor_control.controller.current_iris_status.data = 1
    multirotor_control.takeoff(altitude=20)

    list_1 = path_planner.plan_path(original_poses[0], original_poses[1])
    del list_1[0]
    del list_1[-1]
    multirotor_control.controller.current_iris_status.data = 2
    for pose in list_1:
        print(f"规划路径点: {pose}")
        success = multirotor_control.go_to_position(pose, stop=False)
        if not success:
            print("移动到路径点失败")
            return

    multirotor_control.controller.current_iris_status.data = 3
    success = multirotor_control.go_to_position(self.third_target_pose, stop=False)
    if not success:
        print("移动到第三人位置失败")
        return
    
    multirotor_control.controller.current_iris_status.data = 4
    success = self.visual_landing(target_type="landing_target_camo")
    if not success:
        print("视觉降落失败，执行基础降落")
        multirotor_control.land(altitude=0.65)
        return
    print("视觉降落成功，准备移动到第一人位置")

    multirotor_control.controller.current_iris_status.data = 5
    multirotor_control.takeoff(altitude=20)
    success = multirotor_control.go_to_position(self.first_target_pose, stop=False)
    if not success:
        print("移动到第一人位置失败")
        return
    
    multirotor_control.controller.current_iris_status.data = 6
    success = multirotor_control.visual_landing(target_type="landing_target_red")
    if not success:
        print("视觉降落失败，执行基础降落")
        multirotor_control.land(altitude=0.65)
        return
    print("视觉降落成功，准备返回起点")

    multirotor_control.controller.current_iris_status.data = 7 
    multirotor_control.takeoff(altitude=20)
    list_2 = path_planner.plan_path(original_poses[2], original_poses[3])
    del list_2[0]
    del list_2[-1]
    for pose in list_2:
        print(f"规划路径点: {pose}")
        success = multirotor_control.go_to_position(pose, stop=False)
        if not success:
            print("移动到路径点失败")
            return
    multirotor_control.takeoff(altitude=20)
    multirotor_control.go_to_position(original_poses[3], stop=True)
    multirotor_control.controller.return_to_home()
    multirotor_control.controller.current_iris_status.data = 8

    if success:
        print("保持悬停，按Ctrl+C退出...")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("收到退出信号")
    
    print("程序退出")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n收到Ctrl+C信号，正在退出...")
    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        print("清理完成")
