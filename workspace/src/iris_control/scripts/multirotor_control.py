#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
多旋翼无人机控制节点 - 集成世界坐标降落功能

该模块提供了完整的多旋翼无人机控制功能，包括：
1. 基础飞行控制（起飞、移动、悬停、降落）
2. 路径规划和导航
3. 基于世界坐标的精确降落控制
4. PID控制算法
5. 任务执行
6. 动态PID参数调整

世界坐标降落功能支持多种目标类型：
- landing_target_camo: 迷彩降落目标
- landing_target_red: 红色降落目标
- landing_target_custom: 自定义降落目标

使用方法：
1. 基础模式：
   python3 multirotor_control.py <multirotor_type> <multirotor_id> <control_type>

2. 世界坐标降落演示：
   python3 multirotor_control.py <multirotor_type> <multirotor_id> <control_type> demo_visual_landing [target_type]

3. 在代码中使用世界坐标降落：
   multirotor_control.visual_landing(target_type="landing_target_camo")
   
   或者在execute_mission中启用：
   multirotor_control.execute_mission(use_visual_landing=True, visual_target_type="landing_target_red")

动态参数调整：
节点运行时可以通过ROS参数服务器动态调整PID参数，参数每0.5秒自动检查更新一次。

支持的动态参数：
- ~pid_x/kp, ~pid_x/ki, ~pid_x/kd: X轴PID参数
- ~pid_y/kp, ~pid_y/ki, ~pid_y/kd: Y轴PID参数  
- ~pid_z/kp, ~pid_z/ki, ~pid_z/kd: Z轴PID参数
- ~max_vel_xy: XY方向最大速度 (m/s)
- ~max_vel_z: Z方向最大速度 (m/s)
- ~landing_threshold: 降落世界坐标误差阈值 (m)
- ~min_altitude: 最小安全高度 (m)
- ~descent_rate: 降落速率 (m/s)
- ~target_timeout: 目标丢失超时时间 (s)

动态调整示例：
# 调整X轴PID参数
rosparam set /multirotor_control_node/pid_x/kp 3.0
rosparam set /multirotor_control_node/pid_x/ki 0.1
rosparam set /multirotor_control_node/pid_x/kd 0.05

# 调整速度限制
rosparam set /multirotor_control_node/max_vel_xy 2.0
rosparam set /multirotor_control_node/max_vel_z 1.5

# 调整降落参数
rosparam set /multirotor_control_node/landing_threshold 1.0
rosparam set /multirotor_control_node/descent_rate 0.3

硬件要求：
- 无人机位姿话题: /iris_0/mavros/local_position/pose (PoseStamped)
- 目标世界坐标话题: /<target_type> (Point)
"""

import sys
import os

# 添加脚本目录到Python路径
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

# 现在导入DroneController
try:
    from drone_control import DroneController
except ImportError:
    # 如果还是导入失败，尝试直接执行文件并获取类
    import importlib.util
    spec = importlib.util.spec_from_file_location("drone_control", os.path.join(script_dir, "drone_control.py"))
    drone_control_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(drone_control_module)
    DroneController = drone_control_module.DroneController
import math
import time
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, Twist, PointStamped, Pose, Point
from std_msgs.msg import Int8, String, Bool, Float64
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from vtol_Astar import VTOLAstarPlanner
MAX_LINEAR = 10.0

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
        self.a=1.0

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
    
    def update_parameters(self, kp=None, ki=None, kd=None, output_limits=None, windup_limit=None):
        """动态更新PID参数"""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        if output_limits is not None:
            self.output_limits = output_limits
        if windup_limit is not None:
            self.windup_limit = windup_limit

class MultirotorControl:
    def __init__(self, controller):
        self.controller = controller
        self.MAX_LINEAR = MAX_LINEAR
        
        # 世界坐标降落相关参数
        self.current_drone_pose = PoseStamped()
        self.target_world_position = Point()
        
        # PID控制参数
        self.kp_x = 2.0    # X轴比例增益
        self.ki_x = 0.0      # X轴积分增益
        self.kd_x = 0.0     # X轴微分增益

        self.kp_y = 2.0     # Y轴比例增益
        self.ki_y = 0.0      # Y轴积分增益
        self.kd_y = 0.0     # Y轴微分增益

        self.kp_z = 4.0     # Z轴比例增益
        self.ki_z = 0.0     # Z轴积分增益
        self.kd_z = 0.0     # Z轴微分增益

        # 降落参数
        self.landing_threshold = 1.0    # 世界坐标误差阈值(米)
        self.min_altitude = 0.8        # 最小安全高度
        self.descent_rate = 5.0      # 降落速率 m/s
        self.max_vel_xy = 5.0          # XY方向最大速度
        self.max_vel_z = 1.0           # Z方向最大速度
        self.target_timeout = 1.0     # 目标丢失超时时间(秒)
        
        # 其他控制参数也从ROS参数服务器获取
        self.max_vel_xy = rospy.get_param('~max_vel_xy', 5.0)          # XY方向最大速度
        self.max_vel_z = rospy.get_param('~max_vel_z', 1.0)            # Z方向最大速度
        self.landing_threshold = rospy.get_param('~landing_threshold', 1.0)    # 世界坐标误差阈值(米)
        self.min_altitude = rospy.get_param('~min_altitude', 0.65)            # 最小安全高度
        self.descent_rate = rospy.get_param('~descent_rate', 1.0)            # 降落速率 m/s
        self.target_timeout = rospy.get_param('~target_timeout', 2.0)         # 目标丢失超时时间(秒)
        
        # 世界坐标降落状态变量
        self.target_detected = False
        self.target_world_position = Point()
        self.last_target_time = rospy.Time.now()
        
        # 无人机位姿订阅器
        self.drone_pose_sub = rospy.Subscriber('/iris_0/mavros/local_position/pose', PoseStamped, self._drone_pose_callback)
        
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
        
        # 世界坐标降落相关话题订阅器
        self.target_sub = None
        self.landing_enabled = False
        
        # 位姿发布器 - 用于在降落完成后发布无人机位姿
        self.bad_man_pose_pub = rospy.Publisher('/zhihang2025/iris_bad_man/pose', Pose, queue_size=1)
        self.healthy_man_pose_pub = rospy.Publisher('/zhihang2025/iris_healthy_man/pose', Pose, queue_size=1)
        
        # 参数更新定时器 - 每0.5秒检查一次参数更新
        self.param_update_timer = rospy.Timer(rospy.Duration(0.5), self._update_parameters_callback)
        
        # 打印当前PID参数
        self._print_current_parameters()

    def _print_current_parameters(self):
        """打印当前PID参数"""
        rospy.loginfo("=== 当前PID参数 ===")
        rospy.loginfo(f"X轴PID: Kp={self.kp_x:.3f}, Ki={self.ki_x:.3f}, Kd={self.kd_x:.3f}")
        rospy.loginfo(f"Y轴PID: Kp={self.kp_y:.3f}, Ki={self.ki_y:.3f}, Kd={self.kd_y:.3f}")
        rospy.loginfo(f"Z轴PID: Kp={self.kp_z:.3f}, Ki={self.ki_z:.3f}, Kd={self.kd_z:.3f}")
        rospy.loginfo(f"最大XY速度: {self.max_vel_xy:.2f} m/s")
        rospy.loginfo(f"最大Z速度: {self.max_vel_z:.2f} m/s")
        rospy.loginfo(f"降落阈值: {self.landing_threshold:.2f} m")  # 修改为米单位
        rospy.loginfo(f"最小高度: {self.min_altitude:.2f} m")
        rospy.loginfo(f"下降速率: {self.descent_rate:.2f} m/s")
        rospy.loginfo("==================")

    def _update_parameters_callback(self, event):
        """参数更新回调函数"""
        try:
            # 检查PID参数是否有更新
            new_kp_x = rospy.get_param('~pid_x/kp', self.kp_x)
            new_ki_x = rospy.get_param('~pid_x/ki', self.ki_x)
            new_kd_x = rospy.get_param('~pid_x/kd', self.kd_x)
            
            new_kp_y = rospy.get_param('~pid_y/kp', self.kp_y)
            new_ki_y = rospy.get_param('~pid_y/ki', self.ki_y)
            new_kd_y = rospy.get_param('~pid_y/kd', self.kd_y)
            
            new_kp_z = rospy.get_param('~pid_z/kp', self.kp_z)
            new_ki_z = rospy.get_param('~pid_z/ki', self.ki_z)
            new_kd_z = rospy.get_param('~pid_z/kd', self.kd_z)
            
            # 检查其他控制参数
            new_max_vel_xy = rospy.get_param('~max_vel_xy', self.max_vel_xy)
            new_max_vel_z = rospy.get_param('~max_vel_z', self.max_vel_z)
            new_landing_threshold = rospy.get_param('~landing_threshold', self.landing_threshold)
            new_min_altitude = rospy.get_param('~min_altitude', self.min_altitude)
            new_descent_rate = rospy.get_param('~descent_rate', self.descent_rate)
            new_target_timeout = rospy.get_param('~target_timeout', self.target_timeout)
            
            # 检查是否有参数变化
            parameters_changed = False
            
            # 更新X轴PID参数
            if (new_kp_x != self.kp_x or new_ki_x != self.ki_x or new_kd_x != self.kd_x or
                new_max_vel_xy != self.max_vel_xy):
                self.kp_x, self.ki_x, self.kd_x = new_kp_x, new_ki_x, new_kd_x
                self.pid_x.update_parameters(
                    kp=self.kp_x, ki=self.ki_x, kd=self.kd_x,
                    output_limits=(-new_max_vel_xy, new_max_vel_xy)
                )
                parameters_changed = True
                rospy.loginfo(f"更新X轴PID参数: Kp={self.kp_x:.3f}, Ki={self.ki_x:.3f}, Kd={self.kd_x:.3f}")
            
            # 更新Y轴PID参数
            if (new_kp_y != self.kp_y or new_ki_y != self.ki_y or new_kd_y != self.kd_y or
                new_max_vel_xy != self.max_vel_xy):
                self.kp_y, self.ki_y, self.kd_y = new_kp_y, new_ki_y, new_kd_y
                self.pid_y.update_parameters(
                    kp=self.kp_y, ki=self.ki_y, kd=self.kd_y,
                    output_limits=(-new_max_vel_xy, new_max_vel_xy)
                )
                parameters_changed = True
                rospy.loginfo(f"更新Y轴PID参数: Kp={self.kp_y:.3f}, Ki={self.ki_y:.3f}, Kd={self.kd_y:.3f}")
            
            # 更新Z轴PID参数
            if (new_kp_z != self.kp_z or new_ki_z != self.ki_z or new_kd_z != self.kd_z or
                new_max_vel_z != self.max_vel_z):
                self.kp_z, self.ki_z, self.kd_z = new_kp_z, new_ki_z, new_kd_z
                self.pid_z.update_parameters(
                    kp=self.kp_z, ki=self.ki_z, kd=self.kd_z,
                    output_limits=(-new_max_vel_z, new_max_vel_z)
                )
                parameters_changed = True
                rospy.loginfo(f"更新Z轴PID参数: Kp={self.kp_z:.3f}, Ki={self.ki_z:.3f}, Kd={self.kd_z:.3f}")
            
            # 更新其他参数
            if new_max_vel_xy != self.max_vel_xy:
                self.max_vel_xy = new_max_vel_xy
                parameters_changed = True
                rospy.loginfo(f"更新最大XY速度: {self.max_vel_xy:.2f} m/s")
                
            if new_max_vel_z != self.max_vel_z:
                self.max_vel_z = new_max_vel_z
                parameters_changed = True
                rospy.loginfo(f"更新最大Z速度: {self.max_vel_z:.2f} m/s")
                
            if new_landing_threshold != self.landing_threshold:
                self.landing_threshold = new_landing_threshold
                parameters_changed = True
                rospy.loginfo(f"更新降落阈值: {self.landing_threshold} px")
                
            if new_min_altitude != self.min_altitude:
                self.min_altitude = new_min_altitude
                parameters_changed = True
                rospy.loginfo(f"更新最小高度: {self.min_altitude:.2f} m")
                
            if new_descent_rate != self.descent_rate:
                self.descent_rate = new_descent_rate
                parameters_changed = True
                rospy.loginfo(f"更新下降速率: {self.descent_rate:.2f} m/s")
                
            if new_target_timeout != self.target_timeout:
                self.target_timeout = new_target_timeout
                parameters_changed = True
                rospy.loginfo(f"更新目标超时: {self.target_timeout:.2f} s")
                
        except Exception as e:
            rospy.logwarn(f"参数更新过程中出现错误: {e}")

    def manual_update_parameters(self):
        """手动触发参数更新"""
        rospy.loginfo("手动更新参数...")
        self._update_parameters_callback(None)
        self._print_current_parameters()

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
        基于世界坐标的精确降落控制器
        Args:
            target_type: 降落目标类型，可选值：
                        "landing_target_camo" - 迷彩降落目标
                        "landing_target_red" - 红色降落目标
                        "landing_target_custom" - 自定义降落目标
        """
        print(f"开始世界坐标降落，目标类型: {target_type}")
        
        # 手动更新参数确保使用最新的PID配置
        self.manual_update_parameters()
        
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
        
        # 检查是否成功降落，如果成功则发布位姿信息
        landing_success = self.current_landing_state == self.LANDING_STATES['LANDED']
        if landing_success:
            print("视觉降落成功，发布位姿信息...")
            self._publish_landing_pose(target_type)
        
        return landing_success
    
    def _setup_visual_landing_subscriber(self, target_type):
        """设置视觉降落目标话题订阅器"""
        if self.target_sub:
            self.target_sub.unregister()
        
        self.target_sub = rospy.Subscriber(
            f'/{target_type}', 
            Point, 
            self._target_callback
        )
        print(f"已订阅视觉目标话题: /{target_type}")
    
    def _target_callback(self, msg):
        """目标世界坐标回调函数"""
        self.target_world_position = msg
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
    
    def _publish_landing_pose(self, target_type):
        """根据目标类型发布降落完成后的位姿信息"""
        try:
            # 获取当前位姿信息
            pose_info, _ = self.controller.get_current_position()
            if pose_info is None:
                print("无法获取当前位姿信息，无法发布位姿")
                return False
            
            # 创建Pose消息
            pose_msg = Pose()
            pose_msg.position = pose_info.pose.position
            pose_msg.orientation = pose_info.pose.orientation
            
            # 根据目标类型选择发布话题
            if target_type == "landing_target_red":
                self.bad_man_pose_pub.publish(pose_msg)
                print(f"已发布危重病人位姿到 /zhihang2025/iris_bad_man/pose")
                print(f"位置: x={pose_msg.position.x:.3f}, y={pose_msg.position.y:.3f}, z={pose_msg.position.z:.3f}")
            elif target_type == "landing_target_camo":
                self.healthy_man_pose_pub.publish(pose_msg)
                print(f"已发布健康人员位姿到 /zhihang2025/iris_healthy_man/pose")
                print(f"位置: x={pose_msg.position.x:.3f}, y={pose_msg.position.y:.3f}, z={pose_msg.position.z:.3f}")
            else:
                print(f"未知的目标类型 {target_type}，不发布位姿信息")
                return False
            
            return True
            
        except Exception as e:
            print(f"发布位姿信息时出现错误: {e}")
            return False

    def _calculate_world_error(self):
        """计算世界坐标误差"""
        if not self.target_detected:
            return None, None, None
            
        # 获取无人机当前位置
        drone_x = self.current_drone_pose.pose.position.x
        drone_y = self.current_drone_pose.pose.position.y
        
        # 获取目标世界坐标
        target_x = self.target_world_position.x
        target_y = self.target_world_position.y
        
        # 计算误差
        error_x = target_x - drone_x
        error_y = target_y - drone_y
        
        # 计算总误差距离
        error_distance = math.sqrt(error_x**2 + error_y**2)
        
        return error_x, error_y, error_distance
    
    def _world_error_to_velocity_pid(self, error_x, error_y):
        """使用PID控制器将世界坐标误差转换为速度命令"""
        current_time = rospy.Time.now()
        
        # 直接使用世界坐标误差进行PID控制
        vel_x = self.pid_x.update(error_x, current_time)
        vel_y = self.pid_y.update(error_y, current_time)
        
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
        """执行世界坐标降落状态机"""
        if not self.landing_enabled:
            return
            
        # 检查目标超时
        target_timeout = self._check_target_timeout()
        
        # 计算世界坐标误差
        error_x, error_y, error_distance = self._calculate_world_error()
        
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
            if error_distance is not None:
                print(f"跟踪目标中，误差距离: {error_distance:.2f}m")
            else:
                print("跟踪目标中，等待目标检测...")
            
            if target_timeout:
                self.current_landing_state = self.LANDING_STATES['SEARCHING']
                self.pid_x.reset()
                self.pid_y.reset()
                print("目标丢失，切换到搜索状态")
                
            if error_distance is not None:
                # 使用PID控制器计算XY方向速度
                vel_x, vel_y = self._world_error_to_velocity_pid(error_x, error_y)
                
                # 应用速度命令
                self.controller.current_twist.linear.x = vel_x
                self.controller.current_twist.linear.y = vel_y
                self.controller.current_twist.linear.z = 0.0
                
                print(f"跟踪调整: vx={vel_x:.3f}, vy={vel_y:.3f}, 误差: x={error_x:.2f}m, y={error_y:.2f}m")
                
                # 判断是否可以开始下降
                if error_distance < self.landing_threshold and current_altitude > 0.65:
                    self.current_landing_state = self.LANDING_STATES['DESCENDING']
                    print("目标居中，开始下降")
                    
        elif self.current_landing_state == self.LANDING_STATES['DESCENDING']:
            if error_distance is not None:
                print(f"下降中，高度: {current_altitude:.2f}m，目标误差: {error_distance:.2f}m")
            else:
                print(f"下降中，高度: {current_altitude:.2f}m，等待目标检测...")
            
            if target_timeout:
                self.current_landing_state = self.LANDING_STATES['SEARCHING']
                self.pid_x.reset()
                self.pid_y.reset()
                self.pid_z.reset()
                print("下降过程中目标丢失，切换到搜索状态")
                return
                
            if error_distance is not None:
                # 使用PID控制器计算XY方向速度（下降时保持跟踪）
                vel_x, vel_y = self._world_error_to_velocity_pid(error_x, error_y)
                
                # 使用PID控制器控制下降速度
                target_altitude = max(0.65, current_altitude - 0.5)
                vel_z = self._altitude_to_velocity_pid(target_altitude)
                
                # 限制下降速度
                vel_z = max(-self.descent_rate, min(0.0, vel_z))
                
                # 应用速度命令
                self.controller.current_twist.linear.x = vel_x
                self.controller.current_twist.linear.y = vel_y
                self.controller.current_twist.linear.z = vel_z
                
                print(f"下降调整: vx={vel_x:.3f}, vy={vel_y:.3f}, vz={vel_z:.3f}")
                
                # 判断是否已经降落到目标高度（0.65米）
                if current_altitude <= 0.65:
                    # 停止所有运动
                    self.controller.current_twist.linear.x = 0.0
                    self.controller.current_twist.linear.y = 0.0
                    self.controller.current_twist.linear.z = 0.0
                    self.controller.set_hover_mode()
                    
                    self.current_landing_state = self.LANDING_STATES['LANDED']
                    print(f"已降落到目标高度 {current_altitude:.2f}m，视觉降落完成")
                    
        elif self.current_landing_state == self.LANDING_STATES['LANDING']:
            # 这个状态现在基本不会用到，因为直接从DESCENDING跳到LANDED
            print("执行最终降落")
            
            # 停止所有运动
            self.controller.current_twist.linear.x = 0.0
            self.controller.current_twist.linear.y = 0.0
            self.controller.current_twist.linear.z = 0.0
            self.controller.set_hover_mode()
            
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
        position = msg.position
        self.first_target_pose = (position.x, position.y)  # 假设高度为20米
        print(f"接收到第一人的位置信息: x={position.x}, y={position.y}, z={position.z}")

    def third_pose_callback(self, msg):
        """处理第三人的位置的信息"""
        position = msg.position
        self.third_target_pose = (position.x, position.y)  # 假设高度为20米
        print(f"接收到第三人的位置信息: x={position.x}, y={position.y}, z={position.z}")

    def done_callback(self, msg):
        self.done = (msg.data == 5)
        if self.done:
            print("固定翼飞机已降落")
        else:
            print("固定翼飞机还在起飞")

def demo_visual_landing():
    """演示世界坐标降落功能的示例函数"""
    if len(sys.argv) < 4:
        print("用法: python3 multirotor_control.py <multirotor_type> <multirotor_id> <control_type> [demo_visual_landing]")
        return

    multirotor_type = sys.argv[1]
    multirotor_id = int(sys.argv[2])
    control_type = sys.argv[3]

    # 创建控制器和控制类
    controller = DroneController(multirotor_type, multirotor_id, control_type)
    multirotor_control = MultirotorControl(controller)
    
    print("=== 世界坐标降落演示 ===")
    
    # 步骤1：起飞到合适高度
    print("1. 起飞到20米高度...")
    multirotor_control.takeoff(altitude=20)
    
    # 步骤2：移动到降落区域上方
    print("2. 移动到降落区域上方...")
    landing_area_position = [0, 0]  # 可以根据需要调整位置
    multirotor_control.go_to_position(landing_area_position, stop=True)
    
    # 步骤3：执行世界坐标降落
    print("3. 开始世界坐标降落...")
    
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
        print("4. 世界坐标降落成功完成！")
        multirotor_control.controller.current_iris_status.data = 8  # 任务完成状态
    else:
        print("4. 世界坐标降落失败，执行基础降落...")
        multirotor_control.land(altitude=0.65)
    
    print("=== 世界坐标降落演示完成 ===")

def main():
    # 过滤掉ROS的重映射参数
    filtered_args = []
    for arg in sys.argv:
        if ':=' not in arg:  # 过滤掉包含':='的ROS重映射参数
            filtered_args.append(arg)
    
    # 检查是否是演示模式
    if len(filtered_args) > 4 and "demo_visual_landing" in filtered_args:
        demo_visual_landing()
        return
    
    # 先尝试从命令行参数获取，如果没有则使用默认值
    if len(filtered_args) >= 4:
        multirotor_type = filtered_args[1]
        multirotor_id = int(filtered_args[2])
        control_type = filtered_args[3]
    else:
        # 使用默认值，这些可能会被ROS参数覆盖
        multirotor_type = "iris"
        multirotor_id = 0
        control_type = "vel"
    
    # 先创建控制器（这会初始化ROS节点）
    controller = DroneController(multirotor_type, multirotor_id, control_type)
    
    # 如果通过ROS参数服务器获取参数，则更新参数
    if rospy.has_param('~multirotor_type'):
        controller.multirotor_type = rospy.get_param('~multirotor_type', multirotor_type)
    if rospy.has_param('~multirotor_id'):
        controller.multirotor_id = rospy.get_param('~multirotor_id', multirotor_id)
    if rospy.has_param('~control_type'):
        controller.control_type = rospy.get_param('~control_type', control_type)
    multirotor_control = MultirotorControl(controller)
    
    node = PoseNode()
    while not node.done:
        rospy.sleep(0.1)

    original_poses = [(2, 3),node.third_target_pose, node.first_target_pose , (0, 1)]
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
    success = multirotor_control.go_to_position(node.third_target_pose, stop=True)
    if not success:
        print("移动到第三人位置失败")
        return
    
    multirotor_control.controller.current_iris_status.data = 4
    multirotor_control.land(altitude=10)
    success = multirotor_control.visual_landing(target_type="landing_target_camo/world_coord")
    if not success:
        print("世界坐标降落失败，执行基础降落")
        multirotor_control.land(altitude=0.65)
    print("世界坐标降落成功，准备移动到第一人位置")

    multirotor_control.controller.current_iris_status.data = 5
    multirotor_control.takeoff(altitude=20)
    success = multirotor_control.go_to_position(node.first_target_pose, stop=True)
    if not success:
        print("移动到第一人位置失败")
        return
    
    multirotor_control.controller.current_iris_status.data = 6
    success = multirotor_control.visual_landing(target_type="landing_target_red/world_coord")
    if not success:
        print("世界坐标降落失败，执行基础降落")
        multirotor_control.land(altitude=0.65)
    print("世界坐标降落成功，准备返回起点")

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
    multirotor_control.controller.return_home()
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