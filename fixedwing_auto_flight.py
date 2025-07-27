#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
固定翼无人机自动飞行脚本
参考 vtol_communication_enhanced.py 的接口
目标：让 standard_vtol_0 飞到指定目标点 person_red (x=1495.000, y=-105.000, z=0.200)
"""

import rospy
import time
import math
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, CommandVtolTransition, SetMode
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from pyquaternion import Quaternion


class FixedWingAutoFlight:
    def __init__(self, vehicle_type="standard_vtol", vehicle_id="0"):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        
        # 目标位置 (person_red)
        self.target_x = 1495.000
        self.target_y = -105.000
        self.target_z = 0.200
        
        # 当前状态
        self.current_position = None
        self.current_yaw = 0
        self.arm_state = False
        self.flight_mode = None
        self.transition_state = 'multirotor'  # 初始为多旋翼模式
        self.plane_mission = None
        
        # 控制参数
        self.coordinate_frame = 1  # ENU坐标系
        self.motion_type = 0  # 位置控制
        self.target_motion = PositionTarget()
        
        # 飞行状态
        self.mission_state = "INIT"  # INIT, ARMED, TAKEOFF, CRUISE, APPROACH, LAND, COMPLETE
        self.takeoff_height = 50.0  # 起飞高度
        self.cruise_height = 80.0   # 巡航高度
        self.approach_distance = 100.0  # 开始下降的距离
        
        print(f"初始化固定翼自动飞行控制器: {self.vehicle_type}_{self.vehicle_id}")
        self.init_ros()

    def init_ros(self):
        """初始化ROS节点和通信"""
        rospy.init_node(f"{self.vehicle_type}_{self.vehicle_id}_auto_flight")
        
        # 订阅者
        self.local_pose_sub = rospy.Subscriber(
            f"{self.vehicle_type}_{self.vehicle_id}/mavros/local_position/pose", 
            PoseStamped, self.local_pose_callback, queue_size=1)
        
        # 发布者
        self.target_motion_pub = rospy.Publisher(
            f"{self.vehicle_type}_{self.vehicle_id}/mavros/setpoint_raw/local", 
            PositionTarget, queue_size=1)
        
        self.cmd_pub = rospy.Publisher(
            f"/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd", 
            String, queue_size=1)
        
        # 服务客户端
        self.arm_service = rospy.ServiceProxy(
            f"{self.vehicle_type}_{self.vehicle_id}/mavros/cmd/arming", CommandBool)
        
        self.flight_mode_service = rospy.ServiceProxy(
            f"{self.vehicle_type}_{self.vehicle_id}/mavros/set_mode", SetMode)
        
        self.transition_service = rospy.ServiceProxy(
            f"{self.vehicle_type}_{self.vehicle_id}/mavros/cmd/vtol_transition", 
            CommandVtolTransition)
        
        print("ROS通信初始化完成")

    def local_pose_callback(self, msg):
        """位置回调函数"""
        self.current_position = msg.pose.position
        self.current_yaw = self.q2yaw(msg.pose.orientation)

    def q2yaw(self, q):
        """四元数转偏航角"""
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]
        return rotate_z_rad

    def construct_target(self, x=0, y=0, z=0, vx=0, vy=0, vz=0, yaw=0, yaw_rate=0):
        """构造目标位置消息"""
        target = PositionTarget()
        target.coordinate_frame = self.coordinate_frame
        
        target.position.x = x
        target.position.y = y
        target.position.z = z
        target.velocity.x = vx
        target.velocity.y = vy
        target.velocity.z = vz
        target.yaw = yaw
        target.yaw_rate = yaw_rate
        
        if self.transition_state == 'plane':
            # 固定翼模式下的特殊type_mask设置
            if self.plane_mission == 'takeoff':
                target.type_mask = 4096
            elif self.plane_mission == 'land':
                target.type_mask = 8192
            elif self.plane_mission == 'loiter':
                target.type_mask = 12288
            else:
                target.type_mask = 16384
        else:
            # 多旋翼模式
            if self.motion_type == 0:  # 位置控制
                target.type_mask = (PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + 
                                  PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + 
                                  PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + 
                                  PositionTarget.IGNORE_YAW_RATE)
            elif self.motion_type == 1:  # 速度控制
                target.type_mask = (PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + 
                                  PositionTarget.IGNORE_PZ + PositionTarget.IGNORE_AFX + 
                                  PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + 
                                  PositionTarget.IGNORE_YAW)
        
        return target

    def arm(self):
        """解锁"""
        try:
            if self.arm_service(True):
                self.arm_state = True
                print(f"{self.vehicle_type}_{self.vehicle_id}: 解锁成功")
                return True
            else:
                print(f"{self.vehicle_type}_{self.vehicle_id}: 解锁失败")
                return False
        except Exception as e:
            print(f"解锁服务调用失败: {e}")
            return False

    def set_mode(self, mode):
        """设置飞行模式"""
        try:
            if self.flight_mode_service(custom_mode=mode):
                self.flight_mode = mode
                print(f"{self.vehicle_type}_{self.vehicle_id}: 模式切换到 {mode}")
                return True
            else:
                print(f"{self.vehicle_type}_{self.vehicle_id}: 模式切换失败 {mode}")
                return False
        except Exception as e:
            print(f"模式切换服务调用失败: {e}")
            return False

    def transition_to_plane(self):
        """切换到固定翼模式"""
        try:
            result = self.transition_service(state=4)  # 4 = 固定翼模式
            if result.success:
                self.transition_state = 'plane'
                print(f"{self.vehicle_type}_{self.vehicle_id}: 切换到固定翼模式成功")
                return True
            else:
                print(f"{self.vehicle_type}_{self.vehicle_id}: 切换到固定翼模式失败")
                return False
        except Exception as e:
            print(f"模式切换服务调用失败: {e}")
            return False

    def get_distance_to_target(self):
        """计算到目标点的距离"""
        if self.current_position is None:
            return float('inf')
        
        dx = self.target_x - self.current_position.x
        dy = self.target_y - self.current_position.y
        dz = self.target_z - self.current_position.z
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def get_horizontal_distance_to_target(self):
        """计算到目标点的水平距离"""
        if self.current_position is None:
            return float('inf')
        
        dx = self.target_x - self.current_position.x
        dy = self.target_y - self.current_position.y
        
        return math.sqrt(dx*dx + dy*dy)

    def calculate_target_yaw(self):
        """计算朝向目标点的偏航角"""
        if self.current_position is None:
            return 0
        
        dx = self.target_x - self.current_position.x
        dy = self.target_y - self.current_position.y
        
        return math.atan2(dy, dx)

    def execute_mission(self):
        """执行自动飞行任务"""
        rate = rospy.Rate(20)  # 20Hz
        
        print("开始执行自动飞行任务...")
        print(f"目标位置: x={self.target_x}, y={self.target_y}, z={self.target_z}")
        
        while not rospy.is_shutdown():
            if self.current_position is None:
                print("等待位置信息...")
                rate.sleep()
                continue
            
            # 状态机
            if self.mission_state == "INIT":
                self.init_mission()
            elif self.mission_state == "ARMED":
                self.takeoff_phase()
            elif self.mission_state == "TAKEOFF":
                self.transition_phase()
            elif self.mission_state == "CRUISE":
                self.cruise_phase()
            elif self.mission_state == "APPROACH":
                self.approach_phase()
            elif self.mission_state == "LAND":
                self.land_phase()
            elif self.mission_state == "COMPLETE":
                print("任务完成!")
                break
            
            # 发布目标位置
            self.target_motion_pub.publish(self.target_motion)
            
            # 打印状态信息
            if hasattr(self, 'status_counter'):
                self.status_counter += 1
            else:
                self.status_counter = 0
                
            if self.status_counter % 100 == 0:  # 每5秒打印一次状态
                dist = self.get_distance_to_target()
                print(f"状态: {self.mission_state}, 位置: ({self.current_position.x:.1f}, "
                      f"{self.current_position.y:.1f}, {self.current_position.z:.1f}), "
                      f"距离目标: {dist:.1f}m")
            
            rate.sleep()

    def init_mission(self):
        """初始化任务"""
        print("初始化任务阶段...")
        
        # 设置OFFBOARD模式
        if self.set_mode("OFFBOARD"):
            time.sleep(1)
            
            # 解锁
            if self.arm():
                self.mission_state = "ARMED"
                print("切换到解锁阶段")
            else:
                print("解锁失败，重试...")
                time.sleep(2)
        else:
            print("设置OFFBOARD模式失败，重试...")
            time.sleep(2)

    def takeoff_phase(self):
        """起飞阶段"""
        # 只在第一次进入时打印
        if not hasattr(self, 'takeoff_started'):
            print("起飞阶段...")
            self.takeoff_started = True
        
        # 垂直起飞到指定高度
        target_yaw = self.calculate_target_yaw()
        self.target_motion = self.construct_target(
            x=self.current_position.x,
            y=self.current_position.y,
            z=self.takeoff_height,
            yaw=target_yaw
        )
        
        # 检查是否达到起飞高度（考虑地面以下的情况）
        height_above_start = self.current_position.z - (-0.6)  # 假设起始地面为-0.6m
        if height_above_start >= self.takeoff_height - 5.0:  # 放宽容忍度
            self.mission_state = "TAKEOFF"
            print(f"起飞完成，当前高度：{self.current_position.z:.1f}m，准备切换到固定翼模式")

    def transition_phase(self):
        """模式切换阶段"""
        if not hasattr(self, 'transition_started'):
            print("切换到固定翼模式...")
            self.transition_started = True
        
        # 切换到固定翼模式
        if self.transition_to_plane():
            time.sleep(3)  # 等待切换完成
            self.mission_state = "CRUISE"
            print("切换到巡航阶段")
        else:
            if not hasattr(self, 'transition_retry_count'):
                self.transition_retry_count = 0
            self.transition_retry_count += 1
            if self.transition_retry_count % 10 == 0:  # 每10次尝试打印一次
                print("固定翼模式切换失败，重试...")
            time.sleep(2)

    def cruise_phase(self):
        """巡航阶段"""
        # 计算目标方向
        target_yaw = self.calculate_target_yaw()
        horizontal_dist = self.get_horizontal_distance_to_target()
        
        # 打印巡航状态（每20次循环打印一次）
        if not hasattr(self, 'cruise_counter'):
            self.cruise_counter = 0
        self.cruise_counter += 1
        if self.cruise_counter % 20 == 0:
            print(f"巡航中...距离目标: {horizontal_dist:.1f}m")
        
        # 如果接近目标，开始下降
        if horizontal_dist < self.approach_distance:
            self.mission_state = "APPROACH"
            print("开始接近目标，进入下降阶段")
            return
        
        # 直接朝目标飞行
        self.target_motion = self.construct_target(
            x=self.target_x,
            y=self.target_y,
            z=self.cruise_height,
            yaw=target_yaw
        )

    def approach_phase(self):
        """接近目标阶段"""
        horizontal_dist = self.get_horizontal_distance_to_target()
        
        # 计算下降高度（线性插值）
        if horizontal_dist > 10:
            descent_ratio = (horizontal_dist - 10) / (self.approach_distance - 10)
            current_target_z = self.target_z + descent_ratio * (self.cruise_height - self.target_z)
        else:
            current_target_z = self.target_z
        
        target_yaw = self.calculate_target_yaw()
        
        self.target_motion = self.construct_target(
            x=self.target_x,
            y=self.target_y,
            z=current_target_z,
            yaw=target_yaw
        )
        
        # 检查是否到达目标
        total_dist = self.get_distance_to_target()
        if total_dist < 5.0:
            self.mission_state = "LAND"
            print("接近目标，准备降落")

    def land_phase(self):
        """降落阶段"""
        print("降落阶段...")
        
        # 设置降落任务
        self.plane_mission = 'land'
        target_yaw = self.calculate_target_yaw()
        
        self.target_motion = self.construct_target(
            x=self.target_x,
            y=self.target_y,
            z=self.target_z,
            yaw=target_yaw
        )
        
        # 检查是否着陆
        if self.current_position.z < self.target_z + 1.0:
            self.mission_state = "COMPLETE"
            print("降落完成")

    def run(self):
        """运行主程序"""
        print("等待ROS连接...")
        time.sleep(2)
        
        print("开始自动飞行任务")
        self.execute_mission()


if __name__ == '__main__':
    try:
        # 创建固定翼自动飞行控制器
        auto_flight = FixedWingAutoFlight("standard_vtol", "0")
        auto_flight.run()
    except rospy.ROSInterruptException:
        print("程序被中断")
    except Exception as e:
        print(f"程序异常: {e}")
        import traceback
        traceback.print_exc()
