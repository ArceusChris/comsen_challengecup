#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
固定翼无人机瞬移控制脚本
Author: GitHub Copilot
Description: 通过Gazebo服务实现固定翼无人机的瞬移功能
"""

import rospy
import sys
import math
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Pose, Twist
from std_msgs.msg import String
from pyquaternion import Quaternion
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Point, Quaternion as GeometryQuaternion

class PlaneTeleport:
    """固定翼无人机瞬移控制类"""
    
    def __init__(self, vehicle_type, vehicle_id):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        self.current_position = None
        self.current_yaw = 0
        self.target_motion = PositionTarget()
        self.arm_state = False
        self.coordinate_frame = 1
        self.motion_type = 0
        self.flight_mode = None
        self.plane_mission = None
        self.last_cmd = None
        self.min_airspeed = 15.0  # 固定翼最小空速 (m/s)
        self.cruise_altitude = 100.0  # 巡航高度 (m)
        self.min_altitude = 10.0  # 最小飞行高度 (m)
        
        # 初始化ROS节点
        rospy.init_node(self.vehicle_type + '_' + self.vehicle_id + "_plane_teleport")
        self.rate = rospy.Rate(30)
        
        '''
        ROS订阅者
        '''
        self.local_pose_sub = rospy.Subscriber(
            self.vehicle_type + '_' + self.vehicle_id + "/mavros/local_position/pose", 
            PoseStamped, self.local_pose_callback, queue_size=1)
        
        self.cmd_sub = rospy.Subscriber(
            "/xtdrone/" + self.vehicle_type + '_' + self.vehicle_id + "/cmd", 
            String, self.cmd_callback, queue_size=3)
        
        self.cmd_pose_enu_sub = rospy.Subscriber(
            "/xtdrone/" + self.vehicle_type + '_' + self.vehicle_id + "/cmd_pose_enu", 
            Pose, self.cmd_pose_enu_callback, queue_size=1)
        
        '''
        ROS发布者
        '''
        self.target_motion_pub = rospy.Publisher(
            self.vehicle_type + '_' + self.vehicle_id + "/mavros/setpoint_raw/local", 
            PositionTarget, queue_size=1)
        
        '''
        ROS服务
        '''
        self.armService = rospy.ServiceProxy(
            self.vehicle_type + '_' + self.vehicle_id + "/mavros/cmd/arming", CommandBool)
        
        self.flightModeService = rospy.ServiceProxy(
            self.vehicle_type + '_' + self.vehicle_id + "/mavros/set_mode", SetMode)
        
        # Gazebo仿真服务
        rospy.wait_for_service('/gazebo/get_model_state')
        rospy.wait_for_service('/gazebo/set_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        print(f"{self.vehicle_type}_{self.vehicle_id}: Plane teleport communication initialized")

    def start(self):
        """主循环"""
        while not rospy.is_shutdown():
            self.target_motion_pub.publish(self.target_motion)
            self.rate.sleep()

    def local_pose_callback(self, msg):
        """位置回调函数"""
        self.current_position = msg.pose.position
        self.current_yaw = self.q2yaw(msg.pose.orientation)

    def construct_target(self, x=0, y=0, z=0, vx=0, vy=0, vz=0, yaw=0, yaw_rate=0):
        """构造目标运动指令"""
        target_raw_pose = PositionTarget()
        target_raw_pose.coordinate_frame = self.coordinate_frame
        
        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z
        
        target_raw_pose.velocity.x = vx
        target_raw_pose.velocity.y = vy
        target_raw_pose.velocity.z = vz
        
        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate
        
        if self.motion_type == 0:  # 位置控制
            target_raw_pose.type_mask = (PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + 
                                       PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + 
                                       PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + 
                                       PositionTarget.IGNORE_YAW_RATE)
        elif self.motion_type == 1:  # 速度控制
            target_raw_pose.type_mask = (PositionTarget.IGNORE_PX + PositionTarget.IGNORE_PY + 
                                       PositionTarget.IGNORE_PZ + PositionTarget.IGNORE_AFX + 
                                       PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + 
                                       PositionTarget.IGNORE_YAW)
        
        return target_raw_pose

    def cmd_pose_enu_callback(self, msg):
        """ENU坐标系位置指令回调"""
        self.coordinate_frame = 1
        self.motion_type = 0
        yaw = self.q2yaw(msg.orientation)
        
        # 固定翼确保最小飞行高度
        z = max(msg.position.z, self.min_altitude)
        
        self.target_motion = self.construct_target(
            x=msg.position.x, y=msg.position.y, z=z, yaw=yaw)

    def cmd_callback(self, msg):
        """命令回调函数"""
        if msg.data == self.last_cmd or msg.data == '' or msg.data == 'stop controlling':
            return

        elif msg.data == 'ARM':
            self.arm_state = self.arm()
            print(f"{self.vehicle_type}_{self.vehicle_id}: Armed {self.arm_state}")

        elif msg.data == 'DISARM':
            self.arm_state = not self.disarm()
            print(f"{self.vehicle_type}_{self.vehicle_id}: Armed {self.arm_state}")

        # 瞬移到指定模型位置
        elif msg.data.startswith('TELEPORT_TO_MODEL'):
            # 格式: "TELEPORT_TO_MODEL,model_name,height_offset,airspeed"
            try:
                parts = msg.data.split(',')
                if len(parts) >= 2:
                    model_name = parts[1]
                    height_offset = float(parts[2]) if len(parts) > 2 else 50.0
                    airspeed = float(parts[3]) if len(parts) > 3 else self.min_airspeed
                    self.teleport_to_model(model_name, height_offset, airspeed)
            except Exception as e:
                print(f"{self.vehicle_type}_{self.vehicle_id}: Error in teleport_to_model: {e}")
        
        # Gazebo直接瞬移
        elif msg.data.startswith('GAZEBO_TELEPORT'):
            # 格式: "GAZEBO_TELEPORT,x,y,z,yaw,airspeed"
            try:
                parts = msg.data.split(',')
                if len(parts) >= 4:
                    x = float(parts[1])
                    y = float(parts[2])
                    z = float(parts[3])
                    yaw = float(parts[4]) if len(parts) > 4 else 0.0
                    airspeed = float(parts[5]) if len(parts) > 5 else self.min_airspeed
                    self.gazebo_teleport(x, y, z, yaw, airspeed)
            except ValueError:
                print(f"{self.vehicle_type}_{self.vehicle_id}: Invalid gazebo teleport command format")

        # 计算航向并瞬移到目标点
        elif msg.data.startswith('TELEPORT_TO_POINT'):
            # 格式: "TELEPORT_TO_POINT,x,y,z,airspeed"
            try:
                parts = msg.data.split(',')
                if len(parts) >= 4:
                    x = float(parts[1])
                    y = float(parts[2])
                    z = float(parts[3])
                    airspeed = float(parts[4]) if len(parts) > 4 else self.min_airspeed
                    self.teleport_to_point(x, y, z, airspeed)
            except ValueError:
                print(f"{self.vehicle_type}_{self.vehicle_id}: Invalid teleport to point command format")

        # 固定翼特殊任务
        elif msg.data.startswith('PLANE_'):
            if msg.data == 'PLANE_TAKEOFF':
                self.plane_takeoff()
            elif msg.data == 'PLANE_LAND':
                self.plane_land()
            elif msg.data == 'PLANE_LOITER':
                self.plane_loiter()
            elif msg.data == 'PLANE_CRUISE':
                self.plane_cruise()

        else:
            self.flight_mode = msg.data
            self.flight_mode_switch()

        self.last_cmd = msg.data

    def calculate_heading_to_point(self, target_x, target_y):
        """计算到目标点的航向角"""
        if self.current_position:
            dx = target_x - self.current_position.x
            dy = target_y - self.current_position.y
            return math.atan2(dy, dx)
        return 0.0

    def teleport_to_point(self, x, y, z, airspeed=15.0):
        """瞬移到指定点，自动计算航向"""
        # 计算面向目标的航向角
        yaw = self.calculate_heading_to_point(x, y)
        self.gazebo_teleport(x, y, z, yaw, airspeed)

    def teleport_to_model(self, model_name, height_offset=50.0, airspeed=15.0):
        """瞬移到指定模型位置上方"""
        try:
            response = self.get_model_state(model_name, '')
            if response.success:
                target_x = response.pose.position.x
                target_y = response.pose.position.y
                target_z = max(response.pose.position.z + height_offset, self.min_altitude)
                
                # 计算面向目标的航向角
                yaw = self.calculate_heading_to_point(target_x, target_y)
                
                self.gazebo_teleport(target_x, target_y, target_z, yaw, airspeed)
                print(f"{self.vehicle_type}_{self.vehicle_id}: Teleported to {model_name} at ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
            else:
                print(f"{self.vehicle_type}_{self.vehicle_id}: Failed to get {model_name} position")
        except Exception as e:
            print(f"{self.vehicle_type}_{self.vehicle_id}: Error getting model state: {e}")

    def gazebo_teleport(self, x, y, z, yaw, airspeed=15.0):
        """Gazebo固定翼瞬移（带初始速度）"""
        try:
            # 确保飞行高度合理
            z = max(z, self.min_altitude)
            
            # 创建模型状态
            model_state = ModelState()
            model_state.model_name = self.vehicle_type + '_' + self.vehicle_id
            
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
            
            # 设置初始速度（固定翼需要维持前进速度）
            model_state.twist.linear.x = airspeed * math.cos(yaw)
            model_state.twist.linear.y = airspeed * math.sin(yaw)
            model_state.twist.linear.z = 0.0
            model_state.twist.angular.x = 0.0
            model_state.twist.angular.y = 0.0
            model_state.twist.angular.z = 0.0
            
            model_state.reference_frame = 'world'
            
            # 执行瞬移
            response = self.set_model_state(model_state)
            if response.success:
                print(f"{self.vehicle_type}_{self.vehicle_id}: Gazebo teleported to ({x:.2f}, {y:.2f}, {z:.2f}, yaw:{yaw:.2f}, speed:{airspeed:.1f})")
                
                # 设置飞行控制目标
                self.coordinate_frame = 1
                self.motion_type = 0
                self.target_motion = self.construct_target(x=x, y=y, z=z, yaw=yaw)
                
                # 自动设置为OFFBOARD模式
                rospy.sleep(0.5)
                self.flight_mode = 'OFFBOARD'
                self.flight_mode_switch()
                
            else:
                print(f"{self.vehicle_type}_{self.vehicle_id}: Gazebo teleport failed: {response.status_message}")
                
        except Exception as e:
            print(f"{self.vehicle_type}_{self.vehicle_id}: Error in gazebo_teleport: {e}")

    def plane_takeoff(self):
        """固定翼起飞"""
        if self.current_position:
            # 起飞到前方200米处，高度100米
            takeoff_x = self.current_position.x + 200
            takeoff_y = self.current_position.y
            takeoff_z = self.cruise_altitude
            yaw = self.current_yaw
            
            self.gazebo_teleport(takeoff_x, takeoff_y, takeoff_z, yaw, 20.0)
            print(f"{self.vehicle_type}_{self.vehicle_id}: Plane takeoff executed")
        else:
            # 默认起飞位置
            self.gazebo_teleport(200, 0, self.cruise_altitude, 0, 20.0)

    def plane_land(self):
        """固定翼降落"""
        if self.current_position:
            # 降落到前方500米处，降低高度
            land_x = self.current_position.x + 500
            land_y = self.current_position.y
            land_z = 5.0
            yaw = self.current_yaw
            
            self.gazebo_teleport(land_x, land_y, land_z, yaw, 18.0)
            print(f"{self.vehicle_type}_{self.vehicle_id}: Plane landing executed")

    def plane_loiter(self):
        """固定翼盘旋"""
        if self.current_position:
            # 在当前位置上方盘旋
            loiter_z = max(self.current_position.z, 50.0)
            self.coordinate_frame = 1
            self.motion_type = 0
            self.target_motion = self.construct_target(
                x=self.current_position.x, 
                y=self.current_position.y, 
                z=loiter_z, 
                yaw=self.current_yaw
            )
            print(f"{self.vehicle_type}_{self.vehicle_id}: Plane loiter mode")

    def plane_cruise(self):
        """固定翼巡航"""
        if self.current_position:
            # 前方1000米巡航
            cruise_x = self.current_position.x + 1000
            cruise_y = self.current_position.y
            cruise_z = self.cruise_altitude
            yaw = self.current_yaw
            
            self.coordinate_frame = 1
            self.motion_type = 0
            self.target_motion = self.construct_target(x=cruise_x, y=cruise_y, z=cruise_z, yaw=yaw)
            print(f"{self.vehicle_type}_{self.vehicle_id}: Plane cruise mode")

    def q2yaw(self, q):
        """四元数转偏航角"""
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]
        return rotate_z_rad

    def arm(self):
        """解锁"""
        try:
            if self.armService(True):
                return True
            else:
                print(f"{self.vehicle_type}_{self.vehicle_id}: arming failed!")
                return False
        except:
            print(f"{self.vehicle_type}_{self.vehicle_id}: arm service call failed!")
            return False

    def disarm(self):
        """上锁"""
        try:
            if self.armService(False):
                return True
            else:
                print(f"{self.vehicle_type}_{self.vehicle_id}: disarming failed!")
                return False
        except:
            print(f"{self.vehicle_type}_{self.vehicle_id}: disarm service call failed!")
            return False

    def flight_mode_switch(self):
        """飞行模式切换"""
        try:
            if self.flightModeService(custom_mode=self.flight_mode):
                print(f"{self.vehicle_type}_{self.vehicle_id}: {self.flight_mode}")
                return True
            else:
                print(f"{self.vehicle_type}_{self.vehicle_id}: {self.flight_mode} failed")
                return False
        except:
            print(f"{self.vehicle_type}_{self.vehicle_id}: flight mode service call failed!")
            return False


def print_usage():
    """打印使用说明"""
    print("="*60)
    print("固定翼无人机瞬移脚本使用说明")
    print("="*60)
    print("启动方法:")
    print("  python plane_teleport.py <vehicle_type> <vehicle_id>")
    print("  例如: python plane_teleport.py plane 0")
    print()
    print("支持的瞬移命令:")
    print("1. Gazebo直接瞬移:")
    print("   rostopic pub /xtdrone/plane_0/cmd std_msgs/String \"data: 'GAZEBO_TELEPORT,x,y,z,yaw,airspeed'\"")
    print("   例如: rostopic pub /xtdrone/plane_0/cmd std_msgs/String \"data: 'GAZEBO_TELEPORT,1000,500,100,1.57,20'\"")
    print()
    print("2. 瞬移到模型位置:")
    print("   rostopic pub /xtdrone/plane_0/cmd std_msgs/String \"data: 'TELEPORT_TO_MODEL,model_name,height_offset,airspeed'\"")
    print("   例如: rostopic pub /xtdrone/plane_0/cmd std_msgs/String \"data: 'TELEPORT_TO_MODEL,person_yellow,50,18'\"")
    print()
    print("3. 智能瞬移到点(自动计算航向):")
    print("   rostopic pub /xtdrone/plane_0/cmd std_msgs/String \"data: 'TELEPORT_TO_POINT,x,y,z,airspeed'\"")
    print("   例如: rostopic pub /xtdrone/plane_0/cmd std_msgs/String \"data: 'TELEPORT_TO_POINT,1500,300,80,22'\"")
    print()
    print("4. 固定翼特殊操作:")
    print("   - 起飞: rostopic pub /xtdrone/plane_0/cmd std_msgs/String \"data: 'PLANE_TAKEOFF'\"")
    print("   - 巡航: rostopic pub /xtdrone/plane_0/cmd std_msgs/String \"data: 'PLANE_CRUISE'\"")
    print("   - 盘旋: rostopic pub /xtdrone/plane_0/cmd std_msgs/String \"data: 'PLANE_LOITER'\"")
    print("   - 降落: rostopic pub /xtdrone/plane_0/cmd std_msgs/String \"data: 'PLANE_LAND'\"")
    print()
    print("5. 基本控制:")
    print("   - 解锁: rostopic pub /xtdrone/plane_0/cmd std_msgs/String \"data: 'ARM'\"")
    print("   - 上锁: rostopic pub /xtdrone/plane_0/cmd std_msgs/String \"data: 'DISARM'\"")
    print("   - 模式: rostopic pub /xtdrone/plane_0/cmd std_msgs/String \"data: 'OFFBOARD'\"")
    print("="*60)


def main():
    """主函数"""
    if len(sys.argv) < 3:
        print("错误: 参数不足!")
        print_usage()
        return
    
    try:
        print("正在启动固定翼瞬移控制...")
        communication = PlaneTeleport(sys.argv[1], sys.argv[2])
        print("固定翼瞬移控制已启动!")
        print_usage()
        communication.start()
    except rospy.ROSInterruptException:
        print("ROS node interrupted")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == '__main__':
    main()
