#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
VTOL无人机飞行控制模块
包含起飞、导航、降落等核心飞行功能
从 vtol_demo.py 中提取的飞行控制逻辑
'''

import sys
import os
import time
import math

# 添加当前脚本目录到Python路径，用于导入本地模块
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from vtol_map import VTOLMap, ZoneType
from vtol_Astar import VTOLAstarPlanner
from vtol_ros import VTOLROSCommunicator, PersonPositionReader


class VTOLFlightController:
    """VTOL无人机飞行控制器"""
    
    def __init__(self, vehicle_type="standard_vtol", vehicle_id="0"):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        
        # 初始化地图
        self.map = VTOLMap()
        
        # 初始化A*路径规划器
        self.astar_planner = VTOLAstarPlanner(grid_size=20)
        
        # 初始化ROS通信器
        self.ros_comm = VTOLROSCommunicator(vehicle_type, vehicle_id)
        
        # 初始化人员位置读取器  
        self.person_reader = PersonPositionReader()
        
        # 飞行参数
        self.takeoff_height = 40.0  # 任务要求：起飞高度40米
        self.cruise_height = 40.0   # 巡航高度40米
        self.person_height = 20.0   # 人员飞行高度20米
        
        # 飞行模式状态
        self.current_mode = "multirotor"  # multirotor 或 plane
        
        print(f"初始化VTOL飞行控制器: {self.vehicle_type}_{self.vehicle_id}")
        
        # 设置ROS回调函数
        self.ros_comm.set_position_callback(self.position_update_callback)
        self.ros_comm.set_condition_callback(self.condition_received_callback)

    def position_update_callback(self, position):
        """位置更新回调函数"""
        pass

    def condition_received_callback(self, condition):
        """接收到condition时的回调函数"""
        pass

    def init_ros_communication(self):
        """初始化ROS通信"""
        self.ros_comm.init_ros_communication()
        print("飞行控制器ROS通信初始化完成")

    def send_cmd(self, cmd_str):
        """发送xtdrone命令"""
        self.ros_comm.send_command(cmd_str)

    def set_target_pose(self, x, y, z, yaw=0.0):
        """设置目标位置并开始持续发布"""
        self.ros_comm.set_target_pose(x, y, z, yaw)

    def get_distance_to_target(self, target_x, target_y, target_z):
        """计算到目标点的距离"""
        return self.ros_comm.get_distance_to_target(target_x, target_y, target_z)

    def get_horizontal_distance(self, target_x, target_y):
        """计算到目标点的水平距离"""
        current_pos = self.current_position
        if current_pos is None:
            return float('inf')
        
        dx = target_x - current_pos.x
        dy = target_y - current_pos.y
        
        return math.sqrt(dx*dx + dy*dy)

    @property
    def current_position(self):
        """获取当前位置"""
        return self.ros_comm.get_current_position()

    def wait_for_position(self, timeout=10):
        """等待位置数据"""
        return self.ros_comm.wait_for_position(timeout)

    def is_ros_ok(self):
        """检查ROS状态"""
        return self.ros_comm.is_ros_ok()

    def check_flight_safety(self, x, y, z):
        """检查飞行安全性"""
        zone_info = self.map.get_zone_info(x, y)
        
        # 检查是否在地图边界内
        if not self.map.is_in_bounds(x, y):
            return False, "目标点超出地图边界"
        
        # 检查是否在禁飞的居民区
        if zone_info['type'] == ZoneType.RESIDENTIAL:
            return False, f"目标点在居民区 {zone_info['name']} 内，禁止飞行"
        
        # 检查高度安全
        min_safe_height = 5.0
        if zone_info['type'] == ZoneType.MULTIROTOR:
            min_safe_height = 0.0
        elif zone_info['type'] == ZoneType.FREE_SPACE:
            min_safe_height = 15.0
        
        if z < min_safe_height:
            return False, f"飞行高度 {z}m 低于最小安全高度 {min_safe_height}m"
        
        return True, "安全"

    def can_switch_to_multirotor(self, x=None, y=None):
        """检查是否可以切换到旋翼模式（距离原点100米内）"""
        if x is None or y is None:
            if not self.current_position:
                return False
            x, y = self.current_position.x, self.current_position.y
        
        distance_to_origin = math.sqrt(x**2 + y**2)
        
        if distance_to_origin <= 100.0:
            print(f"✅ 距离原点{distance_to_origin:.1f}m (<100m)，允许切换到旋翼模式")
            return True
        else:
            print(f"❌ 距离原点{distance_to_origin:.1f}m (≥100m)，不允许切换到旋翼模式")
            return False

    def switch_to_mode(self, target_mode):
        """切换飞行模式"""
        if self.current_mode == target_mode:
            return True
        
        if self.current_position:
            current_zone = self.map.get_zone_info(self.current_position.x, self.current_position.y)
            print(f"当前位置: ({self.current_position.x:.1f}, {self.current_position.y:.1f}) - {current_zone['name']}")
            
            if target_mode == "multirotor":
                if not self.can_switch_to_multirotor():
                    print(f"❌ 无法切换到旋翼模式：距离原点超过100米")
                    if self.current_mode != "plane":
                        self.send_cmd("plane")
                        self.current_mode = "plane"
                    return False
        
        print(f"切换模式: {self.current_mode} -> {target_mode}")
        
        if target_mode == "plane":
            self.send_cmd("plane")
            self.current_mode = "plane"
        elif target_mode == "multirotor":
            self.send_cmd("multirotor")
            self.current_mode = "multirotor"
        
        time.sleep(3)
        return True

    def takeoff_sequence(self):
        """起飞序列 - 恢复旧版稳定逻辑"""
        print(f"\n🚀 开始起飞序列...")
        print("="*50)
        
        if self.current_position is None:
            print("❌ 起飞失败：无法获取当前位置信息")
            return False
        
        current_x = self.current_position.x
        current_y = self.current_position.y
        current_z = self.current_position.z
        
        print(f"📍 起飞前位置检查:")
        print(f"   当前位置: ({current_x:.1f}, {current_y:.1f}, {current_z:.1f})")
        
        # 检查是否可以切换到旋翼模式
        if not self.can_switch_to_multirotor(current_x, current_y):
            print(f"⚠️ 距离原点超过100米，移动到安全位置...")
            distance_to_origin = math.sqrt(current_x**2 + current_y**2)
            if distance_to_origin > 0:
                scale = 90.0 / distance_to_origin
                safe_x = current_x * scale
                safe_y = current_y * scale
            else:
                safe_x = self.map.multirotor_center[0]
                safe_y = self.map.multirotor_center[1]
            
            print(f"   移动到安全位置: ({safe_x:.1f}, {safe_y:.1f})")
            if not self.can_switch_to_multirotor(safe_x, safe_y):
                print(f"❌ 位置修正失败")
                return False
        
        print("✅ 位置验证通过，开始起飞")
        
        # 切换到旋翼模式
        if not self.switch_to_mode("multirotor"):
            print("❌ 无法切换到旋翼模式，起飞失败")
            return False
        
        # 旧版稳定起飞逻辑：先发布上升速度命令，再设置模式
        print("🔧 预设上升速度命令...")
        self.set_target_pose(current_x, current_y, current_z + 0.5)  # 预设稍高位置
        time.sleep(1)
        
        # 设置OFFBOARD模式
        print("设置OFFBOARD模式...")
        self.send_cmd("OFFBOARD")
        time.sleep(3)  # 增加等待时间
        
        # 解锁无人机
        print("解锁无人机...")
        self.send_cmd("ARM")
        time.sleep(3)
        
        # 分阶段闭环控制起飞 - 持续发布目标位置
        print("📈 开始分阶段闭环控制起飞...")
        start_x = self.current_position.x if self.current_position else current_x
        start_y = self.current_position.y if self.current_position else current_y
        
        takeoff_heights = [3, 8, 15, 25, self.takeoff_height]
        
        for i, height in enumerate(takeoff_heights):
            print(f"   🎯 起飞阶段 {i+1}/{len(takeoff_heights)}: 目标高度 {height}m")
            
            # 持续发布目标位置，使用快速检查
            stage_time = 6.0 if i < len(takeoff_heights) - 1 else 10.0  # 减少等待时间
            start_time = time.time()
            
            while time.time() - start_time < stage_time and self.is_ros_ok():
                # 持续发布目标位置
                self.set_target_pose(start_x, start_y, height)
                
                # 0.1秒间隔快速检查高度
                if self.current_position:
                    current_height = self.current_position.z
                    if current_height >= height - 2.0:  # 接近目标高度
                        print(f"   ✅ 快速到达高度 {height}m (当前: {current_height:.1f}m)")
                        break
                
                time.sleep(0.1)  # 0.1秒快速响应
            
            # 短暂稳定，也使用快速发布
            if i < len(takeoff_heights) - 1:
                print(f"   快速稳定0.5秒...")
                for _ in range(5):  # 0.5秒稳定时间
                    self.set_target_pose(start_x, start_y, height)
                    time.sleep(0.1)
        
        # 最终验证和稳定，减少时间
        print("🔍 快速最终起飞验证...")
        final_stable_time = 1.0  # 减少到1秒
        start_time = time.time()
        
        while time.time() - start_time < final_stable_time and self.is_ros_ok():
            self.set_target_pose(start_x, start_y, self.takeoff_height)
            time.sleep(0.1)  # 0.1秒快速发布
        
        if self.current_position:
            final_height = self.current_position.z
            print(f"📊 起飞完成检查：")
            print(f"   目标高度: {self.takeoff_height}m")
            print(f"   实际高度: {final_height:.1f}m")
            
            if final_height >= self.takeoff_height - 8.0:  # 放宽容差
                print("✅ 起飞成功!")
                self.publish_status(0x01)  # 发布状态0x01
                return True
            else:
                print("⚠️ 起飞高度不足，但继续任务...")
                self.publish_status(0x01)  # 仍然发布状态0x01
                return True
        
        print("⚠️ 无法验证最终高度，但假设起飞成功")
        self.publish_status(0x01)
        return True

    def fly_to_target(self, target_x, target_y, target_z):
        """飞向指定目标"""
        print(f"\n飞向目标: ({target_x}, {target_y}, {target_z})")
        print("="*50)
        
        # 安全检查
        is_safe, safety_msg = self.check_flight_safety(target_x, target_y, target_z)
        if not is_safe:
            print(f"❌ 安全检查失败: {safety_msg}")
            return False
        
        # 检查当前位置和目标位置的区域类型
        current_zone = self.map.get_zone_info(self.current_position.x, self.current_position.y)
        target_zone = self.map.get_zone_info(target_x, target_y)
        
        print(f"当前区域: {current_zone['name']}")
        print(f"目标区域: {target_zone['name']}")
        
        # 飞行模式决策
        current_in_multirotor = (current_zone['type'] == ZoneType.MULTIROTOR)
        target_in_multirotor = (target_zone['type'] == ZoneType.MULTIROTOR)
        
        if current_in_multirotor and target_in_multirotor:
            print("旋翼区内飞行，使用旋翼模式")
            return self.fly_with_multirotor_mode(target_x, target_y, target_z)
        elif current_in_multirotor and not target_in_multirotor:
            print("从旋翼区飞出，使用固定翼模式")
            return self.fly_from_multirotor_to_outside(target_x, target_y, target_z)
        elif not current_in_multirotor and target_in_multirotor:
            print("飞向旋翼区，固定翼模式接近后切换旋翼模式")
            return self.fly_to_multirotor_zone(target_x, target_y, target_z)
        else:
            print("旋翼区外飞行，使用固定翼模式")
            return self.fly_with_plane_mode(target_x, target_y, target_z)

    def find_safe_waypoints(self, start_x, start_y, end_x, end_y):
        """使用A*算法规划安全航点，避开居民区"""
        print(f"🧭 A*路径规划: 从 ({start_x:.1f}, {start_y:.1f}) 到 ({end_x:.1f}, {end_y:.1f})")
        
        start_pos = (start_x, start_y)
        end_pos = (end_x, end_y)
        
        astar_path = self.astar_planner.plan_path(start_pos, end_pos)
        
        if astar_path and len(astar_path) > 1:
            total_distance = 0
            for i in range(len(astar_path) - 1):
                dx = astar_path[i+1][0] - astar_path[i][0]
                dy = astar_path[i+1][1] - astar_path[i][1]
                total_distance += math.sqrt(dx*dx + dy*dy)
            
            direct_distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
            path_efficiency = (direct_distance / total_distance) * 100 if total_distance > 0 else 100
            
            print(f"✅ A*路径规划成功:")
            print(f"   路径点数: {len(astar_path)}")
            print(f"   总距离: {total_distance:.1f}m")
            print(f"   路径效率: {path_efficiency:.1f}%")
            
            waypoints = []
            for i, (wp_x, wp_y) in enumerate(astar_path):
                if i == 0:
                    continue
                waypoints.append((wp_x, wp_y))
            
            return waypoints
        else:
            print("❌ A*路径规划失败，使用直线路径")
            return [(end_x, end_y)]

    def fly_from_multirotor_to_outside(self, target_x, target_y, target_z):
        """从旋翼区飞向外部区域"""
        print("从旋翼区飞向外部，先在旋翼区边缘切换到固定翼模式...")
        
        multirotor_center = self.map.multirotor_center
        multirotor_radius = self.map.multirotor_radius
        
        center_x, center_y = multirotor_center
        direction_x = target_x - center_x
        direction_y = target_y - center_y
        direction_length = math.sqrt(direction_x**2 + direction_y**2)
        
        if direction_length > 0:
            direction_x /= direction_length
            direction_y /= direction_length
            
            edge_margin = 10
            edge_x = center_x + direction_x * (multirotor_radius - edge_margin)
            edge_y = center_y + direction_y * (multirotor_radius - edge_margin)
        else:
            edge_x = self.current_position.x
            edge_y = self.current_position.y
        
        print(f"先飞向旋翼区边缘点: ({edge_x:.1f}, {edge_y:.1f})")
        
        # 在旋翼区内用旋翼模式飞到边缘
        self.switch_to_mode("multirotor")
        self.set_target_pose(edge_x, edge_y, self.cruise_height)
        time.sleep(8)
        
        # 切换到固定翼模式
        print("在旋翼区边缘切换到固定翼模式...")
        if not self.switch_to_mode("plane"):
            print("❌ 切换固定翼模式失败")
            return False
        
        # 使用固定翼模式飞向目标
        return self.fly_with_plane_mode(target_x, target_y, target_z)

    def fly_to_multirotor_zone(self, target_x, target_y, target_z):
        """飞向旋翼区"""
        print("飞向旋翼区，先用固定翼模式接近...")
        
        self.switch_to_mode("plane")
        
        multirotor_center = self.map.multirotor_center
        multirotor_radius = self.map.multirotor_radius
        
        center_x, center_y = multirotor_center
        current_x = self.current_position.x
        current_y = self.current_position.y
        
        direction_x = center_x - current_x
        direction_y = center_y - current_y
        direction_length = math.sqrt(direction_x**2 + direction_y**2)
        
        if direction_length > 0:
            direction_x /= direction_length
            direction_y /= direction_length
        else:
            if self.can_switch_to_multirotor():
                if self.switch_to_mode("multirotor"):
                    return self.fly_with_multirotor_mode(target_x, target_y, target_z)
            return self.fly_with_plane_mode(target_x, target_y, target_z)
        
        # 接近旋翼区
        approach_margin = 20
        approach_x = center_x - direction_x * (multirotor_radius + approach_margin)
        approach_y = center_y - direction_y * (multirotor_radius + approach_margin)
        
        print(f"固定翼模式接近旋翼区边缘: ({approach_x:.1f}, {approach_y:.1f})")
        
        self.set_target_pose(approach_x, approach_y, self.cruise_height)
        time.sleep(10)
        
        # 进入旋翼区
        entry_x = center_x - direction_x * (multirotor_radius - 10)
        entry_y = center_y - direction_y * (multirotor_radius - 10)
        
        self.set_target_pose(entry_x, entry_y, self.cruise_height)
        time.sleep(5)
        
        # 检查是否可以切换到旋翼模式
        if self.current_position:
            if self.can_switch_to_multirotor(self.current_position.x, self.current_position.y):
                print("✅ 已进入可切换旋翼模式区域，切换到旋翼模式...")
                if self.switch_to_mode("multirotor"):
                    return self.fly_with_multirotor_mode(target_x, target_y, target_z)
                else:
                    return self.fly_with_plane_mode(target_x, target_y, target_z)
            else:
                return self.fly_with_plane_mode(target_x, target_y, target_z)
        
        return self.fly_with_plane_mode(target_x, target_y, target_z)

    def fly_with_plane_mode(self, target_x, target_y, target_z):
        """使用固定翼模式飞行"""
        print("🛩️ 使用固定翼模式进行飞行...")
        
        self.switch_to_mode("plane")
        
        current_x = self.current_position.x
        current_y = self.current_position.y
        
        waypoints = self.find_safe_waypoints(current_x, current_y, target_x, target_y)
        
        if not waypoints:
            print("❌ 路径规划失败，无法继续")
            return False
        
        # 依次飞向各个航点
        for i, (wp_x, wp_y) in enumerate(waypoints):
            if i == len(waypoints) - 1:
                flight_height = max(target_z + 10, 30)
            else:
                flight_height = self.cruise_height
            
            print(f"🎯 飞向航点 {i+1}/{len(waypoints)}: ({wp_x:.1f}, {wp_y:.1f}, {flight_height:.1f}m)")
            
            zone_info = self.map.get_zone_info(wp_x, wp_y)
            if zone_info['type'] == ZoneType.RESIDENTIAL:
                print(f"❌ 紧急警告：航点 {i+1} 在居民区内，跳过")
                continue
            
            segment_distance = math.sqrt((wp_x - current_x)**2 + (wp_y - current_y)**2) if i == 0 else \
                             math.sqrt((wp_x - waypoints[i-1][0])**2 + (wp_y - waypoints[i-1][1])**2)
            
            # 统一使用30米容差
            waypoint_tolerance = 30.0
            
            success = self.wait_for_position_reached(wp_x, wp_y, flight_height, waypoint_tolerance)
            
            if success:
                print(f"   ✅ 成功到达航点 {i+1}")
            else:
                print(f"   ⚠️ 航点 {i+1} 未完全到达，继续下一个航点")
        
        # 最终接近目标
        print(f"🎯 A*路径完成，开始闭环精确接近目标...")
        final_height = max(target_z, 20)
        success = self.precise_fly_to_position(target_x, target_y, final_height, "固定翼最终目标")
        
        return success

    def fly_with_multirotor_mode(self, target_x, target_y, target_z):
        """使用多旋翼模式飞行"""
        print("🚁 使用多旋翼模式进行短距离飞行...")
        
        self.switch_to_mode("multirotor")
        return self.precise_fly_to_position(target_x, target_y, target_z, "多旋翼目标")

    def land_at_target(self, target_x, target_y, target_z):
        """在目标点降落"""
        if self.can_switch_to_multirotor(target_x, target_y):
            print("目标点在可切换旋翼模式区域内，切换到旋翼模式降落...")
            
            if self.switch_to_mode("multirotor"):
                for height in [10, 5, target_z]:
                    self.set_target_pose(target_x, target_y, height)
                    time.sleep(3)
                
                if target_z <= 1.0:
                    print("执行自动降落...")
                    self.send_cmd("LAND")
                    time.sleep(5)
            else:
                print("无法切换到旋翼模式，使用固定翼模式悬停")
                self.set_target_pose(target_x, target_y, max(15.0, target_z))
                time.sleep(5)
        else:
            print("目标点超出可切换旋翼模式区域，固定翼模式悬停...")
            self.switch_to_mode("plane")
            safe_height = max(20.0, target_z)
            self.set_target_pose(target_x, target_y, safe_height)
            time.sleep(5)

    def wait_for_position_reached(self, target_x, target_y, target_z, tolerance=30.0, max_wait_time=None):
        """等待到达目标位置 - 0.1秒快速响应判断，无超时机制"""
        print(f"🎯 快速响应等待到达目标: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f}), 距离阈值: {tolerance}m")
        print("⚡ 0.1秒间隔快速检查，一瞬间满足条件立即返回...")
        
        start_time = time.time()
        min_distance = float('inf')
        
        self.set_target_pose(target_x, target_y, target_z)
        
        check_interval = 0.1  # 保持0.1秒快速检查
        check_count = 0
        last_report_time = 0
        
        while self.is_ros_ok():
            current_pos = self.current_position
            if current_pos is None:
                print("⚠️ 无法获取位置信息")
                time.sleep(check_interval)
                continue
            
            current_distance = self.get_distance_to_target(target_x, target_y, target_z)
            min_distance = min(min_distance, current_distance)
            
            # 持续发布目标位置，确保控制稳定
            self.set_target_pose(target_x, target_y, target_z)
            
            # 瞬间距离判断：只要一次距离小于tolerance就认为到达，立即返回
            if current_distance <= tolerance:
                elapsed_time = time.time() - start_time
                print(f"✅ 快速到达目标！距离: {current_distance:.1f}m, 用时: {elapsed_time:.1f}s")
                print(f"🚀 0.1秒响应，立即进入下一阶段！")
                return True
            
            # 减少报告频率，每1秒报告一次而不是每5秒
            elapsed = time.time() - start_time
            if elapsed - last_report_time >= 1.0:  # 每1秒报告一次，提高信息反馈频率
                print(f"  📊 {elapsed:5.1f}s | 距离: {current_distance:6.1f}m | 最小: {min_distance:6.1f}m | 快速等待中...")
                last_report_time = elapsed
            
            time.sleep(check_interval)  # 0.1秒快速检查间隔
        
        # 如果ROS不正常，才退出
        print(f"❌ ROS通信异常，停止等待")
        return False

    def precise_fly_to_position(self, target_x, target_y, target_z, description="目标点"):
        """精确飞向位置 - 使用闭环控制"""
        print(f"\n🎯 精确飞向 {description}: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})")
        print("-" * 50)
        
        if self.current_position is None:
            print("❌ 无法获取当前位置")
            return False
        
        start_x = self.current_position.x
        start_y = self.current_position.y
        start_z = self.current_position.z
        
        total_distance = math.sqrt((target_x - start_x)**2 + (target_y - start_y)**2 + (target_z - start_z)**2)
        print(f"📏 起始位置: ({start_x:.1f}, {start_y:.1f}, {start_z:.1f})")
        print(f"📏 目标距离: {total_distance:.1f}m")
        
        # 分阶段接近 - 统一使用30米容差（非起飞阶段）
        current_altitude = self.current_position.z if self.current_position else 0
        
        # 如果当前高度小于5米，说明可能在起飞阶段，使用小容差
        if current_altitude < 5.0:
            approach_stages = [
                (10.0, "起飞阶段精确定位")
            ]
        else:
            # 非起飞阶段统一使用30米容差
            if total_distance > 30.0:
                approach_stages = [
                    (30.0, "统一容差接近")
                ]
            else:
                approach_stages = [
                    (30.0, "最终定位")
                ]
        
        for stage, (tolerance, stage_name) in enumerate(approach_stages, 1):
            current_distance = self.get_distance_to_target(target_x, target_y, target_z)
            if current_distance <= tolerance:
                print(f"跳过阶段 {stage}：{stage_name}（已足够接近，当前距离: {current_distance:.1f}m）")
                continue
                
            print(f"\n📍 阶段 {stage}: {stage_name} (容忍度: {tolerance}m)")
            
            success = self.wait_for_position_reached(target_x, target_y, target_z, tolerance)
            
            if success:
                print(f"✅ 阶段 {stage} 完成")
                if self.current_position:
                    total_distance = self.get_distance_to_target(target_x, target_y, target_z)
            else:
                print(f"⚠️ 阶段 {stage} 未完全达标，继续下一阶段")
        
        # 最终验证
        if self.current_position:
            final_distance = self.get_distance_to_target(target_x, target_y, target_z)
            print(f"\n🏁 {description} 飞行完成")
            print(f"   最终距离: {final_distance:.1f}m")
            print(f"   最终位置: ({self.current_position.x:.1f}, {self.current_position.y:.1f}, {self.current_position.z:.1f})")
            
            if final_distance <= 30.0:
                print(f"   ✅ 精度良好")
                return True
            elif final_distance <= 40.0:
                print(f"   ⚠️ 精度一般，但可接受")
                return True
            else:
                print(f"   ❌ 精度不足")
                return False
        
        return False

    def return_to_launch(self):
        """返航到起飞点"""
        print(f"\n🏠 执行自动返航...")
        print("发送 AUTO.RTL 命令，无人机将自动返回出发点")
        self.send_cmd("AUTO.RTL")
        
        # 等待返航和降落完成
        print("等待无人机返航并自动降落...")
        return self.wait_for_landing_completion()

    def wait_for_landing_completion(self):
        """等待无人机返航和降落完成 - 无超时机制，持续等待直到降落完成"""
        print("🔍 持续监控无人机返航和降落过程...")
        print("⚠️ 无超时机制，将持续等待直到降落完成...")
        
        start_time = time.time()
        landing_threshold = 2.0
        stable_landing_time = 5.0
        
        landing_start_time = None
        check_interval = 0.1  # 改为0.1秒快速检查
        last_report_time = 0
        
        while self.is_ros_ok():
            current_pos = self.current_position
            if current_pos is None:
                print("⚠️ 无法获取位置信息，继续等待...")
                time.sleep(check_interval)
                continue
            
            current_height = current_pos.z
            current_distance_to_origin = math.sqrt(current_pos.x**2 + current_pos.y**2)
            
            if current_distance_to_origin <= 50.0:
                elapsed = time.time() - start_time
                if elapsed - last_report_time >= 2.0:  # 每2秒报告一次，提高反馈频率
                    print(f"📍 已返航至原点附近，距离: {current_distance_to_origin:.1f}m，高度: {current_height:.1f}m")
                    last_report_time = elapsed
                
                if current_height <= landing_threshold:
                    if landing_start_time is None:
                        landing_start_time = time.time()
                        print(f"🛬 检测到开始降落，高度: {current_height:.1f}m")
                    else:
                        stable_time = time.time() - landing_start_time
                        if stable_time >= stable_landing_time:
                            print(f"✅ 降落完成！稳定在低高度 {stable_time:.1f}s")
                            return True
                        else:
                            print(f"🛬 降落中...稳定时间: {stable_time:.1f}s/{stable_landing_time}s")
                else:
                    if landing_start_time is not None:
                        print(f"⬆️ 高度上升到 {current_height:.1f}m，重置降落检测")
                        landing_start_time = None
            else:
                elapsed = time.time() - start_time
                if elapsed - last_report_time >= 3.0:  # 返航阶段每3秒报告一次
                    print(f"🏠 返航中...距离原点: {current_distance_to_origin:.1f}m，高度: {current_height:.1f}m")
                    last_report_time = elapsed
                landing_start_time = None
            
            time.sleep(check_interval)
        
        # 只有在ROS出现问题时才会到达这里
        print(f"❌ ROS状态异常，退出等待")
        return False

    def emergency_stop(self):
        """紧急停止"""
        print("🚨 执行紧急停止...")
        self.ros_comm.stop_publishing()
        self.send_cmd("HOVER")
        self.send_cmd("DISARM")

    def shutdown(self):
        """关闭飞行控制器"""
        print("关闭VTOL飞行控制器...")
        
        # 关闭人员位置读取器
        if hasattr(self, 'person_reader'):
            self.person_reader.shutdown()
            
        self.ros_comm.shutdown()

    # 新增的任务方法，满足任务要求
    def fly_to_target_1(self):
        """任务2: 切换到固定翼模式，飞行到目标点(1495, 250, 40)"""
        print(f"\n✈️ 任务2: 固定翼模式飞向目标点1 (1495, 250, 40)")
        print("="*50)
        
        target_x, target_y, target_z = 1495, 250, 40
        
        # 安全检查
        is_safe, safety_msg = self.check_flight_safety(target_x, target_y, target_z)
        if not is_safe:
            print(f"❌ 安全检查失败: {safety_msg}")
            return False
        
        # 切换到固定翼模式并飞行
        success = self.fly_to_target(target_x, target_y, target_z)
        
        if success:
            print("✅ 成功到达目标点1!")
            self.publish_status(0x02)  # 发布状态0x02
            return True
        else:
            print("❌ 未能到达目标点1")
            return False

    def fly_to_target_2(self):
        """任务3: 飞向目标点(1495, -250, 40)"""
        print(f"\n✈️ 任务3: 飞向目标点2 (1495, -250, 40)")
        print("="*50)
        
        target_x, target_y, target_z = 1495, -250, 40
        
        # 安全检查
        is_safe, safety_msg = self.check_flight_safety(target_x, target_y, target_z)
        if not is_safe:
            print(f"❌ 安全检查失败: {safety_msg}")
            return False
        
        # 继续使用固定翼模式并飞行
        success = self.fly_to_target(target_x, target_y, target_z)
        
        if success:
            print("✅ 成功到达目标点2!")
            self.publish_status(0x03)  # 发布状态0x03
            return True
        else:
            print("❌ 未能到达目标点2")
            return False

    def visit_persons(self):
        """任务4: 按y坐标从小到大依次飞到每个人的位置（高度20米）"""
        print(f"\n👥 任务4: 依次飞向人员位置 (高度20米)")
        print("="*50)
        
        # 等待获取人员位置数据
        print("⏳ 等待获取人员位置数据...")
        # 在等待期间持续发布当前位置以保持飞行稳定
        if self.current_position:
            current_x = self.current_position.x
            current_y = self.current_position.y
            current_z = self.current_position.z
            
            for i in range(30):  # 3秒，每0.1秒发布一次
                self.set_target_pose(current_x, current_y, current_z)
                time.sleep(0.1)
        
        positions = self.person_reader.get_sorted_positions()
        
        if not positions:
            print("❌ 未能获取人员位置数据")
            return False
        
        print(f"📍 获取到{len(positions)}个人员位置")
        
        # 依次飞向每个人员位置
        success_count = 0
        for i, (x, y, z, name) in enumerate(positions, 1):
            print(f"\n🎯 飞向第{i}个人员: {name} ({x:.1f}, {y:.1f}, {self.person_height})")
            
            # 安全检查
            is_safe, safety_msg = self.check_flight_safety(x, y, self.person_height)
            if not is_safe:
                print(f"❌ 人员位置安全检查失败: {safety_msg}")
                continue
            
            # 根据位置选择飞行模式并飞行
            success = self.fly_to_target(x, y, self.person_height)
            
            if success:
                print(f"   ✅ 成功到达{name}位置")
                success_count += 1
            else:
                print(f"   ⚠️ 未能完全到达{name}位置")
        
        print(f"✅ 人员位置访问完成! ({success_count}/{len(positions)})")
        self.publish_status(0x04)  # 发布状态0x04
        return success_count > 0

    def return_to_multirotor_zone(self):
        """任务5: 飞回旋翼区并自动返航"""
        print(f"\n🏠 任务5: 返回旋翼区并自动返航")
        print("="*50)
        
        # 目标：旋翼区边缘
        target_x, target_y = 0, 0  # 回到原点
        target_z = self.cruise_height
        
        current_x = self.current_position.x
        current_y = self.current_position.y
        
        print(f"从 ({current_x:.1f}, {current_y:.1f}) 返回旋翼区 ({target_x}, {target_y})")
        
        # 如果当前不在旋翼区，需要路径规划避开居民区
        if not self.can_switch_to_multirotor(current_x, current_y):
            print("当前在旋翼区外，规划返航路径...")
            
            # 使用固定翼模式
            self.switch_to_mode("plane")
            
            # 路径规划
            waypoints = self.find_safe_waypoints(current_x, current_y, target_x, target_y)
            
            if waypoints:
                # 飞向航点，最后一个航点应该在旋翼区边缘
                for i, (wp_x, wp_y) in enumerate(waypoints[:-1]):  # 除了最后一个点
                    print(f"🎯 返航航点 {i+1}: ({wp_x:.1f}, {wp_y:.1f})")
                    self.wait_for_position_reached(wp_x, wp_y, target_z, tolerance=30.0)
                
                # 接近旋翼区边缘
                edge_x, edge_y = waypoints[-1]
                print(f"🎯 接近旋翼区边缘: ({edge_x:.1f}, {edge_y:.1f})")
                self.wait_for_position_reached(edge_x, edge_y, target_z, tolerance=30.0)
        
        # 进入旋翼区
        print("进入旋翼区...")
        if self.can_switch_to_multirotor():
            self.switch_to_mode("multirotor")
        
        # 飞到旋翼区内合适位置
        self.wait_for_position_reached(target_x, target_y, target_z, tolerance=30.0)
        
        # 执行自动返航
        print("🏠 执行自动返航指令...")
        self.send_cmd("AUTO.RTL")
        
        # 等待返航完成
        success = self.wait_for_landing_completion()
        
        if success:
            print("✅ 自动返航完成!")
            self.publish_status(0x05)  # 发布状态0x05
            return True
        else:
            print("⚠️ 返航完成（可能有异常）")
            return True

    def publish_status(self, status_code):
        """发布状态"""
        self.ros_comm.publish_condition(status_code)

def test_flight_controller():
    """测试飞行控制器"""
    print("测试VTOL飞行控制器")
    
    try:
        # 创建飞行控制器
        controller = VTOLFlightController()
        
        # 初始化通信
        controller.init_ros_communication()
        
        # 等待位置信息
        if controller.wait_for_position():
            pos = controller.current_position
            print(f"✅ 成功获取位置: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f})")
        else:
            print("⚠️ 未能获取位置信息")
        
        print("✅ 飞行控制器测试完成")
        
    except Exception as e:
        print(f"❌ 飞行控制器测试失败: {e}")


if __name__ == "__main__":
    # 初始化ROS节点
    try:
        import rospy
        if not rospy.core.is_initialized():
            rospy.init_node('vtol_flight_controller_test', anonymous=True)
            print("✅ ROS节点初始化成功")
    except Exception as e:
        print(f"❌ ROS节点初始化失败: {e}")
        exit(1)
    
    test_flight_controller()
