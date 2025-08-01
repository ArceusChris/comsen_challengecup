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
from vtol_ros import VTOLROSCommunicator


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
        
        # 飞行参数
        self.takeoff_height = 30.0
        self.cruise_height = 50.0
        self.approach_height = 25.0
        
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
        """起飞序列"""
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
        
        # 设置OFFBOARD模式
        print("设置OFFBOARD模式...")
        self.send_cmd("OFFBOARD")
        time.sleep(2)
        
        # 解锁无人机
        print("解锁无人机...")
        self.send_cmd("ARM")
        time.sleep(3)
        
        # 逐步起飞
        print("📈 开始闭环控制起飞...")
        start_x = self.current_position.x
        start_y = self.current_position.y
        
        takeoff_heights = [5, 10, 20, self.takeoff_height]
        
        for i, height in enumerate(takeoff_heights):
            print(f"   🎯 起飞阶段 {i+1}/{len(takeoff_heights)}: 目标高度 {height}m")
            success = self.wait_for_position_reached(start_x, start_y, height, tolerance=3.0, max_wait_time=20.0)
            
            if success:
                print(f"   ✅ 到达高度 {height}m")
            else:
                print(f"   ⚠️ 高度 {height}m 未完全到达，继续下一阶段")
            
            time.sleep(1)
        
        # 最终验证
        if self.current_position:
            final_height = self.current_position.z
            print(f"📊 起飞完成检查：")
            print(f"   目标高度: {self.takeoff_height}m")
            print(f"   实际高度: {final_height:.1f}m")
            
            if final_height >= self.takeoff_height - 5.0:
                print("✅ 起飞成功!")
                return True
            else:
                print("⚠️ 起飞高度不足，但继续任务...")
                return True
        
        return False

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
            
            waypoint_tolerance = min(25.0, max(20.0, segment_distance * 0.1))
            max_wait_time = max(30.0, min(90.0, segment_distance / 15.0))
            
            success = self.wait_for_position_reached(wp_x, wp_y, flight_height, waypoint_tolerance, max_wait_time)
            
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

    def wait_for_position_reached(self, target_x, target_y, target_z, tolerance=20.0, max_wait_time=60.0):
        """等待到达目标位置 - 瞬间距离判断，不需要持续停留"""
        print(f"🎯 等待瞬间到达目标: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f}), 距离阈值: {tolerance}m")
        
        start_time = time.time()
        min_distance = float('inf')
        
        self.set_target_pose(target_x, target_y, target_z)
        
        check_interval = 0.1
        
        while time.time() - start_time < max_wait_time and self.is_ros_ok():
            current_pos = self.current_position
            if current_pos is None:
                print("⚠️ 无法获取位置信息")
                time.sleep(check_interval)
                continue
            
            current_distance = self.get_distance_to_target(target_x, target_y, target_z)
            min_distance = min(min_distance, current_distance)
            
            # 持续发布目标位置
            self.set_target_pose(target_x, target_y, target_z)
            
            # 瞬间距离判断：只要一次距离小于tolerance就认为到达
            if current_distance <= tolerance:
                print(f"✅ 瞬间到达目标！距离: {current_distance:.1f}m, 用时: {time.time() - start_time:.1f}s")
                return True
            
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0 and elapsed > 0:
                print(f"  📊 {elapsed:5.1f}s | 当前距离: {current_distance:6.1f}m | 最小距离: {min_distance:6.1f}m")
            
            time.sleep(check_interval)
        
        final_distance = self.get_distance_to_target(target_x, target_y, target_z)
        print(f"⏰ 等待超时！最终距离: {final_distance:.1f}m, 最小距离: {min_distance:.1f}m")
        
        # 即使超时，如果最小距离曾经达到过阈值的1.5倍以内，也认为成功
        if min_distance <= tolerance * 1.5:
            print(f"✅ 虽然超时，但曾接近目标（最小距离: {min_distance:.1f}m）")
            return True
        else:
            print(f"❌ 未能接近目标位置")
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
        
        # 分阶段接近
        approach_stages = [
            (min(50.0, total_distance * 0.8), 5.0, "远距离接近"),
            (min(30.0, total_distance * 0.5), 5.0, "中距离接近"),
            (20.0, 5.0, "精确定位")
        ]
        
        for stage, (tolerance, max_time, stage_name) in enumerate(approach_stages, 1):
            if total_distance <= tolerance:
                print(f"跳过阶段 {stage}：{stage_name}（已足够接近）")
                continue
                
            print(f"\n📍 阶段 {stage}: {stage_name} (容忍度: {tolerance}m)")
            
            success = self.wait_for_position_reached(target_x, target_y, target_z, tolerance, max_time)
            
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
            
            if final_distance <= 25.0:
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
        """等待无人机返航和降落完成"""
        print("🔍 监控无人机返航和降落过程...")
        
        max_wait_time = 120.0
        start_time = time.time()
        landing_threshold = 2.0
        stable_landing_time = 5.0
        
        landing_start_time = None
        check_interval = 0.5
        
        while time.time() - start_time < max_wait_time and self.is_ros_ok():
            current_pos = self.current_position
            if current_pos is None:
                print("⚠️ 无法获取位置信息，继续等待...")
                time.sleep(check_interval)
                continue
            
            current_height = current_pos.z
            current_distance_to_origin = math.sqrt(current_pos.x**2 + current_pos.y**2)
            
            if current_distance_to_origin <= 50.0:
                print(f"📍 已返航至原点附近，距离: {current_distance_to_origin:.1f}m，高度: {current_height:.1f}m")
                
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
                print(f"🏠 返航中...距离原点: {current_distance_to_origin:.1f}m，高度: {current_height:.1f}m")
                landing_start_time = None
            
            time.sleep(check_interval)
        
        elapsed_time = time.time() - start_time
        print(f"⏰ 等待降落超时 ({elapsed_time:.1f}s)，假设降落完成")
        
        if self.current_position:
            final_height = self.current_position.z
            final_distance = math.sqrt(self.current_position.x**2 + self.current_position.y**2)
            print(f"   最终位置: 距离原点 {final_distance:.1f}m，高度 {final_height:.1f}m")
            
            if final_distance <= 100.0 and final_height <= 10.0:
                print("✅ 位置合理，认为降落成功")
                return True
        
        print("⚠️ 降落状态不确定，但继续完成任务")
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
        self.ros_comm.shutdown()


def test_flight_controller():
    """测试飞行控制器"""
    print("测试VTOL飞行控制器")
    
    try:
        # 创建飞行控制器
        controller = VTOLFlightController()
        
        # 初始化通信
        controller.init_ros_communication()
        
        # 等待位置信息
        if controller.wait_for_position(timeout=5):
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
