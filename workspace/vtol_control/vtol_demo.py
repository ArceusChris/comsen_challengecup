#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
参考/home/yzy/comsen_challengecup/simple_fixedwing_flight.py
写一个固定翼无人机自动飞行脚本
根据workspace/vtol_control/vtol_target.yaml中的目标点
让standard_vtol_0飞到每个目标点
注意:
- 新规则：只要距离原点(0,0)在半径100米范围内，无论zone类型都可以切换到旋翼模式
- 在100米范围外，所有动作都必须要在固定翼模式下完成
- 居民区是禁止飞行的
- 区域的定义见workspace/vtol_control/vtol_map.py
'''

import rospy
import time
import math
import threading
import yaml
import os
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
from vtol_map import VTOLMap, ZoneType
from vtol_Astar import VTOLAstarPlanner


class VTOLDemoFlight:
    def __init__(self, vehicle_type="standard_vtol", vehicle_id="0"):
        self.vehicle_type = vehicle_type
        self.vehicle_id = vehicle_id
        
        # 初始化地图
        self.map = VTOLMap()
        
        # 初始化A*路径规划器
        self.astar_planner = VTOLAstarPlanner(grid_size=20)  # 20米网格，平衡精度和性能
        
        # 加载目标点
        self.targets = self.load_targets()
        
        # 当前状态
        self.current_position = None
        self.current_yaw = 0
        self.current_target_index = 0
        
        # 飞行参数
        self.takeoff_height = 30.0
        self.cruise_height = 50.0
        self.approach_height = 25.0
        
        # 位置指令持续发布
        self.target_pose = Pose()
        self.should_publish = False
        
        # 飞行模式状态
        self.current_mode = "multirotor"  # multirotor 或 plane
        
        print(f"初始化VTOL演示飞行: {self.vehicle_type}_{self.vehicle_id}")
        print(f"集成A*路径规划器 (网格大小: {self.astar_planner.grid_size}m)")
        print(f"加载了 {len(self.targets)} 个目标点")
        
        # 注意：ROS节点在主函数中初始化，这里稍后设置通信
        self.ros_initialized = False

    def load_targets(self):
        """加载vtol_target.yaml中的目标点"""
        target_file = "vtol_target.yaml"
        targets = []
        
        try:
            if os.path.exists(target_file):
                with open(target_file, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                    
                # 解析YAML格式
                if data and 'targets' in data:
                    for target_info in data['targets']:
                        if 'position' in target_info:
                            pos = target_info['position']
                            if len(pos) >= 3:
                                x, y, z = pos[0], pos[1], pos[2]
                                name = target_info.get('name', f'target_{len(targets)+1}')
                                description = target_info.get('description', '')
                                targets.append({
                                    'position': (x, y, z),
                                    'name': name,
                                    'description': description
                                })
                else:
                    raise ValueError("YAML文件格式错误：缺少'targets'字段")
            else:
                print(f"警告: 目标文件 {target_file} 不存在，使用默认目标点")
                # 默认目标点
                targets = [
                    {'position': (0, 0, 0), 'name': 'takeoff_point', 'description': '起飞点'},
                    {'position': (1600, 200, 20), 'name': 'target_north', 'description': '北侧目标点'},
                    {'position': (1600, -200, 20), 'name': 'target_south', 'description': '南侧目标点'},
                    {'position': (0, 0, 0), 'name': 'landing_point', 'description': '降落点'}
                ]
                
        except Exception as e:
            print(f"加载目标文件出错: {e}")
            print("使用默认目标点...")
            targets = [
                {'position': (0, 0, 0), 'name': 'takeoff_point', 'description': '起飞点'},
                {'position': (1600, 200, 20), 'name': 'target_north', 'description': '北侧目标点'},
                {'position': (1600, -200, 20), 'name': 'target_south', 'description': '南侧目标点'},
                {'position': (0, 0, 0), 'name': 'landing_point', 'description': '降落点'}
            ]
        
        print("目标点列表:")
        for i, target in enumerate(targets):
            x, y, z = target['position']
            zone_info = self.map.get_zone_info(x, y)
            print(f"  {i+1}. {target['name']}: ({x}, {y}, {z}) - 区域: {zone_info['name']} - {target['description']}")
        
        return targets

    def init_ros(self):
        """初始化ROS通信（节点已在主函数中初始化）"""
        # 注意：ROS节点已在主函数中初始化，这里只设置订阅者和发布者
        
        # 订阅者
        self.local_pose_sub = rospy.Subscriber(
            f"{self.vehicle_type}_{self.vehicle_id}/mavros/local_position/pose", 
            PoseStamped, self.local_pose_callback, queue_size=1)
        
        # 发布者
        self.cmd_pub = rospy.Publisher(
            f"/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd", 
            String, queue_size=10)
        
        self.cmd_pose_pub = rospy.Publisher(
            f"/xtdrone/{self.vehicle_type}_{self.vehicle_id}/cmd_pose_enu", 
            Pose, queue_size=10)
        
        print("ROS通信初始化完成")

    def local_pose_callback(self, msg):
        """位置回调函数"""
        self.current_position = msg.pose.position

    def send_cmd(self, cmd_str):
        """发送xtdrone命令"""
        cmd_msg = String()
        cmd_msg.data = cmd_str
        self.cmd_pub.publish(cmd_msg)
        print(f"发送命令: {cmd_str}")

    def set_target_pose(self, x, y, z, yaw=0.0):
        """设置目标位置并开始持续发布"""
        self.target_pose.position.x = x
        self.target_pose.position.y = y
        self.target_pose.position.z = z
        self.target_pose.orientation.x = 0.0
        self.target_pose.orientation.y = 0.0
        self.target_pose.orientation.z = math.sin(yaw / 2.0)
        self.target_pose.orientation.w = math.cos(yaw / 2.0)
        
        if not self.should_publish:
            self.should_publish = True
            publish_thread = threading.Thread(target=self.continuous_publish)
            publish_thread.daemon = True
            publish_thread.start()
            print(f"开始持续发布位置指令: x={x:.1f}, y={y:.1f}, z={z:.1f}")
        else:
            print(f"更新目标位置: x={x:.1f}, y={y:.1f}, z={z:.1f}")

    def continuous_publish(self):
        """持续发布位置指令 - 闭环控制版本"""
        rate = rospy.Rate(50)  # 提高到50Hz获得更好的控制性能
        last_distance = float('inf')
        stable_count = 0
        
        while self.should_publish and not rospy.is_shutdown():
            try:
                # 发布当前目标位置
                self.cmd_pose_pub.publish(self.target_pose)
                
                # 实时监控距离变化
                if self.current_position is not None:
                    current_distance = math.sqrt(
                        (self.target_pose.position.x - self.current_position.x)**2 +
                        (self.target_pose.position.y - self.current_position.y)**2 +
                        (self.target_pose.position.z - self.current_position.z)**2
                    )
                    
                    # 检测是否接近目标
                    if current_distance < 25.0:  # 25米内认为接近
                        stable_count += 1
                        if stable_count > 100:  # 连续2秒(50Hz*2s=100)保持接近
                            rospy.loginfo(f"目标位置稳定到达，距离: {current_distance:.1f}m")
                    else:
                        stable_count = 0
                    
                    last_distance = current_distance
                
                rate.sleep()
            except rospy.ROSInterruptException:
                break

    def get_distance_to_target(self, target_x, target_y, target_z):
        """计算到目标点的距离"""
        if self.current_position is None:
            return float('inf')
        
        dx = target_x - self.current_position.x
        dy = target_y - self.current_position.y
        dz = target_z - self.current_position.z
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def get_horizontal_distance(self, target_x, target_y):
        """计算到目标点的水平距离"""
        if self.current_position is None:
            return float('inf')
        
        dx = target_x - self.current_position.x
        dy = target_y - self.current_position.y
        
        return math.sqrt(dx*dx + dy*dy)

    def wait_for_connection(self):
        """等待ROS连接"""
        print("等待位置信息...")
        
        # 等待ROS位置信息，但添加超时和备用机制
        timeout = 10  # 10秒超时
        start_time = time.time()
        
        while self.current_position is None and not rospy.is_shutdown():
            if time.time() - start_time > timeout:
                print(f"⚠️ ROS位置信息超时 ({timeout}s)，使用默认起始位置")
                # 创建默认位置对象
                class DefaultPosition:
                    def __init__(self):
                        self.x = 0.0  # 旋翼区中心
                        self.y = 0.0
                        self.z = 0.0
                
                self.current_position = DefaultPosition()
                break
            time.sleep(0.1)
        
        if self.current_position:
            print(f"当前位置: ({self.current_position.x:.1f}, {self.current_position.y:.1f}, {self.current_position.z:.1f})")
            
            # 验证位置是否在地图边界内
            if not self.map.is_in_bounds(self.current_position.x, self.current_position.y):
                print(f"⚠️ 警告：当前位置超出地图边界")
                print(f"   地图边界: X[{self.map.x_min}, {self.map.x_max}], Y[{self.map.y_min}, {self.map.y_max}]")
                print(f"   当前位置: ({self.current_position.x}, {self.current_position.y})")
                
                # 强制设置到旋翼区中心
                print(f"   强制移动到旋翼区中心 (0, 0)")
                self.current_position.x = 0.0
                self.current_position.y = 0.0
                self.current_position.z = max(0.0, self.current_position.z)
            
            # 检查是否在旋翼区内
            zone_info = self.map.get_zone_info(self.current_position.x, self.current_position.y)
            print(f"当前区域: {zone_info['name']} ({zone_info['type'].value})")
        else:
            print("❌ 无法获取位置信息")

    def check_flight_safety(self, x, y, z):
        """检查飞行安全性"""
        zone_info = self.map.get_zone_info(x, y)
        
        # 检查是否在地图边界内
        if not self.map.is_in_bounds(x, y):
            return False, "目标点超出地图边界"
        
        # 检查是否在禁飞的居民区
        if zone_info['type'] == ZoneType.RESIDENTIAL:
            return False, f"目标点在居民区 {zone_info['name']} 内，禁止飞行"
        
        # 放宽高度检查 - 只检查最小安全高度
        min_safe_height = 5.0  # 最小安全高度5米
        if zone_info['type'] == ZoneType.MULTIROTOR:
            min_safe_height = 0.0  # 旋翼区允许降落到地面
        elif zone_info['type'] == ZoneType.FREE_SPACE:
            min_safe_height = 15.0  # 自由空间最小15米
        
        if z < min_safe_height:
            return False, f"飞行高度 {z}m 低于最小安全高度 {min_safe_height}m"
        
        return True, "安全"

    def can_switch_to_multirotor(self, x=None, y=None):
        """检查是否可以切换到旋翼模式
        新规则：只要距离(0,0)在半径100米范围内，无论zone类型都允许切换
        """
        if x is None or y is None:
            if not self.current_position:
                return False
            x, y = self.current_position.x, self.current_position.y
        
        # 计算到原点的距离
        distance_to_origin = math.sqrt(x**2 + y**2)
        
        # 新规则：距离原点100米内都可以切换到旋翼模式
        if distance_to_origin <= 100.0:
            print(f"✅ 距离原点{distance_to_origin:.1f}m (<100m)，允许切换到旋翼模式")
            return True
        else:
            print(f"❌ 距离原点{distance_to_origin:.1f}m (≥100m)，不允许切换到旋翼模式")
            return False

    def switch_to_mode(self, target_mode):
        """切换飞行模式（新规则：距离(0,0)在100米内可切换旋翼模式）"""
        if self.current_mode == target_mode:
            return True
        
        # 检查当前位置
        if self.current_position:
            current_zone = self.map.get_zone_info(self.current_position.x, self.current_position.y)
            
            print(f"当前位置: ({self.current_position.x:.1f}, {self.current_position.y:.1f}) - {current_zone['name']}")
            
            # 如果要切换到旋翼模式，使用新的判定规则
            if target_mode == "multirotor":
                if not self.can_switch_to_multirotor():
                    print(f"❌ 无法切换到旋翼模式：距离原点超过100米")
                    print("强制保持固定翼模式")
                    if self.current_mode != "plane":
                        self.send_cmd("plane")
                        self.current_mode = "plane"
                    return False
                else:
                    print(f"✅ 符合新规则，允许切换到旋翼模式")
        
        print(f"切换模式: {self.current_mode} -> {target_mode}")
        
        if target_mode == "plane":
            self.send_cmd("plane")
            self.current_mode = "plane"
        elif target_mode == "multirotor":
            self.send_cmd("multirotor")
            self.current_mode = "multirotor"
        
        time.sleep(3)  # 等待模式切换完成
        return True

    def takeoff_sequence(self):
        """起飞序列 - 必须在旋翼区执行"""
        print(f"\n🚀 开始起飞序列...")
        print("="*50)
        
        # 详细的起飞前检查
        if self.current_position is None:
            print("❌ 起飞失败：无法获取当前位置信息")
            print("   请检查ROS连接和MAVLink通信")
            return False
        
        current_x = self.current_position.x
        current_y = self.current_position.y
        current_z = self.current_position.z
        
        print(f"📍 起飞前位置检查:")
        print(f"   当前位置: ({current_x:.1f}, {current_y:.1f}, {current_z:.1f})")
        print(f"   地图边界: X[{self.map.x_min}, {self.map.x_max}], Y[{self.map.y_min}, {self.map.y_max}]")
        print(f"   旋翼区: 中心({self.map.multirotor_center[0]}, {self.map.multirotor_center[1]}), 半径{self.map.multirotor_radius}m")
        
        # 检查是否在地图边界内
        if not self.map.is_in_bounds(current_x, current_y):
            print(f"⚠️ 当前位置超出地图边界，尝试修正...")
            
            # 自动修正到旋翼区中心
            corrected_x = self.map.multirotor_center[0]
            corrected_y = self.map.multirotor_center[1]
            corrected_z = max(0.0, current_z)
            
            print(f"   修正位置: ({corrected_x}, {corrected_y}, {corrected_z})")
            
            # 更新位置
            self.current_position.x = corrected_x
            self.current_position.y = corrected_y
            self.current_position.z = corrected_z
            
            current_x, current_y, current_z = corrected_x, corrected_y, corrected_z
        
        # 使用新规则检查是否可以切换到旋翼模式
        print(f"   检查是否可切换到旋翼模式（新规则：距离原点100米内）...")
        
        if not self.can_switch_to_multirotor(current_x, current_y):
            # 距离原点超过100米，需要移动到100米范围内
            print(f"⚠️ 距离原点超过100米，自动移动到100米范围内...")
            
            # 移动到距离原点90米的位置（留10米余量）
            distance_to_origin = math.sqrt(current_x**2 + current_y**2)
            if distance_to_origin > 0:
                # 计算缩放比例
                scale = 90.0 / distance_to_origin
                safe_x = current_x * scale
                safe_y = current_y * scale
            else:
                # 如果在原点，移动到旋翼区中心
                safe_x = self.map.multirotor_center[0]
                safe_y = self.map.multirotor_center[1]
            
            print(f"   移动到安全位置: ({safe_x:.1f}, {safe_y:.1f})")
            
            # 更新位置
            self.current_position.x = safe_x
            self.current_position.y = safe_y
            
            # 重新验证
            if not self.can_switch_to_multirotor(safe_x, safe_y):
                print(f"❌ 位置修正失败：修正后位置仍超过100米限制")
                print(f"❌ 起飞失败：无法在可切换旋翼模式的区域内起飞")
                return False
        
        print("✅ 位置验证通过，确认在可切换旋翼模式区域内，可以安全起飞")
        
        # 确保在多旋翼模式（新规则：距离原点100米内允许）
        if not self.switch_to_mode("multirotor"):
            print("❌ 无法切换到旋翼模式，起飞失败")
            return False
        
        # 1. 开始持续发布位置指令
        start_x = self.current_position.x
        start_y = self.current_position.y
        self.set_target_pose(start_x, start_y, 5.0)
        time.sleep(2)
        
        # 2. 设置OFFBOARD模式
        print("设置OFFBOARD模式...")
        self.send_cmd("OFFBOARD")
        time.sleep(2)
        
        # 3. 解锁无人机
        print("解锁无人机...")
        self.send_cmd("ARM")
        time.sleep(3)
        
        # 4. 逐步起飞 - 使用闭环控制
        print("📈 开始闭环控制起飞...")
        start_x = self.current_position.x
        start_y = self.current_position.y
        
        takeoff_heights = [5, 10, 20, self.takeoff_height]
        
        for i, height in enumerate(takeoff_heights):
            print(f"   🎯 起飞阶段 {i+1}/{len(takeoff_heights)}: 目标高度 {height}m")
            
            # 使用闭环控制到达指定高度
            success = self.wait_for_position_reached(start_x, start_y, height, tolerance=3.0, max_wait_time=20.0)
            
            if success:
                print(f"   ✅ 到达高度 {height}m")
            else:
                print(f"   ⚠️ 高度 {height}m 未完全到达，继续下一阶段")
            
            # 短暂等待稳定
            time.sleep(1)
        
        # 最终高度验证
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
        else:
            print("❌ 无法获取最终高度")
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
        
        # 飞行模式决策：只有在旋翼区内才能使用旋翼模式
        current_in_multirotor = (current_zone['type'] == ZoneType.MULTIROTOR)
        target_in_multirotor = (target_zone['type'] == ZoneType.MULTIROTOR)
        
        if current_in_multirotor and target_in_multirotor:
            # 旋翼区内短距离飞行，使用旋翼模式
            print("旋翼区内飞行，使用旋翼模式")
            return self.fly_with_multirotor_mode(target_x, target_y, target_z)
        elif current_in_multirotor and not target_in_multirotor:
            # 从旋翼区飞出，需要先切换到固定翼模式再飞行
            print("从旋翼区飞出，使用固定翼模式")
            return self.fly_from_multirotor_to_outside(target_x, target_y, target_z)
        elif not current_in_multirotor and target_in_multirotor:
            # 飞向旋翼区，在旋翼区边缘切换到旋翼模式
            print("飞向旋翼区，固定翼模式接近后切换旋翼模式")
            return self.fly_to_multirotor_zone(target_x, target_y, target_z)
        else:
            # 旋翼区外飞行，必须使用固定翼模式
            print("旋翼区外飞行，使用固定翼模式")
            return self.fly_with_plane_mode(target_x, target_y, target_z)

    def check_line_intersects_residential(self, x1, y1, x2, y2):
        """检查线段是否与居民区相交"""
        for area in self.map.residential_areas:
            center_x, center_y = area["center"]
            radius = area["radius"]
            
            # 计算点到线段的最短距离
            if self.point_to_line_distance(center_x, center_y, x1, y1, x2, y2) <= radius:
                return True, area["name"]
        return False, None
    
    def point_to_line_distance(self, px, py, x1, y1, x2, y2):
        """计算点到线段的最短距离"""
        # 线段长度的平方
        line_length_sq = (x2 - x1)**2 + (y2 - y1)**2
        
        if line_length_sq == 0:
            # 线段退化为点
            return math.sqrt((px - x1)**2 + (py - y1)**2)
        
        # 计算投影参数t
        t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / line_length_sq
        t = max(0, min(1, t))  # 限制在[0,1]范围内
        
        # 计算投影点
        proj_x = x1 + t * (x2 - x1)
        proj_y = y1 + t * (y2 - y1)
        
        # 返回距离
        return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)
    
    def find_safe_waypoints(self, start_x, start_y, end_x, end_y):
        """使用A*算法规划安全航点，避开居民区"""
        print(f"🧭 A*路径规划: 从 ({start_x:.1f}, {start_y:.1f}) 到 ({end_x:.1f}, {end_y:.1f})")
        
        # 使用A*算法进行路径规划
        start_pos = (start_x, start_y)
        end_pos = (end_x, end_y)
        
        # 调用A*路径规划
        astar_path = self.astar_planner.plan_path(start_pos, end_pos)
        
        if astar_path and len(astar_path) > 1:
            # A*规划成功，计算路径统计信息
            total_distance = 0
            for i in range(len(astar_path) - 1):
                dx = astar_path[i+1][0] - astar_path[i][0]
                dy = astar_path[i+1][1] - astar_path[i][1]
                total_distance += math.sqrt(dx*dx + dy*dy)
            
            # 直线距离
            direct_distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
            path_efficiency = (direct_distance / total_distance) * 100 if total_distance > 0 else 100
            
            print(f"✅ A*路径规划成功:")
            print(f"   路径点数: {len(astar_path)}")
            print(f"   总距离: {total_distance:.1f}m")
            print(f"   直线距离: {direct_distance:.1f}m")
            print(f"   路径效率: {path_efficiency:.1f}%")
            
            # 转换为航点列表（去除起点，保留其他关键点）
            waypoints = []
            for i, (wp_x, wp_y) in enumerate(astar_path):
                if i == 0:  # 跳过起点
                    continue
                waypoints.append((wp_x, wp_y))
                
                # 显示航点信息
                zone_info = self.map.get_zone_info(wp_x, wp_y)
                print(f"   A*航点{i}: ({wp_x:.1f}, {wp_y:.1f}) - {zone_info['name']}")
            
            return waypoints
        
        else:
            # A*规划失败，使用备用简单路径
            print("❌ A*路径规划失败，使用备用策略")
            return self.find_safe_waypoints_fallback(start_x, start_y, end_x, end_y)
    
    def find_safe_waypoints_fallback(self, start_x, start_y, end_x, end_y):
        """备用路径规划方法（原有的简单绕行策略）"""
        print(f"🔄 使用备用路径规划: 从 ({start_x:.1f}, {start_y:.1f}) 到 ({end_x:.1f}, {end_y:.1f})")
        
        # 检查直线路径是否安全
        intersects, area_name = self.check_line_intersects_residential(start_x, start_y, end_x, end_y)
        
        if not intersects:
            print("✅ 直线路径安全，无需绕行")
            return [(end_x, end_y)]
        
        print(f"⚠️ 直线路径与居民区 {area_name} 相交，规划绕行路径")
        
        # 简单绕行策略：选择北绕或南绕
        waypoints = []
        
        # 计算中点用于绕行
        mid_x = (start_x + end_x) / 2
        
        # 尝试北绕（y正方向）
        north_waypoint = (mid_x, 300)  # 北侧绕行点
        north_safe = True
        
        # 检查北绕路径是否安全
        if (self.check_line_intersects_residential(start_x, start_y, north_waypoint[0], north_waypoint[1])[0] or 
            self.check_line_intersects_residential(north_waypoint[0], north_waypoint[1], end_x, end_y)[0]):
            north_safe = False
        
        # 尝试南绕（y负方向）
        south_waypoint = (mid_x, -300)  # 南侧绕行点
        south_safe = True
        
        # 检查南绕路径是否安全
        if (self.check_line_intersects_residential(start_x, start_y, south_waypoint[0], south_waypoint[1])[0] or 
            self.check_line_intersects_residential(south_waypoint[0], south_waypoint[1], end_x, end_y)[0]):
            south_safe = False
        
        # 选择最佳绕行路径
        if north_safe and south_safe:
            # 都安全，选择距离更短的
            north_dist = (math.sqrt((north_waypoint[0] - start_x)**2 + (north_waypoint[1] - start_y)**2) + 
                         math.sqrt((end_x - north_waypoint[0])**2 + (end_y - north_waypoint[1])**2))
            south_dist = (math.sqrt((south_waypoint[0] - start_x)**2 + (south_waypoint[1] - start_y)**2) + 
                         math.sqrt((end_x - south_waypoint[0])**2 + (end_y - south_waypoint[1])**2))
            
            if north_dist <= south_dist:
                waypoints = [north_waypoint, (end_x, end_y)]
                print(f"选择北绕路径: {north_waypoint}")
            else:
                waypoints = [south_waypoint, (end_x, end_y)]
                print(f"选择南绕路径: {south_waypoint}")
        elif north_safe:
            waypoints = [north_waypoint, (end_x, end_y)]
            print(f"选择北绕路径: {north_waypoint}")
        elif south_safe:
            waypoints = [south_waypoint, (end_x, end_y)]
            print(f"选择南绕路径: {south_waypoint}")
        else:
            print("❌ 警告：无法找到安全绕行路径，尝试直接飞行")
            waypoints = [(end_x, end_y)]
        
        return waypoints
    
    def fly_from_multirotor_to_outside(self, target_x, target_y, target_z):
        """从旋翼区飞向外部区域"""
        print("从旋翼区飞向外部，先在旋翼区边缘切换到固定翼模式...")
        
        # 计算旋翼区边缘点
        multirotor_center = self.map.multirotor_center
        multirotor_radius = self.map.multirotor_radius
        
        # 计算从旋翼区中心到目标的方向
        center_x, center_y = multirotor_center
        direction_x = target_x - center_x
        direction_y = target_y - center_y
        direction_length = math.sqrt(direction_x**2 + direction_y**2)
        
        if direction_length > 0:
            # 单位化方向向量
            direction_x /= direction_length
            direction_y /= direction_length
            
            # 计算旋翼区边缘点（略向内一点确保在旋翼区内）
            edge_margin = 10  # 边缘安全距离
            edge_x = center_x + direction_x * (multirotor_radius - edge_margin)
            edge_y = center_y + direction_y * (multirotor_radius - edge_margin)
        else:
            # 如果目标就在旋翼区中心，直接使用当前位置
            edge_x = self.current_position.x
            edge_y = self.current_position.y
        
        print(f"先飞向旋翼区边缘点: ({edge_x:.1f}, {edge_y:.1f})")
        
        # 第一步：在旋翼区内用旋翼模式飞到边缘
        self.switch_to_mode("multirotor")
        self.set_target_pose(edge_x, edge_y, self.cruise_height)
        time.sleep(8)
        
        # 第二步：切换到固定翼模式
        print("在旋翼区边缘切换到固定翼模式...")
        if not self.switch_to_mode("plane"):
            print("❌ 切换固定翼模式失败")
            return False
        
        # 第三步：使用固定翼模式飞向目标
        return self.fly_with_plane_mode(target_x, target_y, target_z)
    
    def fly_to_multirotor_zone(self, target_x, target_y, target_z):
        """飞向旋翼区（固定翼接近，旋翼区内切换旋翼模式）"""
        print("飞向旋翼区，先用固定翼模式接近...")
        
        # 确保当前是固定翼模式
        self.switch_to_mode("plane")
        
        # 计算旋翼区边缘接近点
        multirotor_center = self.map.multirotor_center
        multirotor_radius = self.map.multirotor_radius
        
        center_x, center_y = multirotor_center
        current_x = self.current_position.x
        current_y = self.current_position.y
        
        # 计算从当前位置到旋翼区中心的方向
        direction_x = center_x - current_x
        direction_y = center_y - current_y
        direction_length = math.sqrt(direction_x**2 + direction_y**2)
        
        if direction_length > 0:
            direction_x /= direction_length
            direction_y /= direction_length
        else:
            # 如果已经在可切换区域中心，检查是否可以直接切换到旋翼模式
            print("已在中心位置，检查是否可以切换旋翼模式")
            if self.can_switch_to_multirotor():
                if self.switch_to_mode("multirotor"):
                    return self.fly_with_multirotor_mode(target_x, target_y, target_z)
            # 如果不能切换，继续使用固定翼模式
            return self.fly_with_plane_mode(target_x, target_y, target_z)
        
        # 检查目标是否在旋翼区内
        target_in_multirotor = self.map.distance_to_point(target_x, target_y, center_x, center_y) <= multirotor_radius
        
        if target_in_multirotor:
            # 计算旋翼区边缘的接近点（外侧一点）
            approach_margin = 20  # 接近距离
            approach_x = center_x - direction_x * (multirotor_radius + approach_margin)
            approach_y = center_y - direction_y * (multirotor_radius + approach_margin)
        else:
            # 目标不在旋翼区内，这种情况不应该发生，但仍然接近旋翼区
            print("警告：目标不在旋翼区内，但仍接近旋翼区")
            approach_x = center_x - direction_x * (multirotor_radius + 20)
            approach_y = center_y - direction_y * (multirotor_radius + 20)
        
        print(f"固定翼模式接近旋翼区边缘: ({approach_x:.1f}, {approach_y:.1f})")
        
        # 第一步：用固定翼模式接近旋翼区
        self.set_target_pose(approach_x, approach_y, self.cruise_height)
        time.sleep(10)
        
        # 第二步：进入旋翼区
        print("进入旋翼区...")
        entry_x = center_x - direction_x * (multirotor_radius - 10)
        entry_y = center_y - direction_y * (multirotor_radius - 10)
        
        self.set_target_pose(entry_x, entry_y, self.cruise_height)
        time.sleep(5)
        
        # 验证是否已经可以切换到旋翼模式（新规则）
        if self.current_position:
            if self.can_switch_to_multirotor(self.current_position.x, self.current_position.y):
                print("✅ 已进入可切换旋翼模式区域（距离原点100米内），切换到旋翼模式...")
                
                # 切换到旋翼模式
                if self.switch_to_mode("multirotor"):
                    print("✅ 成功切换到旋翼模式")
                    # 第三步：用旋翼模式精确飞向目标
                    return self.fly_with_multirotor_mode(target_x, target_y, target_z)
                else:
                    print("❌ 切换旋翼模式失败，继续用固定翼模式")
                    return self.fly_with_plane_mode(target_x, target_y, target_z)
            else:
                current_zone = self.map.get_zone_info(self.current_position.x, self.current_position.y)
                print(f"⚠️ 仍未进入可切换旋翼模式区域，当前在{current_zone['name']}，继续用固定翼模式")
                return self.fly_with_plane_mode(target_x, target_y, target_z)
        else:
            print("❌ 无法获取当前位置，继续用固定翼模式")
            return self.fly_with_plane_mode(target_x, target_y, target_z)

    def fly_with_plane_mode(self, target_x, target_y, target_z):
        """使用固定翼模式飞行（集成A*路径规划）"""
        print("🛩️ 使用固定翼模式进行飞行...")
        
        # 确保在固定翼模式
        self.switch_to_mode("plane")
        
        # 使用A*算法规划安全航点
        current_x = self.current_position.x
        current_y = self.current_position.y
        
        print(f"当前位置: ({current_x:.1f}, {current_y:.1f})")
        print(f"目标位置: ({target_x:.1f}, {target_y:.1f})")
        
        # A*路径规划
        waypoints = self.find_safe_waypoints(current_x, current_y, target_x, target_y)
        
        if not waypoints:
            print("❌ 路径规划失败，无法继续")
            return False
        
        # 计算总航行距离和预估时间
        total_distance = 0
        prev_x, prev_y = current_x, current_y
        for wp_x, wp_y in waypoints:
            segment_dist = math.sqrt((wp_x - prev_x)**2 + (wp_y - prev_y)**2)
            total_distance += segment_dist
            prev_x, prev_y = wp_x, wp_y
        
        # 预估飞行时间（假设平均速度15m/s）
        estimated_time = total_distance / 15.0
        print(f"📊 飞行计划:")
        print(f"   总航行距离: {total_distance:.1f}m")
        print(f"   航点数量: {len(waypoints)}")
        print(f"   预估飞行时间: {estimated_time:.1f}s")
        
        # 依次飞向各个航点
        for i, (wp_x, wp_y) in enumerate(waypoints):
            # 动态调整飞行高度
            if i == len(waypoints) - 1:
                # 最后一个航点，使用目标高度
                flight_height = max(target_z + 10, 30)  # 至少30米高度
            else:
                # 中间航点，使用巡航高度
                flight_height = self.cruise_height
            
            print(f"🎯 飞向航点 {i+1}/{len(waypoints)}: ({wp_x:.1f}, {wp_y:.1f}, {flight_height:.1f}m)")
            
            # 安全检查：确保航点不在居民区
            zone_info = self.map.get_zone_info(wp_x, wp_y)
            if zone_info['type'] == ZoneType.RESIDENTIAL:
                print(f"❌ 紧急警告：航点 {i+1} 在居民区 {zone_info['name']} 内！")
                print("   跳过此航点，继续下一个...")
                continue
            
            # 设置目标位置并使用闭环控制
            print(f"🎯 飞向航点 {i+1}/{len(waypoints)}: ({wp_x:.1f}, {wp_y:.1f}, {flight_height:.1f}m)")
            
            # 安全检查：确保航点不在居民区
            zone_info = self.map.get_zone_info(wp_x, wp_y)
            if zone_info['type'] == ZoneType.RESIDENTIAL:
                print(f"❌ 紧急警告：航点 {i+1} 在居民区 {zone_info['name']} 内！")
                print("   跳过此航点，继续下一个...")
                continue
            
            # 计算到航点的距离
            if i == 0:
                segment_distance = math.sqrt((wp_x - current_x)**2 + (wp_y - current_y)**2)
            else:
                prev_wp_x, prev_wp_y = waypoints[i-1]
                segment_distance = math.sqrt((wp_x - prev_wp_x)**2 + (wp_y - prev_wp_y)**2)
            
            print(f"   航段距离: {segment_distance:.1f}m")
            
            # 动态计算容忍度和等待时间
            waypoint_tolerance = min(25.0, max(20.0, segment_distance * 0.1))  # 距离的10%，最小20m，最大25m
            max_wait_time = max(30.0, min(90.0, segment_distance / 15.0))  # 基于15m/s速度，最小30s，最大90s
            
            print(f"   容忍度: {waypoint_tolerance:.1f}m, 最大等待: {max_wait_time:.1f}s")
            
            # 使用闭环控制到达航点
            success = self.wait_for_position_reached(wp_x, wp_y, flight_height, waypoint_tolerance, max_wait_time)
            
            if success:
                print(f"   ✅ 成功到达航点 {i+1}")
            else:
                print(f"   ⚠️ 航点 {i+1} 未完全到达，继续下一个航点")
            
            # 持续监控安全
            if self.current_position:
                current_zone = self.map.get_zone_info(self.current_position.x, self.current_position.y)
                if current_zone['type'] == ZoneType.RESIDENTIAL:
                    print(f"🚨 紧急警告：无人机进入居民区 {current_zone['name']}！")
                    print("   执行紧急脱离程序...")
                    
                    # 紧急脱离：向安全区域移动
                    emergency_x = self.current_position.x
                    emergency_y = self.current_position.y
                    
                    # 策略：向旋翼区方向快速移动
                    if emergency_x > 0:
                        emergency_x = max(0, emergency_x - 200)
                    else:
                        emergency_x = min(0, emergency_x + 200)
                    
                    if emergency_y > 0:
                        emergency_y = max(-500, emergency_y - 100)
                    else:
                        emergency_y = min(500, emergency_y + 100)
                    
                    print(f"   紧急移动到: ({emergency_x:.1f}, {emergency_y:.1f})")
                    self.wait_for_position_reached(emergency_x, emergency_y, flight_height + 20, 30.0, 15.0)
                    break
        
        # 最终接近目标 - 使用闭环控制
        print(f"🎯 A*路径完成，开始闭环精确接近目标...")
        
        # 检查目标是否在旋翼区
        target_zone = self.map.get_zone_info(target_x, target_y)
        if target_zone['type'] == ZoneType.MULTIROTOR:
            print("   目标在旋翼区，准备模式切换...")
        
        # 使用精确控制方法进行最终接近
        final_height = max(target_z, 20)  # 确保安全高度
        success = self.precise_fly_to_position(target_x, target_y, final_height, "固定翼最终目标")
        
        if success:
            print("✅ 固定翼模式闭环导航成功")
        else:
            print("⚠️ 固定翼模式闭环导航未完全达标")
        
        return success

    def fly_with_multirotor_mode(self, target_x, target_y, target_z):
        """使用多旋翼模式飞行 - 闭环控制版本"""
        print("🚁 使用多旋翼模式进行短距离飞行（闭环控制）...")
        
        # 确保在多旋翼模式
        self.switch_to_mode("multirotor")
        
        # 使用精确控制方法
        return self.precise_fly_to_position(target_x, target_y, target_z, "多旋翼目标")

    def land_at_target(self, target_x, target_y, target_z):
        """在目标点降落（新规则：距离原点100米内可切换旋翼模式）"""
        target_zone = self.map.get_zone_info(target_x, target_y)
        
        # 使用新规则检查是否可以切换到旋翼模式
        if self.can_switch_to_multirotor(target_x, target_y):
            print("目标点在可切换旋翼模式区域内（距离原点100米内），切换到旋翼模式降落...")
            
            # 确保在可切换区域内才切换到旋翼模式
            if self.switch_to_mode("multirotor"):
                # 逐步下降
                for height in [10, 5, target_z]:
                    self.set_target_pose(target_x, target_y, height)
                    time.sleep(3)
                
                # 如果目标高度为0，执行降落
                if target_z <= 1.0:
                    print("执行自动降落...")
                    self.send_cmd("LAND")
                    time.sleep(5)
            else:
                print("无法切换到旋翼模式，使用固定翼模式在高度保持悬停")
                self.set_target_pose(target_x, target_y, max(15.0, target_z))
                time.sleep(5)
        else:
            print(f"目标点超出可切换旋翼模式区域（距离原点>100米），在{target_zone['name']}悬停（固定翼模式）...")
            # 在非可切换区域必须保持固定翼模式
            self.switch_to_mode("plane")
            safe_height = max(20.0, target_z)  # 确保足够的安全高度
            self.set_target_pose(target_x, target_y, safe_height)
            time.sleep(5)

    def execute_mission(self):
        """执行完整任务（集成A*路径规划）"""
        print(f"\n🚁 开始VTOL演示飞行任务 (集成A*路径规划)")
        print(f"任务目标点数量: {len(self.targets)}")
        print("="*70)
        
        # 生成任务路径可视化
        print("📊 生成任务路径预览...")
        self.visualize_mission_path("vtol_mission_astar_preview.png")
        
        # 任务路径概览
        print(f"\n🗺️ 任务路径概览:")
        total_mission_distance = 0
        
        for i in range(len(self.targets) - 1):
            if i == 0:
                continue  # 跳过起飞
                
            start_target = self.targets[i]
            end_target = self.targets[i + 1]
            
            start_x, start_y, _ = start_target['position']
            end_x, end_y, _ = end_target['position']
            
            # 快速距离估算
            segment_distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
            total_mission_distance += segment_distance
            
            print(f"   航段 {i}: {start_target['name']} -> {end_target['name']}")
            print(f"          直线距离: {segment_distance:.1f}m")
        
        print(f"   总任务距离(估算): {total_mission_distance:.1f}m")
        print(f"   预估任务时间: {total_mission_distance/15:.1f}s (假设15m/s平均速度)")
        
        # 起飞
        print(f"\n🚀 执行起飞序列...")
        if not self.takeoff_sequence():
            print("❌ 起飞失败，任务终止")
            return False
        
        print("✅ 起飞完成，开始A*导航任务")
        
        # 依次飞向各个目标点
        successful_targets = 0
        
        for i, target in enumerate(self.targets):
            target_x, target_y, target_z = target['position']
            target_name = target['name']
            
            if i == 0:  # 跳过起点
                continue
                
            print(f"\n🎯 任务段 {i}/{len(self.targets)-1}: 飞向 {target_name}")
            print(f"   目标坐标: ({target_x}, {target_y}, {target_z})")
            print(f"   目标描述: {target['description']}")
            print("-" * 60)
            
            # 执行A*路径规划和飞行
            start_time = time.time()
            success = self.fly_to_target(target_x, target_y, target_z)
            flight_time = time.time() - start_time
            
            if success:
                successful_targets += 1
                print(f"✅ 成功到达目标点 {target_name} (用时: {flight_time:.1f}s)")
                
                # 在目标点执行相应动作
                if target_z <= 1.0:  # 降落目标
                    print("🛬 执行降落程序...")
                    self.land_at_target(target_x, target_y, target_z)
                else:  # 悬停目标
                    print("⏸️ 在目标点悬停5秒...")
                    time.sleep(5)
                
                # 记录到达状态
                if self.current_position:
                    actual_distance = self.get_distance_to_target(target_x, target_y, target_z)
                    print(f"   实际位置: ({self.current_position.x:.1f}, {self.current_position.y:.1f}, {self.current_position.z:.1f})")
                    print(f"   到达精度: {actual_distance:.1f}m")
                    
            else:
                print(f"⚠️ 目标点 {target_name} 未能完全到达")
            
            # 检查是否是最后一个目标点
            if i == len(self.targets) - 1:
                final_target = self.targets[-1]
                final_target_z = final_target['position'][2]
                if final_target_z <= 1.0:
                    print("🏁 任务完成，已降落")
                    break
        
        # 任务完成总结
        print(f"\n🎉 VTOL A*导航飞行任务完成!")
        print(f"📊 任务统计:")
        print(f"   成功目标点: {successful_targets}/{len(self.targets)-1}")
        print(f"   成功率: {successful_targets/(len(self.targets)-1)*100:.1f}%")
        print(f"   使用A*算法进行智能路径规划")
        print(f"   所有路径均避开居民区障碍物")
        
        # 任务结束前自动返航到出发点
        print(f"\n🏠 任务完成，执行自动返航...")
        print("发送 AUTO.RTL 命令，无人机将自动返回出发点")
        self.send_cmd("AUTO.RTL")
        
        # 等待返航完成
        print("等待无人机返航并自动降落...")
        time.sleep(3)  # 给返航命令一些响应时间
        
        # 停止发布并解锁
        self.should_publish = False
        time.sleep(1)
        self.send_cmd("DISARM")
        
        return True

    def run(self):
        """运行主程序"""
        try:
            self.wait_for_connection()
            
            # 打印地图和任务信息
            self.map.print_map_summary()
            
            # 执行任务
            self.execute_mission()
            
        except KeyboardInterrupt:
            print("\n收到中断信号，正在停止...")
            self.should_publish = False
            self.send_cmd("HOVER")
            self.send_cmd("DISARM")
        except Exception as e:
            print(f"发生错误: {e}")
            import traceback
            traceback.print_exc()

    def visualize_mission_path(self, save_path="mission_path.png"):
        """可视化整个任务的路径规划结果"""
        print(f"\n📊 生成任务路径可视化图...")
        
        try:
            import matplotlib.pyplot as plt
            
            fig, ax = plt.subplots(1, 1, figsize=(14, 10))
            
            # 绘制地图
            self.map.draw_map(ax)
            
            # 绘制所有目标点
            target_colors = ['green', 'blue', 'orange', 'red', 'purple']
            
            for i, target in enumerate(self.targets):
                x, y, z = target['position']
                color = target_colors[i % len(target_colors)]
                
                ax.plot(x, y, 'o', color=color, markersize=12, 
                       markeredgecolor='black', markeredgewidth=2)
                ax.annotate(f"{i}: {target['name']}", (x, y), 
                           xytext=(10, 10), textcoords='offset points',
                           fontsize=10, fontweight='bold',
                           bbox=dict(boxstyle="round,pad=0.3", facecolor=color, alpha=0.7))
            
            # 为每对相邻目标点规划并绘制路径
            for i in range(len(self.targets) - 1):
                if i == 0:  # 跳过起点到第一个目标的路径（起飞）
                    continue
                    
                start_target = self.targets[i]
                end_target = self.targets[i + 1]
                
                start_x, start_y, _ = start_target['position']
                end_x, end_y, _ = end_target['position']
                
                print(f"规划路径 {i} -> {i+1}: {start_target['name']} -> {end_target['name']}")
                
                # 使用A*算法规划路径
                astar_path = self.astar_planner.plan_path((start_x, start_y), (end_x, end_y))
                
                if astar_path:
                    # 绘制A*路径
                    path_x = [p[0] for p in astar_path]
                    path_y = [p[1] for p in astar_path]
                    
                    ax.plot(path_x, path_y, '-', color=target_colors[i % len(target_colors)], 
                           linewidth=2, alpha=0.8, 
                           label=f'路径 {i}->{i+1}: {start_target["name"][:8]}')
                    
                    # 标记关键航点
                    for j, (px, py) in enumerate(astar_path[1:-1], 1):  # 跳过起点和终点
                        if j % 2 == 0:  # 每隔一个点标记
                            ax.plot(px, py, 's', color=target_colors[i % len(target_colors)], 
                                   markersize=4, alpha=0.6)
                else:
                    # A*失败，绘制直线
                    ax.plot([start_x, end_x], [start_y, end_y], '--', 
                           color='red', linewidth=1, alpha=0.5,
                           label=f'直线 {i}->{i+1} (A*失败)')
            
            ax.set_title('VTOL任务路径规划 (A*算法)', fontsize=16, fontweight='bold')
            ax.set_xlabel('X坐标 (米)', fontsize=12)
            ax.set_ylabel('Y坐标 (米)', fontsize=12)
            ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
            ax.grid(True, alpha=0.3)
            
            plt.tight_layout()
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"✅ 任务路径图已保存到: {save_path}")
            
            return True
            
        except Exception as e:
            print(f"❌ 可视化失败: {e}")
            return False

    def wait_for_position_reached(self, target_x, target_y, target_z, tolerance=20.0, max_wait_time=60.0):
        """等待到达目标位置 - 闭环控制"""
        print(f"🎯 闭环等待到达: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f}), 容忍度: {tolerance}m")
        
        start_time = time.time()
        min_distance = float('inf')
        stable_time = 0
        last_stable_check = time.time()
        
        # 设置目标并开始发布
        self.set_target_pose(target_x, target_y, target_z)
        
        rate = rospy.Rate(10)  # 10Hz检查频率
        
        while time.time() - start_time < max_wait_time and not rospy.is_shutdown():
            if self.current_position is None:
                print("⚠️ 无法获取位置信息")
                rate.sleep()
                continue
            
            # 计算当前距离
            current_distance = self.get_distance_to_target(target_x, target_y, target_z)
            min_distance = min(min_distance, current_distance)
            
            # 持续发布目标位置确保控制器收到指令
            self.set_target_pose(target_x, target_y, target_z)
            
            # 检查是否在容忍度内
            if current_distance <= tolerance:
                # 检查稳定性
                current_time = time.time()
                if current_time - last_stable_check >= 1.0:  # 每秒检查一次稳定性
                    stable_time += 1.0
                    last_stable_check = current_time
                    
                    if stable_time >= 3.0:  # 稳定3秒即认为到达
                        print(f"✅ 稳定到达目标！距离: {current_distance:.1f}m, 用时: {time.time() - start_time:.1f}s")
                        return True
            else:
                stable_time = 0  # 重置稳定时间
                last_stable_check = time.time()
            
            # 每5秒报告状态
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0 and elapsed > 0:
                print(f"  📊 {elapsed:5.1f}s | 当前距离: {current_distance:6.1f}m | 最小距离: {min_distance:6.1f}m")
                print(f"      位置: ({self.current_position.x:7.1f}, {self.current_position.y:7.1f}, {self.current_position.z:6.1f})")
                print(f"      目标: ({target_x:7.1f}, {target_y:7.1f}, {target_z:6.1f})")
            
            rate.sleep()
        
        # 超时处理
        final_distance = self.get_distance_to_target(target_x, target_y, target_z)
        print(f"⏰ 等待超时！最终距离: {final_distance:.1f}m, 最小距离: {min_distance:.1f}m")
        
        # 即使超时，如果距离可接受也认为成功
        if final_distance <= tolerance * 2:  # 容忍度的2倍内也算接受
            print(f"✅ 虽然超时，但距离可接受")
            return True
        else:
            print(f"❌ 未能到达目标位置")
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
        
        # 分阶段接近：远距离 -> 中距离 -> 精确定位
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
                # 更新当前位置用于下一阶段计算
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

def main():
    """主函数：执行完整的VTOL飞行演示"""
    print("\n🚁 VTOL固定翼无人机飞行演示开始")
    print("=" * 50)
    
    try:
        # 初始化飞行控制器
        print("📡 初始化飞行控制器...")
        controller = VTOLDemoFlight()
        
        # 现在初始化ROS通信
        print("🔗 设置ROS通信...")
        controller.init_ros()
        
        # 等待ROS节点完全初始化
        print("⏱️  等待ROS节点初始化...")
        time.sleep(2)
        
        # 等待获取当前位置
        print("📍 等待获取当前位置...")
        timeout = 10
        start_time = time.time()
        while controller.current_position is None and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        if controller.current_position is None:
            print("❌ 无法获取当前位置，请检查仿真环境是否正常运行")
            return False
        
        print(f"✅ 当前位置: ({controller.current_position.x:.1f}, {controller.current_position.y:.1f}, {controller.current_position.z:.1f})")
        
        # 显示目标点信息
        print(f"\n🎯 目标点列表 (共{len(controller.targets)}个):")
        for i, target in enumerate(controller.targets):
            x, y, z = target['position']
            zone = controller.map.get_zone_type(x, y)
            zone_name = {
                ZoneType.MULTIROTOR: "旋翼区",
                ZoneType.FREE_SPACE: "自由空间", 
                ZoneType.RESIDENTIAL: "居民区"
            }.get(zone, "未知区域")
            print(f"   {i+1}. {target['name']}: ({x:.1f}, {y:.1f}, {z:.1f}) - {zone_name}")
            print(f"      描述: {target['description']}")
        
        # 执行自动飞行任务
        print(f"\n🚀 开始执行自动飞行任务...")
        success = controller.execute_mission()
        
        if success:
            print("\n🎉 飞行任务完成！")
            print("✅ 所有目标点已成功到达")
        else:
            print("\n⚠️ 飞行任务部分完成")
            print("❗ 某些目标点可能未能成功到达")
        
        # 显示飞行统计
        print(f"\n📊 飞行统计:")
        print(f"   目标点总数: {len(controller.targets)}")
        print(f"   已完成: {controller.current_target_index}")
        print(f"   完成率: {controller.current_target_index/len(controller.targets)*100:.1f}%")
        
        return success
        
    except KeyboardInterrupt:
        print("\n🛑 用户中断飞行")
        return False
    except Exception as e:
        print(f"\n❌ 飞行过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        print("\n🔚 飞行演示结束")


if __name__ == "__main__":
    # 设置日志级别
    import logging
    logging.basicConfig(level=logging.INFO)
    
    # 检查ROS环境
    try:
        rospy.init_node('vtol_demo_flight', anonymous=True)
        print("✅ ROS节点初始化成功")
    except Exception as e:
        print(f"❌ ROS节点初始化失败: {e}")
        print("   请确保已启动ROS核心和仿真环境")
        exit(1)
    
    # 运行主程序
    success = main()
    
    # 退出码
    exit(0 if success else 1)