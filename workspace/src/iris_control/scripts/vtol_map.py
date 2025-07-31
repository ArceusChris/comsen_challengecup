#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
创建一个地图类
x范围0：2000，y范围-1000：1000
两个居民区，中心点分别为(1200, 0)和(1800, 0)，半径200
旋翼区中心(0,0),半径100
'''

import math
import matplotlib.pyplot as plt
import numpy as np
from enum import Enum
import heapq
from typing import List, Tuple, Optional, Dict, Any


class ZoneType(Enum):
    """区域类型枚举"""
    MULTIROTOR = "multirotor"  # 旋翼区
    RESIDENTIAL = "residential"  # 居民区
    FREE_SPACE = "free_space"  # 自由空间


class PathNode:
    """路径规划节点"""
    def __init__(self, x: float, y: float, g_cost: float = 0, h_cost: float = 0, parent=None):
        self.x = x
        self.y = y
        self.g_cost = g_cost  # 从起点到当前节点的实际距离
        self.h_cost = h_cost  # 从当前节点到终点的启发式距离
        self.f_cost = g_cost + h_cost  # 总估计成本
        self.parent = parent
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        return abs(self.x - other.x) < 10 and abs(self.y - other.y) < 10


class VTOLMap:
    """VTOL无人机地图类"""
    
    def __init__(self):
        # 地图边界
        self.x_min = 0
        self.x_max = 2000
        self.y_min = -1000
        self.y_max = 1000
        
        # 旋翼区 (起降区)
        self.multirotor_center = (0, 0)
        self.multirotor_radius = 100
        
        # 居民区
        self.residential_areas = [
            {"center": (1200, 0), "radius": 200, "name": "downtown"},
            {"center": (1800, 0), "radius": 200, "name": "suburb"}
        ]
        
        # 路径规划参数
        self.grid_resolution = 50  # 网格分辨率(米)
        self.safety_margin = 100   # 居民区安全距离(米)
        
        # 重要位置点
        self.landmarks = {
            "origin": (0, 0),
            "downtown_center": (1200, 0),
            "suburb_center": (1800, 0),
            "person_red": (1495.0, -105.0),
            "person_yellow": (1492.0, 82.6),
        }
        
        print("VTOL地图初始化完成")
        print(f"地图范围: x[{self.x_min}, {self.x_max}], y[{self.y_min}, {self.y_max}]")
    
    def is_in_bounds(self, x, y):
        """检查坐标是否在地图边界内"""
        return (self.x_min <= x <= self.x_max and 
                self.y_min <= y <= self.y_max)
    
    def distance_to_point(self, x1, y1, x2, y2):
        """计算两点间距离"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def is_in_multirotor_zone(self, x, y):
        """检查坐标是否在旋翼区内"""
        dist = self.distance_to_point(x, y, 
                                     self.multirotor_center[0], 
                                     self.multirotor_center[1])
        return dist <= self.multirotor_radius
    
    def is_in_residential_area(self, x, y):
        """检查坐标是否在居民区内"""
        for area in self.residential_areas:
            dist = self.distance_to_point(x, y, 
                                         area["center"][0], 
                                         area["center"][1])
            if dist <= area["radius"]:
                return True, area["name"]
        return False, None
    
    def get_zone_type(self, x, y):
        """获取指定坐标的区域类型"""
        if not self.is_in_bounds(x, y):
            return None
        
        if self.is_in_multirotor_zone(x, y):
            return ZoneType.MULTIROTOR
        
        in_residential, area_name = self.is_in_residential_area(x, y)
        if in_residential:
            return ZoneType.RESIDENTIAL
        
        return ZoneType.FREE_SPACE
    
    def get_zone_info(self, x, y):
        """获取指定坐标的详细区域信息"""
        zone_type = self.get_zone_type(x, y)
        
        if zone_type is None:
            return {"type": None, "name": "out_of_bounds"}
        
        if zone_type == ZoneType.MULTIROTOR:
            return {"type": zone_type, "name": "multirotor_zone"}
        
        if zone_type == ZoneType.RESIDENTIAL:
            _, area_name = self.is_in_residential_area(x, y)
            return {"type": zone_type, "name": area_name}
        
        return {"type": zone_type, "name": "free_space"}
    
    def get_safe_flight_height(self, x, y):
        """获取指定位置的安全飞行高度"""
        zone_type = self.get_zone_type(x, y)
        
        if zone_type == ZoneType.MULTIROTOR:
            return 5.0  # 旋翼区低空飞行
        elif zone_type == ZoneType.RESIDENTIAL:
            return 50.0  # 居民区高空飞行
        else:
            return 30.0  # 自由空间中等高度
    
    def plan_waypoints(self, start_x, start_y, end_x, end_y, num_points=5):
        """规划航点路径"""
        waypoints = []
        
        for i in range(num_points + 1):
            progress = i / num_points
            x = start_x + progress * (end_x - start_x)
            y = start_y + progress * (end_y - start_y)
            z = self.get_safe_flight_height(x, y)
            
            zone_info = self.get_zone_info(x, y)
            waypoints.append({
                "position": (x, y, z),
                "zone_type": zone_info["type"],
                "zone_name": zone_info["name"]
            })
        
        return waypoints
    
    def find_nearest_landmark(self, x, y):
        """找到最近的地标点"""
        min_distance = float('inf')
        nearest_landmark = None
        
        for name, (lx, ly) in self.landmarks.items():
            distance = self.distance_to_point(x, y, lx, ly)
            if distance < min_distance:
                min_distance = distance
                nearest_landmark = (name, (lx, ly))
        
        return nearest_landmark, min_distance
    
    def get_landing_zones(self):
        """获取适合降落的区域"""
        landing_zones = []
        
        # 旋翼区适合降落
        landing_zones.append({
            "center": self.multirotor_center,
            "radius": self.multirotor_radius,
            "type": "multirotor_zone",
            "safety_level": "high"
        })
        
        # 居民区中心附近适合降落
        for area in self.residential_areas:
            landing_zones.append({
                "center": area["center"],
                "radius": 50,  # 居民区中心小范围降落
                "type": f"residential_{area['name']}",
                "safety_level": "medium"
            })
        
        return landing_zones
    
    def visualize_map(self, show_landmarks=True, show_zones=True, save_path=None):
        """可视化地图"""
        fig, ax = plt.subplots(1, 1, figsize=(12, 8))
        
        # 设置地图边界
        ax.set_xlim(self.x_min - 100, self.x_max + 100)
        ax.set_ylim(self.y_min - 100, self.y_max + 100)
        
        if show_zones:
            # 绘制旋翼区
            multirotor_circle = plt.Circle(self.multirotor_center, self.multirotor_radius, 
                                         color='blue', alpha=0.3, label='旋翼区')
            ax.add_patch(multirotor_circle)
            
            # 绘制居民区
            for i, area in enumerate(self.residential_areas):
                color = 'red' if i == 0 else 'orange'
                residential_circle = plt.Circle(area["center"], area["radius"], 
                                              color=color, alpha=0.3, 
                                              label=f'居民区-{area["name"]}')
                ax.add_patch(residential_circle)
        
        if show_landmarks:
            # 绘制地标点
            for name, (x, y) in self.landmarks.items():
                ax.plot(x, y, 'ko', markersize=8)
                ax.annotate(name, (x, y), xytext=(5, 5), 
                           textcoords='offset points', fontsize=10)
        
        # 设置图形属性
        ax.set_xlabel('X坐标 (m)')
        ax.set_ylabel('Y坐标 (m)')
        ax.set_title('VTOL无人机地图')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_aspect('equal')
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"地图已保存到: {save_path}")
        
        plt.show()
        return fig, ax
    
    def draw_map(self, ax, show_landmarks=True, show_zones=True):
        """在给定的matplotlib轴上绘制地图（不创建新的图形）"""
        # 设置地图边界
        ax.set_xlim(self.x_min - 100, self.x_max + 100)
        ax.set_ylim(self.y_min - 100, self.y_max + 100)
        
        if show_zones:
            # 绘制旋翼区
            multirotor_circle = plt.Circle(self.multirotor_center, self.multirotor_radius, 
                                         color='blue', alpha=0.3, label='旋翼区')
            ax.add_patch(multirotor_circle)
            
            # 绘制居民区
            for i, area in enumerate(self.residential_areas):
                color = 'red' if i == 0 else 'orange'
                residential_circle = plt.Circle(area["center"], area["radius"], 
                                              color=color, alpha=0.3, 
                                              label=f'居民区-{area["name"]}')
                ax.add_patch(residential_circle)
        
        if show_landmarks:
            # 绘制地标点
            for name, (x, y) in self.landmarks.items():
                ax.plot(x, y, 'ko', markersize=6)
                ax.annotate(name, (x, y), xytext=(5, 5), 
                           textcoords='offset points', fontsize=8, alpha=0.7)
        
        # 设置图形属性
        ax.set_xlabel('X坐标 (m)')
        ax.set_ylabel('Y坐标 (m)')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
    
    def export_landmarks_to_dict(self):
        """导出地标点为字典格式"""
        landmarks_with_zones = {}
        
        for name, (x, y) in self.landmarks.items():
            zone_info = self.get_zone_info(x, y)
            safe_height = self.get_safe_flight_height(x, y)
            
            landmarks_with_zones[name] = {
                "position": (x, y, safe_height),
                "zone_type": zone_info["type"].value if zone_info["type"] else None,
                "zone_name": zone_info["name"]
            }
        
        return landmarks_with_zones
    
    def print_map_summary(self):
        """打印地图摘要信息"""
        print("\n" + "="*50)
        print("VTOL地图摘要")
        print("="*50)
        print(f"地图范围: X[{self.x_min}, {self.x_max}] Y[{self.y_min}, {self.y_max}]")
        print(f"旋翼区: 中心{self.multirotor_center}, 半径{self.multirotor_radius}m")
        
        print("\n居民区:")
        for area in self.residential_areas:
            print(f"  - {area['name']}: 中心{area['center']}, 半径{area['radius']}m")
        
        print("\n地标点:")
        landmarks_dict = self.export_landmarks_to_dict()
        for name, info in landmarks_dict.items():
            pos = info["position"]
            print(f"  - {name}: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}m) [{info['zone_name']}]")
        
        print("\n降落区域:")
        landing_zones = self.get_landing_zones()
        for zone in landing_zones:
            print(f"  - {zone['type']}: 中心{zone['center']}, 半径{zone['radius']}m, 安全级别: {zone['safety_level']}")

    def heuristic_distance(self, x1: float, y1: float, x2: float, y2: float) -> float:
        """计算启发式距离（欧几里德距离）"""
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def is_position_safe(self, x: float, y: float, include_safety_margin: bool = True) -> bool:
        """检查位置是否安全（不在居民区内）"""
        if not self.is_in_bounds(x, y):
            return False
        
        for area in self.residential_areas:
            center_x, center_y = area["center"]
            radius = area["radius"]
            if include_safety_margin:
                radius += self.safety_margin
            
            distance = self.distance_to_point(x, y, center_x, center_y)
            if distance <= radius:
                return False
        
        return True
    
    def get_neighbors(self, node: PathNode) -> List[PathNode]:
        """获取节点的邻居节点（8方向）"""
        neighbors = []
        directions = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1)
        ]
        
        for dx, dy in directions:
            new_x = node.x + dx * self.grid_resolution
            new_y = node.y + dy * self.grid_resolution
            
            # 检查边界和安全性
            if self.is_position_safe(new_x, new_y):
                # 计算移动成本（对角线移动成本更高）
                move_cost = self.grid_resolution * (1.414 if dx != 0 and dy != 0 else 1.0)
                neighbors.append(PathNode(new_x, new_y, node.g_cost + move_cost))
        
        return neighbors
    
    def find_optimal_path(self, start_x: float, start_y: float, end_x: float, end_y: float) -> Optional[List[Tuple[float, float]]]:
        """使用A*算法寻找最优路径"""
        print(f"开始A*路径规划: ({start_x:.1f}, {start_y:.1f}) -> ({end_x:.1f}, {end_y:.1f})")
        
        # 检查起点和终点是否安全
        if not self.is_position_safe(start_x, start_y, include_safety_margin=False):
            print(f"警告: 起点 ({start_x:.1f}, {start_y:.1f}) 不安全")
        
        if not self.is_position_safe(end_x, end_y, include_safety_margin=False):
            print(f"警告: 终点 ({end_x:.1f}, {end_y:.1f}) 不安全")
            return None
        
        # 将坐标对齐到网格
        start_x = round(start_x / self.grid_resolution) * self.grid_resolution
        start_y = round(start_y / self.grid_resolution) * self.grid_resolution
        end_x = round(end_x / self.grid_resolution) * self.grid_resolution
        end_y = round(end_y / self.grid_resolution) * self.grid_resolution
        
        # 初始化节点
        start_node = PathNode(start_x, start_y, 0, self.heuristic_distance(start_x, start_y, end_x, end_y))
        end_node = PathNode(end_x, end_y)
        
        # A*算法
        open_list = [start_node]
        closed_list = set()
        
        max_iterations = 10000  # 防止无限循环
        iteration = 0
        
        while open_list and iteration < max_iterations:
            iteration += 1
            
            # 获取F成本最小的节点
            current_node = heapq.heappop(open_list)
            node_key = (current_node.x, current_node.y)
            
            if node_key in closed_list:
                continue
            
            closed_list.add(node_key)
            
            # 检查是否到达终点
            if current_node == end_node:
                path = []
                while current_node:
                    path.append((current_node.x, current_node.y))
                    current_node = current_node.parent
                path.reverse()
                print(f"A*路径规划成功，共{len(path)}个节点，迭代{iteration}次")
                return self.smooth_path(path)
            
            # 生成邻居节点
            neighbors = self.get_neighbors(current_node)
            for neighbor in neighbors:
                neighbor_key = (neighbor.x, neighbor.y)
                
                if neighbor_key in closed_list:
                    continue
                
                # 计算到邻居的总成本
                neighbor.h_cost = self.heuristic_distance(neighbor.x, neighbor.y, end_x, end_y)
                neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                neighbor.parent = current_node
                
                # 检查是否已在开放列表中
                existing_neighbor = None
                for node in open_list:
                    if node.x == neighbor.x and node.y == neighbor.y:
                        existing_neighbor = node
                        break
                
                if existing_neighbor is None or neighbor.g_cost < existing_neighbor.g_cost:
                    if existing_neighbor:
                        open_list.remove(existing_neighbor)
                    heapq.heappush(open_list, neighbor)
        
        print(f"A*路径规划失败，迭代{iteration}次后未找到路径")
        return None
    
    def smooth_path(self, path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """平滑路径，移除不必要的中间点"""
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        
        i = 0
        while i < len(path) - 1:
            # 尝试从当前点直接连接到更远的点
            j = len(path) - 1
            while j > i + 1:
                if self.is_line_safe(path[i][0], path[i][1], path[j][0], path[j][1]):
                    smoothed.append(path[j])
                    i = j
                    break
                j -= 1
            else:
                # 无法跳过，添加下一个点
                smoothed.append(path[i + 1])
                i += 1
        
        print(f"路径平滑: {len(path)} -> {len(smoothed)} 个节点")
        return smoothed
    
    def is_line_safe(self, x1: float, y1: float, x2: float, y2: float) -> bool:
        """检查线段是否安全（不穿过居民区）"""
        # 采样线段上的点进行安全检查
        distance = self.distance_to_point(x1, y1, x2, y2)
        num_samples = max(5, int(distance / 50))
        
        for i in range(num_samples + 1):
            t = i / num_samples
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            
            if not self.is_position_safe(x, y):
                return False
        
        return True
    
    def plan_safe_path(self, start_x: float, start_y: float, end_x: float, end_y: float) -> List[Tuple[float, float, float]]:
        """规划安全路径（包含高度信息）"""
        print(f"\n开始规划安全路径...")
        
        # 首先检查直线路径是否安全
        if self.is_line_safe(start_x, start_y, end_x, end_y):
            print("直线路径安全，使用直接路径")
            waypoints = []
            distance = self.distance_to_point(start_x, start_y, end_x, end_y)
            num_waypoints = max(2, min(5, int(distance / 300)))
            
            for i in range(1, num_waypoints + 1):
                t = i / num_waypoints
                x = start_x + t * (end_x - start_x)
                y = start_y + t * (end_y - start_y)
                z = self.get_safe_flight_height(x, y)
                waypoints.append((x, y, z))
            
            return waypoints
        
        # 使用A*算法寻找最优路径
        path_2d = self.find_optimal_path(start_x, start_y, end_x, end_y)
        
        if path_2d is None:
            print("未找到安全路径，使用简单绕行策略")
            return self.plan_simple_detour(start_x, start_y, end_x, end_y)
        
        # 将2D路径转换为3D路径
        path_3d = []
        for x, y in path_2d:
            z = self.get_safe_flight_height(x, y)
            path_3d.append((x, y, z))
        
        print(f"A*路径规划完成，共{len(path_3d)}个航点")
        for i, (x, y, z) in enumerate(path_3d):
            zone_info = self.get_zone_info(x, y)
            print(f"  航点{i+1}: ({x:.1f}, {y:.1f}, {z:.1f}) - {zone_info['name']}")
        
        return path_3d
    
    def plan_simple_detour(self, start_x: float, start_y: float, end_x: float, end_y: float) -> List[Tuple[float, float, float]]:
        """简单绕行策略（作为A*的备选方案）"""
        print("使用简单绕行策略...")
        
        # 计算中点
        mid_x = (start_x + end_x) / 2
        mid_y = (start_y + end_y) / 2
        
        # 选择绕行方向（北绕或南绕）
        if mid_y >= 0:
            # 南绕
            detour_y = -500
        else:
            # 北绕
            detour_y = 500
        
        waypoints = []
        
        # 第一段：到绕行点
        detour_x = mid_x
        if self.is_position_safe(detour_x, detour_y):
            z = self.get_safe_flight_height(detour_x, detour_y)
            waypoints.append((detour_x, detour_y, z))
        
        # 第二段：到目标点
        z = self.get_safe_flight_height(end_x, end_y)
        waypoints.append((end_x, end_y, z))
        
        return waypoints
    
    def validate_flight_path(self, waypoints: List[Tuple[float, float, float]]) -> bool:
        """验证飞行路径的安全性"""
        print("\n验证飞行路径安全性...")
        
        for i, (x, y, z) in enumerate(waypoints):
            # 检查边界
            if not self.is_in_bounds(x, y):
                print(f"❌ 航点{i+1} ({x:.1f}, {y:.1f}) 超出地图边界")
                return False
            
            # 检查居民区
            zone_info = self.get_zone_info(x, y)
            if zone_info['type'] == ZoneType.RESIDENTIAL:
                print(f"❌ 航点{i+1} ({x:.1f}, {y:.1f}) 在居民区 {zone_info['name']} 内")
                return False
            
            # 检查高度
            min_height = self.get_safe_flight_height(x, y)
            if z < min_height - 5:  # 5米容差
                print(f"⚠️ 航点{i+1} 高度 {z:.1f}m 低于安全高度 {min_height:.1f}m")
        
        # 检查相邻航点之间的路径
        for i in range(len(waypoints) - 1):
            x1, y1, _ = waypoints[i]
            x2, y2, _ = waypoints[i + 1]
            
            if not self.is_line_safe(x1, y1, x2, y2):
                print(f"❌ 航点{i+1}到航点{i+2}的路径穿过居民区")
                return False
        
        print("✅ 飞行路径验证通过")
        return True
    
    def visualize_path(self, waypoints: List[Tuple[float, float, float]], start_point: Tuple[float, float] = None, 
                      end_point: Tuple[float, float] = None, save_path: str = None):
        """可视化飞行路径"""
        # 先绘制地图基础元素
        fig, ax = self.visualize_map(show_landmarks=True, show_zones=True, save_path=None)
        
        if waypoints:
            # 提取路径坐标
            path_x = [wp[0] for wp in waypoints]
            path_y = [wp[1] for wp in waypoints]
            
            # 绘制路径
            ax.plot(path_x, path_y, 'g-', linewidth=3, alpha=0.7, label='规划路径')
            ax.plot(path_x, path_y, 'go', markersize=8, label='航点')
            
            # 标注航点
            for i, (x, y, z) in enumerate(waypoints):
                ax.annotate(f'WP{i+1}\n({x:.0f},{y:.0f},{z:.0f})', 
                           (x, y), xytext=(10, 10), textcoords='offset points',
                           fontsize=9, bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))
        
        # 绘制起点和终点
        if start_point:
            ax.plot(start_point[0], start_point[1], 'bs', markersize=12, label='起点')
            ax.annotate('起点', start_point, xytext=(10, 10), textcoords='offset points',
                       fontsize=12, fontweight='bold')
        
        if end_point:
            ax.plot(end_point[0], end_point[1], 'rs', markersize=12, label='终点')
            ax.annotate('终点', end_point, xytext=(10, 10), textcoords='offset points',
                       fontsize=12, fontweight='bold')
        
        # 设置图形属性
        ax.set_title('VTOL无人机智能路径规划', fontsize=16, fontweight='bold')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"路径可视化已保存到: {save_path}")
        
        plt.close(fig)  # 避免显示图形
        return fig, ax
    
    def analyze_path_metrics(self, waypoints: List[Tuple[float, float, float]], start_point: Tuple[float, float] = None) -> Dict[str, Any]:
        """分析路径指标"""
        if not waypoints:
            return {}
        
        metrics = {}
        
        # 计算总距离
        total_distance = 0
        all_points = []
        
        if start_point:
            all_points.append((start_point[0], start_point[1], waypoints[0][2] if waypoints else 0))
        
        all_points.extend(waypoints)
        
        for i in range(len(all_points) - 1):
            x1, y1, z1 = all_points[i]
            x2, y2, z2 = all_points[i + 1]
            distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
            total_distance += distance
        
        metrics['total_distance'] = total_distance
        metrics['num_waypoints'] = len(waypoints)
        
        # 计算直线距离（用于计算绕行系数）
        if start_point and waypoints:
            direct_distance = math.sqrt(
                (waypoints[-1][0] - start_point[0])**2 + 
                (waypoints[-1][1] - start_point[1])**2
            )
            metrics['direct_distance'] = direct_distance
            metrics['detour_ratio'] = total_distance / direct_distance if direct_distance > 0 else 1.0
        
        # 分析高度变化
        heights = [wp[2] for wp in waypoints]
        metrics['min_height'] = min(heights) if heights else 0
        metrics['max_height'] = max(heights) if heights else 0
        metrics['avg_height'] = sum(heights) / len(heights) if heights else 0
        
        # 安全性分析
        safe_points = 0
        for x, y, z in waypoints:
            if self.is_position_safe(x, y, include_safety_margin=False):
                safe_points += 1
        
        metrics['safety_ratio'] = safe_points / len(waypoints) if waypoints else 0
        
        return metrics
    
    def print_path_report(self, waypoints: List[Tuple[float, float, float]], start_point: Tuple[float, float] = None):
        """打印路径报告"""
        print("\n" + "="*60)
        print("路径分析报告")
        print("="*60)
        
        if not waypoints:
            print("无有效路径")
            return
        
        metrics = self.analyze_path_metrics(waypoints, start_point)
        
        print(f"航点数量: {metrics.get('num_waypoints', 0)}")
        print(f"总飞行距离: {metrics.get('total_distance', 0):.1f} 米")
        
        if 'direct_distance' in metrics:
            print(f"直线距离: {metrics['direct_distance']:.1f} 米")
            print(f"绕行系数: {metrics['detour_ratio']:.2f}")
        
        print(f"飞行高度: {metrics.get('min_height', 0):.1f} ~ {metrics.get('max_height', 0):.1f} 米 (平均: {metrics.get('avg_height', 0):.1f} 米)")
        print(f"路径安全性: {metrics.get('safety_ratio', 0)*100:.1f}%")
        
        # 详细航点信息
        print("\n航点详情:")
        for i, (x, y, z) in enumerate(waypoints):
            zone_info = self.get_zone_info(x, y)
            safety = "✅" if self.is_position_safe(x, y, include_safety_margin=False) else "❌"
            print(f"  {i+1:2d}. ({x:6.1f}, {y:6.1f}, {z:4.1f}) - {zone_info['name']:15} {safety}")
        
        print("="*60)
        

def main():
    """主函数 - 演示地图功能"""
    # 创建地图实例
    vtol_map = VTOLMap()
    
    # 打印地图摘要
    vtol_map.print_map_summary()
    
    # 测试一些坐标
    test_points = [
        (0, 0),          # 旋翼区中心
        (1200, 0),       # downtown中心
        (1800, 0),       # suburb中心
        (1495, -105),    # person_red
        (500, 500),      # 自由空间
        (2500, 0)        # 超出边界
    ]
    
    print("\n" + "="*50)
    print("坐标测试")
    print("="*50)
    
    for x, y in test_points:
        zone_info = vtol_map.get_zone_info(x, y)
        safe_height = vtol_map.get_safe_flight_height(x, y)
        in_bounds = vtol_map.is_in_bounds(x, y)
        
        print(f"坐标({x}, {y}): 区域={zone_info['name']}, "
              f"安全高度={safe_height}m, 边界内={in_bounds}")
    
    # 规划路径示例
    print("\n" + "="*50)
    print("路径规划示例: 从原点到person_red")
    print("="*50)
    
    waypoints = vtol_map.plan_waypoints(0, 0, 1495, -105, num_points=3)
    for i, wp in enumerate(waypoints):
        pos = wp["position"]
        print(f"航点{i}: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}m) [{wp['zone_name']}]")
    
    # A*路径规划示例
    print("\n" + "="*50)
    print("A*路径规划示例: 从原点到person_red")
    print("="*50)
    
    a_star_path = vtol_map.a_star_pathfinding((0, 0), (1495, -105))
    if a_star_path:
        for i, (x, y) in enumerate(a_star_path):
            z = vtol_map.get_safe_flight_height(x, y)
            zone_info = vtol_map.get_zone_info(x, y)
            print(f"航点{i}: ({x:.1f}, {y:.1f}, {z:.1f}m) [{zone_info['name']}]")
    else:
        print("未找到可行路径")
    
    # 可视化地图
    print("\n正在生成地图可视化...")
    vtol_map.visualize_map(save_path="/home/yzy/comsen_challengecup/vtol_map.png")


if __name__ == '__main__':
    main()