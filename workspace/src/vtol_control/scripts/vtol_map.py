#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
创建一个地图类
x范围-100：2000，y范围-1000：1000
两个居民区，中心点分别为(1200, 0)和(1800, 0)，半径200
旋翼区中心(0,0),半径100
只包含地图定义、区域判断和可视化功能
'''

import math
import matplotlib.pyplot as plt
import numpy as np
from enum import Enum
from typing import List, Tuple, Optional, Dict, Any


class ZoneType(Enum):
    """区域类型枚举"""
    MULTIROTOR = "multirotor"  # 旋翼区
    RESIDENTIAL = "residential"  # 居民区
    FREE_SPACE = "free_space"  # 自由空间


class VTOLMap:
    """VTOL无人机地图类"""
    
    def __init__(self):
        # 地图边界
        self.x_min = -100
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
        
        # 路径规划参数（保留用于兼容性）
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
    
    # 可视化地图
    print("\n正在生成地图可视化...")
    vtol_map.visualize_map(save_path="/home/yzy/comsen_challengecup/vtol_map.png")


if __name__ == '__main__':
    main()