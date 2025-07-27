#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
VTOL无人机飞行模式切换和路径规划验证测试
测试只有在旋翼区内才能切换为旋翼模式的逻辑
"""

import os
import sys

# 添加工作目录到Python路径
workspace_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(workspace_dir)

from workspace.vtol_control.vtol_map import VTOLMap, ZoneType


def test_mode_switching_rules():
    """测试飞行模式切换规则"""
    print("=" * 60)
    print("测试飞行模式切换规则")
    print("=" * 60)
    
    vtol_map = VTOLMap()
    
    # 测试场景
    test_points = [
        (0, 0, "旋翼区中心"),           # 旋翼区
        (50, 0, "旋翼区边缘"),          # 旋翼区内
        (150, 0, "旋翼区外"),           # 自由空间
        (1200, 0, "downtown中心"),      # 居民区downtown
        (1800, 0, "suburb中心"),        # 居民区suburb
        (500, 500, "自由空间"),         # 自由空间
        (2500, 0, "地图外")            # 超出边界
    ]
    
    print("位置区域分析:")
    for x, y, description in test_points:
        zone_info = vtol_map.get_zone_info(x, y)
        in_bounds = vtol_map.is_in_bounds(x, y)
        can_use_multirotor = zone_info['type'] == ZoneType.MULTIROTOR
        safe_height = vtol_map.get_safe_flight_height(x, y)
        
        print(f"{description:15} ({x:4}, {y:4}): "
              f"区域={zone_info['name']:20} "
              f"边界内={str(in_bounds):5} "
              f"可用旋翼={str(can_use_multirotor):5} "
              f"安全高度={safe_height:4.1f}m")
    
    print("\n飞行模式切换规则总结:")
    print("✅ 旋翼区内: 可以使用旋翼模式和固定翼模式")
    print("❌ 旋翼区外: 只能使用固定翼模式")
    print("❌ 居民区内: 禁止飞行")


def test_path_planning_scenarios():
    """测试路径规划场景"""
    print("\n" + "=" * 60)
    print("测试关键路径规划场景")
    print("=" * 60)
    
    vtol_map = VTOLMap()
    
    scenarios = [
        {
            "name": "旋翼区内短距离",
            "start": (20, 20),
            "end": (80, 80),
            "expected_mode": "multirotor"
        },
        {
            "name": "旋翼区到自由空间",
            "start": (50, 0),
            "end": (500, 0),
            "expected_mode": "plane"
        },
        {
            "name": "自由空间长距离",
            "start": (300, 0),
            "end": (1500, 0),
            "expected_mode": "plane"
        },
        {
            "name": "绕过居民区",
            "start": (1000, 0),
            "end": (1400, 0),
            "expected_mode": "plane"
        },
        {
            "name": "远距离到旋翼区",
            "start": (1500, 500),
            "end": (50, 50),
            "expected_mode": "plane_then_multirotor"
        }
    ]
    
    for scenario in scenarios:
        print(f"\n场景: {scenario['name']}")
        start_x, start_y = scenario['start']
        end_x, end_y = scenario['end']
        
        # 分析起点和终点
        start_zone = vtol_map.get_zone_info(start_x, start_y)
        end_zone = vtol_map.get_zone_info(end_x, end_y)
        
        print(f"  起点: ({start_x}, {start_y}) - {start_zone['name']}")
        print(f"  终点: ({end_x}, {end_y}) - {end_zone['name']}")
        
        # 规划路径
        path = vtol_map.plan_safe_path(start_x, start_y, end_x, end_y)
        
        if path:
            print(f"  规划成功: {len(path)} 个航点")
            print(f"  推荐模式: {scenario['expected_mode']}")
            
            # 验证路径安全性
            is_safe = vtol_map.validate_flight_path(path)
            print(f"  路径安全: {'✅' if is_safe else '❌'}")
        else:
            print("  ❌ 路径规划失败")


def test_flight_strategy():
    """测试飞行策略"""
    print("\n" + "=" * 60)
    print("测试飞行策略建议")
    print("=" * 60)
    
    vtol_map = VTOLMap()
    
    # 典型任务场景
    mission_targets = [
        (0, 0, 0, "起飞点"),           # 旋翼区
        (1600, 200, 20, "北目标"),      # 自由空间
        (1600, -200, 20, "南目标"),     # 自由空间  
        (0, 0, 0, "降落点")            # 旋翼区
    ]
    
    print("任务航点分析:")
    for i, (x, y, z, name) in enumerate(mission_targets):
        zone_info = vtol_map.get_zone_info(x, y)
        is_safe = zone_info['type'] != ZoneType.RESIDENTIAL
        can_multirotor = zone_info['type'] == ZoneType.MULTIROTOR
        
        print(f"  {i+1}. {name:8} ({x:4}, {y:4}, {z:2}) - "
              f"{zone_info['name']:20} - "
              f"安全: {'✅' if is_safe else '❌'} - "
              f"旋翼模式: {'✅' if can_multirotor else '❌'}")
    
    print("\n飞行策略建议:")
    print("1. 起飞: 旋翼区内使用旋翼模式")
    print("2. 长距离飞行: 切换到固定翼模式，避开居民区")
    print("3. 目标点悬停: 保持固定翼模式（如在自由空间）")
    print("4. 返回降落: 接近旋翼区时切换回旋翼模式")


def test_residential_avoidance():
    """测试居民区避障"""
    print("\n" + "=" * 60)
    print("测试居民区避障算法")
    print("=" * 60)
    
    vtol_map = VTOLMap()
    
    # 测试穿越居民区的路径
    dangerous_paths = [
        ((800, 0), (1600, 0), "穿越downtown和suburb"),
        ((1100, -100), (1300, 100), "穿越downtown"),
        ((1700, -100), (1900, 100), "穿越suburb"),
        ((900, 200), (1900, -200), "斜穿两个居民区")
    ]
    
    for (start, end, description) in dangerous_paths:
        print(f"\n测试路径: {description}")
        print(f"  从 {start} 到 {end}")
        
        # 检查直线路径是否安全
        is_safe = vtol_map.is_line_safe(start[0], start[1], end[0], end[1])
        print(f"  直线路径安全: {'✅' if is_safe else '❌'}")
        
        if not is_safe:
            # 规划避障路径
            path = vtol_map.plan_safe_path(start[0], start[1], end[0], end[1])
            if path:
                print(f"  避障路径: {len(path)} 个航点")
                # 计算绕行距离
                direct_dist = vtol_map.distance_to_point(start[0], start[1], end[0], end[1])
                
                total_dist = 0
                prev_point = start
                for waypoint in path:
                    dist = vtol_map.distance_to_point(prev_point[0], prev_point[1], waypoint[0], waypoint[1])
                    total_dist += dist
                    prev_point = waypoint[:2]
                
                detour_ratio = total_dist / direct_dist if direct_dist > 0 else 1
                print(f"  绕行比例: {detour_ratio:.2f} (直线: {direct_dist:.0f}m, 实际: {total_dist:.0f}m)")
            else:
                print("  ❌ 无法找到安全路径")


def main():
    """主测试函数"""
    print("VTOL无人机飞行模式和路径规划验证测试")
    print("=" * 60)
    
    # 创建地图
    vtol_map = VTOLMap()
    vtol_map.print_map_summary()
    
    # 运行测试
    test_mode_switching_rules()
    test_path_planning_scenarios()
    test_flight_strategy()
    test_residential_avoidance()
    
    print("\n" + "=" * 60)
    print("关键规则总结")
    print("=" * 60)
    print("🔹 飞行模式规则:")
    print("   - 只有在旋翼区内才能使用旋翼模式")
    print("   - 旋翼区外必须使用固定翼模式")
    print("   - 居民区完全禁止飞行")
    print("\n🔹 路径规划策略:")
    print("   - 自动检测并避开居民区")
    print("   - 优先使用A*算法寻找最优路径")
    print("   - 备用简单绕行策略")
    print("   - 路径安全性验证")
    print("\n🔹 飞行任务流程:")
    print("   1. 旋翼区内起飞（旋翼模式）")
    print("   2. 切换固定翼模式进行长距离飞行")
    print("   3. 智能避开居民区")
    print("   4. 到达旋翼区时切换回旋翼模式降落")
    print("=" * 60)


if __name__ == "__main__":
    main()
