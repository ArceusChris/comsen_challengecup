#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
无人机导航功能测试脚本
测试新的瞬间距离判定逻辑
"""

import sys
import os
import time

# 添加路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 模拟ROS环境（如果没有真实ROS环境）
class MockROSNode:
    def __init__(self):
        self.current_pos = [0, 0, 0]
        self.target_pos = [0, 0, 0]
        
    def set_position(self, x, y, z):
        """模拟设置无人机位置"""
        self.current_pos = [x, y, z]
        
    def get_position(self):
        """模拟获取无人机位置"""
        return self.current_pos
        
    def set_target(self, x, y, z):
        """模拟设置目标位置"""
        self.target_pos = [x, y, z]

def test_distance_judgment():
    """测试距离判定逻辑"""
    print("🧪 测试瞬间距离判定逻辑")
    
    # 模拟一组无人机位置和目标位置
    test_cases = [
        {
            "name": "近距离测试",
            "start": [0, 0, 5],
            "target": [5, 5, 5],
            "expect": True
        },
        {
            "name": "中距离测试", 
            "start": [0, 0, 5],
            "target": [15, 15, 5],
            "expect": False
        },
        {
            "name": "边界距离测试",
            "start": [0, 0, 5], 
            "target": [7, 7, 5],
            "expect": True
        }
    ]
    
    for case in test_cases:
        print(f"\n📋 {case['name']}")
        start_x, start_y, start_z = case['start']
        target_x, target_y, target_z = case['target']
        
        # 计算距离
        distance = ((target_x - start_x)**2 + (target_y - start_y)**2 + (target_z - start_z)**2)**0.5
        reached = distance <= 20.0
        
        print(f"  起始位置: ({start_x}, {start_y}, {start_z})")
        print(f"  目标位置: ({target_x}, {target_y}, {target_z})")
        print(f"  距离: {distance:.1f}m")
        print(f"  预期结果: {case['expect']}")
        print(f"  实际结果: {reached}")
        print(f"  测试结果: {'✅ 通过' if reached == case['expect'] else '❌ 失败'}")

def simulate_navigation_path():
    """模拟导航路径"""
    print("\n🚁 模拟导航路径测试")
    
    # 模拟无人机逐渐接近目标点的过程
    target = [50, 50, 10]
    trajectory = [
        [0, 0, 5],      # 起始点
        [10, 10, 8],    # 接近中
        [25, 25, 9],    # 继续接近
        [40, 40, 10],   # 很接近
        [48, 47, 10],   # 进入10m范围
        [51, 52, 10],   # 超越目标
        [45, 45, 10],   # 回到附近
    ]
    
    tolerance = 20.0
    print(f"目标位置: ({target[0]}, {target[1]}, {target[2]})")
    print(f"距离阈值: {tolerance}m")
    print("\n轨迹分析：")
    
    reached = False
    for i, pos in enumerate(trajectory):
        distance = ((pos[0] - target[0])**2 + (pos[1] - target[1])**2 + (pos[2] - target[2])**2)**0.5
        within_range = distance <= tolerance
        
        status = "🎯 到达" if within_range else "🔄 接近中"
        print(f"  第{i+1}步: ({pos[0]:2.0f}, {pos[1]:2.0f}, {pos[2]:2.0f}) | 距离: {distance:5.1f}m | {status}")
        
        if within_range and not reached:
            print(f"    ✅ 首次进入阈值范围！按新逻辑此时即认为到达目标")
            reached = True
    
    if not reached:
        print("    ❌ 整个轨迹都未能接近目标")

def main():
    """主测试函数"""
    print("=" * 60)
    print("🧪 无人机导航新判定逻辑测试")
    print("=" * 60)
    
    # 测试距离判定逻辑
    test_distance_judgment()
    
    # 模拟导航路径
    simulate_navigation_path()
    
    print("\n" + "=" * 60)
    print("📋 新判定逻辑说明：")
    print("  ✅ 优点：无人机只要瞬间进入20m范围即认为到达，避免了反复徘徊")
    print("  ⚠️ 注意：如果无人机速度过快可能穿越目标点，需要合理的飞行速度控制")
    print("  🎯 适用：多点导航任务，快速通过各个航点")
    print("=" * 60)

if __name__ == "__main__":
    main()
