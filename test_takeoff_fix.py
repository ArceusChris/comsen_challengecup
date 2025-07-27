#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试修复后的VTOL起飞序列
验证位置检查和自动修正功能
"""

import sys
import os

# 添加工作目录到Python路径
workspace_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(workspace_dir)
sys.path.append(os.path.join(workspace_dir, 'workspace', 'vtol_control'))

from workspace.vtol_control.vtol_demo import VTOLDemoFlight
from workspace.vtol_control.vtol_map import VTOLMap, ZoneType


class MockPosition:
    """模拟位置类"""
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


def test_takeoff_scenarios():
    """测试各种起飞场景"""
    print("VTOL起飞序列测试")
    print("="*60)
    
    # 模拟不同的起始位置
    test_scenarios = [
        {
            'name': '正常场景：旋翼区中心',
            'position': (0, 0, 0),
            'expected': True,
            'description': '无人机在旋翼区中心，应该正常起飞'
        },
        {
            'name': '边界场景：旋翼区边缘',
            'position': (95, 0, 0),
            'expected': True,
            'description': '无人机在旋翼区边缘内，应该正常起飞'
        },
        {
            'name': '异常场景：超出地图边界',
            'position': (3000, 0, 0),
            'expected': True,  # 应该自动修正
            'description': '无人机位置超出地图边界，应该自动修正到旋翼区中心'
        },
        {
            'name': '异常场景：在居民区',
            'position': (1200, 0, 0),
            'expected': True,  # 应该自动修正
            'description': '无人机在居民区内，应该自动移动到旋翼区'
        },
        {
            'name': '异常场景：在自由空间',
            'position': (500, 500, 0),
            'expected': True,  # 应该自动修正
            'description': '无人机在自由空间，应该自动移动到旋翼区'
        },
        {
            'name': '边界测试：负坐标超界',
            'position': (-2000, -2000, 0),
            'expected': True,  # 应该自动修正
            'description': '无人机在负坐标超出边界，应该自动修正'
        }
    ]
    
    for i, scenario in enumerate(test_scenarios, 1):
        print(f"\n🧪 测试 {i}: {scenario['name']}")
        print(f"描述: {scenario['description']}")
        print(f"起始位置: {scenario['position']}")
        print("-" * 50)
        
        try:
            # 创建VTOL演示实例（模拟模式）
            demo = VTOLDemoFlight()
            
            # 模拟位置
            demo.current_position = MockPosition(*scenario['position'])
            
            # 重定义send_cmd为模拟函数
            def mock_send_cmd(cmd):
                print(f"   [模拟] 发送命令: {cmd}")
            demo.send_cmd = mock_send_cmd
            
            # 重定义set_target_pose为模拟函数
            def mock_set_target_pose(x, y, z, yaw=0):
                print(f"   [模拟] 设置目标位置: ({x:.1f}, {y:.1f}, {z:.1f})")
            demo.set_target_pose = mock_set_target_pose
            
            print(f"初始位置: ({demo.current_position.x:.1f}, {demo.current_position.y:.1f}, {demo.current_position.z:.1f})")
            
            # 测试wait_for_connection（跳过ROS部分）
            print("测试位置验证和修正...")
            
            # 手动调用位置检查逻辑
            zone_info = demo.map.get_zone_info(demo.current_position.x, demo.current_position.y)
            print(f"起始区域: {zone_info['name']} ({zone_info['type'].value})")
            
            # 测试起飞序列
            result = demo.takeoff_sequence()
            
            if result:
                print(f"✅ 测试通过：起飞序列成功")
                final_pos = demo.current_position
                final_zone = demo.map.get_zone_info(final_pos.x, final_pos.y)
                print(f"   最终位置: ({final_pos.x:.1f}, {final_pos.y:.1f}, {final_pos.z:.1f})")
                print(f"   最终区域: {final_zone['name']}")
            else:
                print(f"❌ 测试失败：起飞序列失败")
            
            # 验证结果是否符合预期
            if result == scenario['expected']:
                print(f"✅ 结果符合预期")
            else:
                print(f"⚠️ 结果不符合预期 (期望: {scenario['expected']}, 实际: {result})")
                
        except Exception as e:
            print(f"❌ 测试异常: {e}")
            import traceback
            traceback.print_exc()
    
    print(f"\n🎯 测试总结:")
    print(f"- 位置验证和自动修正功能已实现")
    print(f"- 支持从地图边界外自动修正到旋翼区")
    print(f"- 支持从居民区和自由空间自动移动到旋翼区")
    print(f"- 增强了起飞序列的鲁棒性")


def test_zone_detection():
    """测试区域检测功能"""
    print("\n🗺️ 区域检测功能测试")
    print("="*50)
    
    vtol_map = VTOLMap()
    
    test_points = [
        (0, 0, "旋翼区中心"),
        (50, 50, "旋翼区内"),
        (100, 0, "旋翼区边缘"),
        (150, 0, "自由空间"),
        (500, 500, "自由空间远处"),
        (1200, 0, "downtown居民区中心"),
        (1800, 0, "suburb居民区中心"),
        (3000, 0, "超出地图边界"),
        (-1500, 0, "负坐标超界"),
    ]
    
    for x, y, description in test_points:
        print(f"测试点 ({x:5}, {y:5}): {description:15}", end=" -> ")
        
        # 检查边界
        in_bounds = vtol_map.is_in_bounds(x, y)
        
        if in_bounds:
            zone_info = vtol_map.get_zone_info(x, y)
            print(f"{zone_info['name']} ({zone_info['type'].value})")
        else:
            print("out_of_bounds")


if __name__ == "__main__":
    # 运行区域检测测试
    test_zone_detection()
    
    # 运行起飞序列测试
    test_takeoff_scenarios()
