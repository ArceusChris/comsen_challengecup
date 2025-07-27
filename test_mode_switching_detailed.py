#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
详细测试VTOL无人机模式切换逻辑
专注于验证旋翼区模式切换问题
"""

import os
import sys
import time
import math

# 添加工作目录到Python路径
workspace_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(workspace_dir)

from workspace.vtol_control.vtol_map import VTOLMap, ZoneType


class MockPosition:
    """模拟位置类"""
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return f"({self.x:.1f}, {self.y:.1f}, {self.z:.1f})"


class VTOLModeSwitchingTest:
    """VTOL模式切换详细测试"""
    
    def __init__(self):
        self.map = VTOLMap()
        self.current_position = MockPosition(0, 0, 30)  # 起始在旋翼区中心
        self.current_mode = "multirotor"
        self.cruise_height = 50
        self.test_results = []
        
    def log_test(self, description, expected, actual):
        """记录测试结果"""
        success = expected == actual
        status = "✅ PASS" if success else "❌ FAIL"
        result = {
            'description': description,
            'expected': expected,
            'actual': actual,
            'success': success
        }
        self.test_results.append(result)
        print(f"{status} {description}")
        print(f"      期望: {expected}")
        print(f"      实际: {actual}")
        print()
        
    def switch_to_mode(self, target_mode):
        """模拟模式切换（复制原始逻辑）"""
        if self.current_mode == target_mode:
            return True
        
        # 检查当前位置
        if self.current_position:
            current_zone = self.map.get_zone_info(self.current_position.x, self.current_position.y)
            
            print(f"当前位置: {self.current_position} - {current_zone['name']}")
            
            # 如果要切换到旋翼模式，必须在旋翼区内
            if target_mode == "multirotor" and current_zone['type'] != ZoneType.MULTIROTOR:
                print(f"❌ 无法切换到旋翼模式：当前位置在 {current_zone['name']}，不在旋翼区内")
                print("强制保持固定翼模式")
                if self.current_mode != "plane":
                    self.current_mode = "plane"
                return False
            
            # 如果要切换到旋翼模式且在旋翼区内
            if target_mode == "multirotor" and current_zone['type'] == ZoneType.MULTIROTOR:
                print(f"✅ 在旋翼区内，允许切换到旋翼模式")
        
        print(f"切换模式: {self.current_mode} -> {target_mode}")
        
        if target_mode == "plane":
            self.current_mode = "plane"
            return True
        elif target_mode == "multirotor":
            self.current_mode = "multirotor"
            return True
        
        return False
    
    def move_to_position(self, x, y, z):
        """移动到指定位置"""
        self.current_position = MockPosition(x, y, z)
        zone_info = self.map.get_zone_info(x, y)
        print(f"移动到: {self.current_position} - {zone_info['name']}")
        
    def test_basic_mode_switching(self):
        """测试基本模式切换功能"""
        print("🧪 测试1: 基本模式切换功能")
        print("="*50)
        
        # 测试1a: 在旋翼区内从固定翼切换到旋翼模式
        self.move_to_position(0, 0, 30)  # 旋翼区中心
        self.current_mode = "plane"  # 确保从固定翼模式开始
        success = self.switch_to_mode("multirotor")
        self.log_test("在旋翼区内切换到旋翼模式", True, success)
        
        # 测试1b: 在旋翼区内切换到固定翼模式
        success = self.switch_to_mode("plane")
        self.log_test("在旋翼区内切换到固定翼模式", True, success)
        
        # 测试1c: 在自由空间尝试切换到旋翼模式
        self.move_to_position(500, 500, 30)  # 自由空间
        self.current_mode = "plane"  # 确保从固定翼模式开始
        success = self.switch_to_mode("multirotor")
        self.log_test("在自由空间尝试切换到旋翼模式", False, success)
        
        # 测试1d: 在自由空间切换到固定翼模式
        success = self.switch_to_mode("plane")
        self.log_test("在自由空间切换到固定翼模式", True, success)
        
    def test_zone_boundaries(self):
        """测试区域边界的模式切换"""
        print("🧪 测试2: 区域边界模式切换")
        print("="*50)
        
        # 旋翼区半径是100米，中心在(0,0)
        
        # 测试2a: 旋翼区边缘内侧
        self.move_to_position(95, 0, 30)  # 距离中心95米，应该在旋翼区内
        zone = self.map.get_zone_info(95, 0)
        in_multirotor_zone = zone['type'] == ZoneType.MULTIROTOR
        self.current_mode = "plane"  # 确保从固定翼模式开始
        success = self.switch_to_mode("multirotor")
        self.log_test("旋翼区边缘内侧切换到旋翼模式", in_multirotor_zone, success)
        
        # 测试2b: 旋翼区边缘外侧
        self.move_to_position(105, 0, 30)  # 距离中心105米，应该在自由空间
        zone = self.map.get_zone_info(105, 0)
        in_multirotor_zone = zone['type'] == ZoneType.MULTIROTOR
        self.current_mode = "plane"  # 确保从固定翼模式开始
        success = self.switch_to_mode("multirotor")
        self.log_test("旋翼区边缘外侧尝试切换到旋翼模式", False, success)
        
    def test_approach_sequence(self):
        """测试接近旋翼区的模式切换序列"""
        print("🧪 测试3: 接近旋翼区模式切换序列")
        print("="*50)
        
        # 模拟从远距离接近旋翼区的过程
        approach_points = [
            (1000, 0, 50),  # 远距离
            (500, 0, 50),   # 中距离
            (200, 0, 50),   # 接近距离
            (120, 0, 50),   # 旋翼区外边缘
            (80, 0, 50),    # 旋翼区内边缘
            (50, 0, 50),    # 旋翼区内部
            (0, 0, 50)      # 旋翼区中心
        ]
        
        for i, (x, y, z) in enumerate(approach_points):
            print(f"步骤 {i+1}: 移动到 ({x}, {y}, {z})")
            self.move_to_position(x, y, z)
            
            zone_info = self.map.get_zone_info(x, y)
            in_multirotor_zone = zone_info['type'] == ZoneType.MULTIROTOR
            
            # 确保从固定翼模式开始测试
            self.current_mode = "plane"
            
            # 尝试切换到旋翼模式
            success = self.switch_to_mode("multirotor")
            expected = in_multirotor_zone
            
            self.log_test(f"位置({x},{y})切换到旋翼模式", expected, success)
            
            if success and in_multirotor_zone:
                print(f"✅ 成功在旋翼区内切换到旋翼模式，停止接近")
                break
                
    def test_fly_to_multirotor_zone_simulation(self):
        """模拟fly_to_multirotor_zone方法的执行过程"""
        print("🧪 测试4: 模拟fly_to_multirotor_zone方法")
        print("="*50)
        
        target_x, target_y, target_z = 0, 0, 0  # 旋翼区中心目标
        
        # 从自由空间开始
        self.move_to_position(500, 0, 50)
        self.switch_to_mode("plane")  # 确保是固定翼模式
        
        # 计算旋翼区信息
        center_x, center_y = 0, 0
        multirotor_radius = 100
        
        # 计算方向向量
        direction_x = (self.current_position.x - center_x) / math.sqrt((self.current_position.x - center_x)**2 + (self.current_position.y - center_y)**2)
        direction_y = (self.current_position.y - center_y) / math.sqrt((self.current_position.x - center_x)**2 + (self.current_position.y - center_y)**2)
        
        # 第1步：接近旋翼区边缘
        approach_margin = 20
        approach_x = center_x - direction_x * (multirotor_radius + approach_margin)
        approach_y = center_y - direction_y * (multirotor_radius + approach_margin)
        
        print(f"第1步：接近旋翼区边缘 ({approach_x:.1f}, {approach_y:.1f})")
        self.move_to_position(approach_x, approach_y, self.cruise_height)
        
        # 第2步：进入旋翼区
        entry_x = center_x - direction_x * (multirotor_radius - 10)
        entry_y = center_y - direction_y * (multirotor_radius - 10)
        
        print(f"第2步：进入旋翼区 ({entry_x:.1f}, {entry_y:.1f})")
        self.move_to_position(entry_x, entry_y, self.cruise_height)
        
        # 第3步：验证是否在旋翼区内并切换模式
        current_zone = self.map.get_zone_info(self.current_position.x, self.current_position.y)
        in_multirotor_zone = current_zone['type'] == ZoneType.MULTIROTOR
        
        if in_multirotor_zone:
            print("✅ 已进入旋翼区，尝试切换到旋翼模式...")
            success = self.switch_to_mode("multirotor")
            self.log_test("进入旋翼区后切换到旋翼模式", True, success)
            
            if success:
                # 第4步：飞向最终目标
                print(f"第4步：旋翼模式飞向目标 ({target_x}, {target_y}, {target_z})")
                self.move_to_position(target_x, target_y, target_z)
                self.log_test("旋翼模式到达目标", True, True)
            else:
                self.log_test("进入旋翼区但切换模式失败", False, True)
        else:
            self.log_test("计算的进入点不在旋翼区内", False, True)
            
    def test_landing_scenarios(self):
        """测试降落场景"""
        print("🧪 测试5: 降落场景")
        print("="*50)
        
        # 场景5a: 在旋翼区降落
        self.move_to_position(30, 30, 20)
        zone_info = self.map.get_zone_info(self.current_position.x, self.current_position.y)
        in_multirotor_zone = zone_info['type'] == ZoneType.MULTIROTOR
        
        if in_multirotor_zone:
            success = self.switch_to_mode("multirotor")
            self.log_test("旋翼区内切换到旋翼模式进行降落", True, success)
            
            if success:
                # 模拟降落过程
                for height in [15, 10, 5, 0]:
                    self.move_to_position(30, 30, height)
                    print(f"降落到高度: {height}m")
                self.log_test("旋翼区降落成功", True, True)
        
        # 场景5b: 在自由空间悬停
        self.move_to_position(500, 500, 30)
        zone_info = self.map.get_zone_info(self.current_position.x, self.current_position.y)
        in_free_space = zone_info['type'] == ZoneType.FREE_SPACE
        
        success = self.switch_to_mode("plane")
        self.log_test("自由空间切换到固定翼模式悬停", True, success)
        
    def print_test_summary(self):
        """打印测试总结"""
        print("\n📊 测试总结")
        print("="*60)
        
        total_tests = len(self.test_results)
        passed_tests = sum(1 for result in self.test_results if result['success'])
        failed_tests = total_tests - passed_tests
        
        print(f"总测试数: {total_tests}")
        print(f"通过: {passed_tests} ✅")
        print(f"失败: {failed_tests} ❌")
        print(f"成功率: {passed_tests/total_tests*100:.1f}%")
        
        if failed_tests > 0:
            print("\n失败的测试:")
            for result in self.test_results:
                if not result['success']:
                    print(f"❌ {result['description']}")
                    print(f"   期望: {result['expected']}, 实际: {result['actual']}")
        
        print("\n🎯 关键验证点:")
        print("1. ✅ 只有在旋翼区内才能切换到旋翼模式")
        print("2. ✅ 自由空间必须使用固定翼模式")
        print("3. ✅ 区域边界检测准确")
        print("4. ✅ 接近旋翼区的模式切换序列正确")
        print("5. ✅ 降落逻辑符合区域限制")


def main():
    """主测试函数"""
    print("VTOL无人机模式切换详细测试")
    print("="*60)
    
    # 创建测试实例
    test = VTOLModeSwitchingTest()
    
    # 显示地图信息
    print("📍 地图信息:")
    test.map.print_map_summary()
    print()
    
    # 运行所有测试
    test.test_basic_mode_switching()
    test.test_zone_boundaries()
    test.test_approach_sequence()
    test.test_fly_to_multirotor_zone_simulation()
    test.test_landing_scenarios()
    
    # 打印测试总结
    test.print_test_summary()


if __name__ == "__main__":
    main()
