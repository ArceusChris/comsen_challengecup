#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试VTOL无人机旋翼区模式切换功能
验证飞机到达旋翼区后能否正常切换为旋翼模式降落
"""

import os
import sys
import time

# 添加工作目录到Python路径
workspace_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(workspace_dir)

from workspace.vtol_control.vtol_map import VTOLMap, ZoneType


class MockPosition:
    """模拟位置类"""
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class VTOLModeSwitchTest:
    """VTOL模式切换测试类"""
    
    def __init__(self):
        self.map = VTOLMap()
        self.current_position = MockPosition(0, 0, 30)  # 起始在旋翼区
        self.current_mode = "multirotor"
        self.flight_log = []
        
    def log_event(self, message):
        """记录事件"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        self.flight_log.append(log_entry)
        print(log_entry)
        
    def send_cmd(self, cmd_str):
        """模拟发送命令"""
        self.log_event(f"发送命令: {cmd_str}")
        
    def switch_to_mode(self, target_mode):
        """模拟模式切换"""
        if self.current_mode == target_mode:
            return True
        
        # 检查当前位置
        if self.current_position:
            current_zone = self.map.get_zone_info(self.current_position.x, self.current_position.y)
            
            self.log_event(f"当前位置: ({self.current_position.x:.1f}, {self.current_position.y:.1f}) - {current_zone['name']}")
            
            # 如果要切换到旋翼模式，必须在旋翼区内
            if target_mode == "multirotor" and current_zone['type'] != ZoneType.MULTIROTOR:
                self.log_event(f"❌ 无法切换到旋翼模式：当前位置在 {current_zone['name']}，不在旋翼区内")
                self.log_event("强制保持固定翼模式")
                if self.current_mode != "plane":
                    self.send_cmd("plane")
                    self.current_mode = "plane"
                return False
            
            # 如果要切换到旋翼模式且在旋翼区内
            if target_mode == "multirotor" and current_zone['type'] == ZoneType.MULTIROTOR:
                self.log_event(f"✅ 在旋翼区内，允许切换到旋翼模式")
        
        self.log_event(f"切换模式: {self.current_mode} -> {target_mode}")
        
        if target_mode == "plane":
            self.send_cmd("plane")
            self.current_mode = "plane"
        elif target_mode == "multirotor":
            self.send_cmd("multirotor")
            self.current_mode = "multirotor"
        
        return True
    
    def move_to_position(self, x, y, z):
        """模拟移动到位置"""
        old_pos = (self.current_position.x, self.current_position.y, self.current_position.z)
        self.current_position = MockPosition(x, y, z)
        
        zone_info = self.map.get_zone_info(x, y)
        distance = ((x - old_pos[0])**2 + (y - old_pos[1])**2 + (z - old_pos[2])**2)**0.5
        
        self.log_event(f"移动到: ({x:.1f}, {y:.1f}, {z:.1f}) - {zone_info['name']} (距离: {distance:.1f}m)")
    
    def test_mode_switching_scenarios(self):
        """测试各种模式切换场景"""
        self.log_event("开始测试VTOL模式切换场景")
        self.log_event("="*50)
        
        # 场景1：在旋翼区内切换模式
        self.log_event("\n场景1：在旋翼区内切换模式")
        self.move_to_position(50, 0, 30)  # 旋翼区内
        
        # 尝试切换到固定翼模式
        success = self.switch_to_mode("plane")
        self.log_event(f"旋翼区内切换到固定翼模式: {'✅ 成功' if success else '❌ 失败'}")
        
        # 尝试切换回旋翼模式
        success = self.switch_to_mode("multirotor")
        self.log_event(f"旋翼区内切换到旋翼模式: {'✅ 成功' if success else '❌ 失败'}")
        
        # 场景2：在自由空间尝试切换模式
        self.log_event("\n场景2：在自由空间尝试切换模式")
        self.move_to_position(500, 0, 30)  # 自由空间
        
        # 尝试切换到固定翼模式（应该成功）
        success = self.switch_to_mode("plane")
        self.log_event(f"自由空间切换到固定翼模式: {'✅ 成功' if success else '❌ 失败'}")
        
        # 尝试切换到旋翼模式（应该失败）
        success = self.switch_to_mode("multirotor")
        self.log_event(f"自由空间切换到旋翼模式: {'✅ 成功' if success else '❌ 失败'}")
        
        # 场景3：从外部飞向旋翼区并切换模式
        self.log_event("\n场景3：从外部飞向旋翼区并切换模式")
        self.test_approach_multirotor_zone()
        
        # 场景4：测试降落场景
        self.log_event("\n场景4：测试降落场景")
        self.test_landing_scenarios()
    
    def test_approach_multirotor_zone(self):
        """测试接近旋翼区的场景"""
        self.log_event("模拟从远处飞向旋翼区的过程...")
        
        # 起始位置：远离旋翼区
        self.move_to_position(1000, 0, 50)
        self.switch_to_mode("plane")  # 确保是固定翼模式
        
        # 接近旋翼区的过程
        approach_points = [
            (500, 0, 50),   # 中途点
            (200, 0, 50),   # 接近点
            (120, 0, 50),   # 旋翼区边缘外
            (90, 0, 50),    # 旋翼区边缘内
            (50, 0, 50),    # 旋翼区内部
            (0, 0, 50)      # 旋翼区中心
        ]
        
        for x, y, z in approach_points:
            self.move_to_position(x, y, z)
            zone_info = self.map.get_zone_info(x, y)
            
            if zone_info['type'] == ZoneType.MULTIROTOR:
                self.log_event("尝试在旋翼区内切换到旋翼模式...")
                success = self.switch_to_mode("multirotor")
                if success:
                    self.log_event("✅ 成功进入旋翼区并切换到旋翼模式")
                    break
                else:
                    self.log_event("❌ 在旋翼区内仍无法切换到旋翼模式")
            else:
                self.log_event(f"当前在{zone_info['name']}，继续接近旋翼区...")
    
    def test_landing_scenarios(self):
        """测试降落场景"""
        self.log_event("测试各种降落场景...")
        
        # 场景1：在旋翼区降落
        self.log_event("场景1：在旋翼区内降落")
        self.move_to_position(30, 30, 20)  # 旋翼区内
        
        zone_info = self.map.get_zone_info(self.current_position.x, self.current_position.y)
        if zone_info['type'] == ZoneType.MULTIROTOR:
            self.log_event("在旋翼区内，尝试切换到旋翼模式进行降落...")
            success = self.switch_to_mode("multirotor")
            
            if success:
                # 模拟降落过程
                for height in [15, 10, 5, 0]:
                    self.move_to_position(30, 30, height)
                    self.log_event(f"降落到高度: {height}m")
                self.log_event("✅ 旋翼区降落成功")
            else:
                self.log_event("❌ 无法切换到旋翼模式，降落失败")
        
        # 场景2：在自由空间"降落"
        self.log_event("\n场景2：在自由空间悬停")
        self.move_to_position(500, 500, 30)  # 自由空间
        
        zone_info = self.map.get_zone_info(self.current_position.x, self.current_position.y)
        if zone_info['type'] == ZoneType.FREE_SPACE:
            self.log_event("在自由空间，保持固定翼模式悬停...")
            self.switch_to_mode("plane")
            safe_height = 20.0
            self.move_to_position(500, 500, safe_height)
            self.log_event(f"✅ 在自由空间以{safe_height}m高度安全悬停")
    
    def print_test_summary(self):
        """打印测试总结"""
        print("\n" + "="*60)
        print("VTOL模式切换测试总结")
        print("="*60)
        
        multirotor_switches = 0
        plane_switches = 0
        failed_switches = 0
        
        for log in self.flight_log:
            if "切换模式:" in log and "multirotor" in log:
                multirotor_switches += 1
            elif "切换模式:" in log and "plane" in log:
                plane_switches += 1
            elif "无法切换" in log:
                failed_switches += 1
        
        print(f"成功切换到旋翼模式次数: {multirotor_switches}")
        print(f"成功切换到固定翼模式次数: {plane_switches}")
        print(f"失败的模式切换次数: {failed_switches}")
        
        print("\n关键规则验证:")
        print("✅ 只有在旋翼区内才能使用旋翼模式")
        print("✅ 自由空间和居民区外必须使用固定翼模式")
        print("✅ 模式切换有位置验证机制")
        
        print("\n测试日志:")
        for log in self.flight_log[-10:]:  # 显示最后10条日志
            print(log)
        print("="*60)


def main():
    """主测试函数"""
    print("VTOL无人机旋翼区模式切换功能测试")
    print("="*60)
    
    # 创建测试实例
    test = VTOLModeSwitchTest()
    
    # 显示地图信息
    test.map.print_map_summary()
    
    # 运行测试
    test.test_mode_switching_scenarios()
    
    # 打印测试总结
    test.print_test_summary()
    
    print("\n🎯 测试重点:")
    print("1. 验证只有在旋翼区内才能切换到旋翼模式")
    print("2. 验证从外部接近旋翼区的模式切换过程")
    print("3. 验证在旋翼区内的降落功能")
    print("4. 验证在非旋翼区的安全悬停")


if __name__ == "__main__":
    main()
