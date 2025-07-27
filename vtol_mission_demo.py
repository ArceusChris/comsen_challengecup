#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
VTOL无人机完整任务演示脚本
演示严格的飞行模式切换规则和智能路径规划
"""

import os
import sys
import time

# 添加工作目录到Python路径
workspace_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(workspace_dir)

from workspace.vtol_control.vtol_map import VTOLMap, ZoneType


class VTOLMissionSimulator:
    """VTOL任务模拟器"""
    
    def __init__(self):
        self.map = VTOLMap()
        self.current_position = (0, 0, 0)  # 起始位置在旋翼区
        self.current_mode = "multirotor"   # 起始模式
        self.flight_log = []
        
    def log_event(self, event_type, message):
        """记录飞行事件"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {event_type}: {message}"
        self.flight_log.append(log_entry)
        print(log_entry)
    
    def check_mode_switch_allowed(self, target_mode):
        """检查模式切换是否允许"""
        x, y, z = self.current_position
        zone_info = self.map.get_zone_info(x, y)
        
        # 只有在旋翼区内才能切换到旋翼模式
        if target_mode == "multirotor" and zone_info['type'] != ZoneType.MULTIROTOR:
            return False, f"无法切换到旋翼模式：当前在{zone_info['name']}"
        
        return True, "模式切换允许"
    
    def switch_mode(self, target_mode):
        """切换飞行模式"""
        if self.current_mode == target_mode:
            return True
        
        allowed, reason = self.check_mode_switch_allowed(target_mode)
        if not allowed:
            self.log_event("模式切换失败", reason)
            return False
        
        self.log_event("模式切换", f"{self.current_mode} -> {target_mode}")
        self.current_mode = target_mode
        return True
    
    def move_to_position(self, x, y, z):
        """移动到指定位置"""
        old_pos = self.current_position
        self.current_position = (x, y, z)
        
        zone_info = self.map.get_zone_info(x, y)
        distance = ((x - old_pos[0])**2 + (y - old_pos[1])**2 + (z - old_pos[2])**2)**0.5
        
        self.log_event("位置更新", 
                      f"从({old_pos[0]:.0f},{old_pos[1]:.0f},{old_pos[2]:.0f}) "
                      f"到({x:.0f},{y:.0f},{z:.0f}) "
                      f"距离{distance:.0f}m - {zone_info['name']}")
    
    def takeoff(self):
        """起飞序列"""
        self.log_event("任务开始", "VTOL无人机起飞序列")
        
        # 检查起飞位置
        x, y, z = self.current_position
        zone_info = self.map.get_zone_info(x, y)
        
        if zone_info['type'] != ZoneType.MULTIROTOR:
            self.log_event("起飞失败", f"不在旋翼区内，当前位置：{zone_info['name']}")
            return False
        
        # 确保旋翼模式
        if not self.switch_mode("multirotor"):
            return False
        
        # 逐步起飞
        takeoff_heights = [5, 15, 30]
        for height in takeoff_heights:
            self.move_to_position(x, y, height)
            self.log_event("起飞进程", f"到达高度 {height}m")
        
        self.log_event("起飞完成", "无人机已准备好执行任务")
        return True
    
    def fly_to_target(self, target_x, target_y, target_z, target_name):
        """飞向目标点"""
        self.log_event("导航开始", f"飞向 {target_name} ({target_x}, {target_y}, {target_z})")
        
        # 安全检查
        target_zone = self.map.get_zone_info(target_x, target_y)
        if target_zone['type'] == ZoneType.RESIDENTIAL:
            self.log_event("导航失败", f"目标在居民区 {target_zone['name']} 内，任务中止")
            return False
        
        # 分析飞行模式需求
        current_x, current_y, current_z = self.current_position
        current_zone = self.map.get_zone_info(current_x, current_y)
        
        # 飞行模式决策
        if current_zone['type'] == ZoneType.MULTIROTOR and target_zone['type'] == ZoneType.MULTIROTOR:
            # 旋翼区内飞行
            self.log_event("飞行策略", "旋翼区内短距离飞行，使用旋翼模式")
            return self._fly_multirotor_mode(target_x, target_y, target_z)
        
        elif current_zone['type'] == ZoneType.MULTIROTOR and target_zone['type'] != ZoneType.MULTIROTOR:
            # 从旋翼区飞出
            self.log_event("飞行策略", "从旋翼区飞出，先在边缘切换固定翼模式")
            return self._fly_from_multirotor_zone(target_x, target_y, target_z)
        
        elif current_zone['type'] != ZoneType.MULTIROTOR and target_zone['type'] == ZoneType.MULTIROTOR:
            # 飞向旋翼区
            self.log_event("飞行策略", "飞向旋翼区，固定翼接近后切换旋翼模式")
            return self._fly_to_multirotor_zone(target_x, target_y, target_z)
        
        else:
            # 旋翼区外飞行
            self.log_event("飞行策略", "旋翼区外长距离飞行，使用固定翼模式")
            return self._fly_plane_mode(target_x, target_y, target_z)
    
    def _fly_multirotor_mode(self, target_x, target_y, target_z):
        """旋翼模式飞行"""
        if not self.switch_mode("multirotor"):
            return False
        
        self.move_to_position(target_x, target_y, target_z)
        self.log_event("导航完成", "旋翼模式精确到达目标")
        return True
    
    def _fly_from_multirotor_zone(self, target_x, target_y, target_z):
        """从旋翼区飞出"""
        # 计算旋翼区边缘点
        center_x, center_y = self.map.multirotor_center
        radius = self.map.multirotor_radius
        
        current_x, current_y, current_z = self.current_position
        direction_x = target_x - center_x
        direction_y = target_y - center_y
        direction_length = (direction_x**2 + direction_y**2)**0.5
        
        if direction_length > 0:
            direction_x /= direction_length
            direction_y /= direction_length
            
            # 旋翼区边缘点
            edge_x = center_x + direction_x * (radius - 10)
            edge_y = center_y + direction_y * (radius - 10)
        else:
            edge_x, edge_y = current_x, current_y
        
        # 步骤1：旋翼模式飞到边缘
        self.log_event("边缘接近", f"旋翼模式飞向边缘点 ({edge_x:.0f}, {edge_y:.0f})")
        self.move_to_position(edge_x, edge_y, 50)
        
        # 步骤2：切换固定翼模式
        if not self.switch_mode("plane"):
            return False
        
        # 步骤3：固定翼模式飞向目标
        return self._fly_plane_mode_with_avoidance(target_x, target_y, target_z)
    
    def _fly_to_multirotor_zone(self, target_x, target_y, target_z):
        """飞向旋翼区"""
        # 确保固定翼模式
        self.switch_mode("plane")
        
        # 计算旋翼区接近点
        center_x, center_y = self.map.multirotor_center
        radius = self.map.multirotor_radius
        
        current_x, current_y, current_z = self.current_position
        direction_x = center_x - current_x
        direction_y = center_y - current_y
        direction_length = (direction_x**2 + direction_y**2)**0.5
        
        if direction_length > 0:
            direction_x /= direction_length
            direction_y /= direction_length
            
            # 旋翼区外接近点
            approach_x = center_x - direction_x * (radius + 20)
            approach_y = center_y - direction_y * (radius + 20)
        else:
            approach_x, approach_y = current_x, current_y
        
        # 步骤1：固定翼模式接近旋翼区
        self.log_event("旋翼区接近", f"固定翼模式接近 ({approach_x:.0f}, {approach_y:.0f})")
        self._fly_plane_mode_with_avoidance(approach_x, approach_y, 50)
        
        # 步骤2：进入旋翼区
        entry_x = center_x - direction_x * (radius - 10)
        entry_y = center_y - direction_y * (radius - 10)
        self.move_to_position(entry_x, entry_y, 50)
        
        # 步骤3：切换旋翼模式
        if not self.switch_mode("multirotor"):
            self.log_event("模式切换警告", "无法切换旋翼模式，继续用固定翼模式")
            self.move_to_position(target_x, target_y, max(20, target_z))
            return True
        
        # 步骤4：旋翼模式精确飞向目标
        self.move_to_position(target_x, target_y, target_z)
        self.log_event("导航完成", "旋翼模式精确到达目标")
        return True
    
    def _fly_plane_mode(self, target_x, target_y, target_z):
        """固定翼模式飞行"""
        if not self.switch_mode("plane"):
            return False
        
        return self._fly_plane_mode_with_avoidance(target_x, target_y, target_z)
    
    def _fly_plane_mode_with_avoidance(self, target_x, target_y, target_z):
        """带避障的固定翼模式飞行"""
        current_x, current_y, current_z = self.current_position
        
        # 规划安全路径
        self.log_event("路径规划", f"规划从({current_x:.0f},{current_y:.0f})到({target_x:.0f},{target_y:.0f})的安全路径")
        waypoints = self.map.plan_safe_path(current_x, current_y, target_x, target_y)
        
        if not waypoints:
            self.log_event("路径规划失败", "无法找到安全路径")
            return False
        
        # 验证路径安全性
        is_safe = self.map.validate_flight_path(waypoints)
        if not is_safe:
            self.log_event("路径安全警告", "规划的路径存在潜在风险")
        
        # 执行航点导航
        for i, (wp_x, wp_y, wp_z) in enumerate(waypoints):
            zone_info = self.map.get_zone_info(wp_x, wp_y)
            if zone_info['type'] == ZoneType.RESIDENTIAL:
                self.log_event("航点跳过", f"航点{i+1}在居民区内，跳过以避免坠毁")
                continue
            
            self.move_to_position(wp_x, wp_y, max(wp_z, 20))  # 确保安全高度
            self.log_event("航点到达", f"航点{i+1}/{len(waypoints)} - {zone_info['name']}")
        
        self.log_event("导航完成", "固定翼模式到达目标区域")
        return True
    
    def land(self):
        """降落序列"""
        x, y, z = self.current_position
        zone_info = self.map.get_zone_info(x, y)
        
        if zone_info['type'] == ZoneType.MULTIROTOR:
            self.log_event("降落开始", "在旋翼区内执行降落")
            
            if not self.switch_mode("multirotor"):
                self.log_event("降落受限", "无法切换旋翼模式，保持安全高度悬停")
                self.move_to_position(x, y, 20)
                return False
            
            # 逐步降落
            for height in [15, 5, 0]:
                self.move_to_position(x, y, height)
                self.log_event("降落进程", f"下降到高度 {height}m")
            
            self.log_event("降落完成", "无人机安全着陆")
            return True
        else:
            self.log_event("降落受限", f"在{zone_info['name']}无法降落，保持安全高度")
            self.move_to_position(x, y, max(20, z))
            return False
    
    def execute_mission(self, targets):
        """执行完整任务"""
        self.log_event("任务启动", f"开始执行包含{len(targets)}个目标点的任务")
        
        # 起飞
        if not self.takeoff():
            self.log_event("任务中止", "起飞失败")
            return False
        
        # 依次飞向目标点
        for i, (x, y, z, name) in enumerate(targets):
            self.log_event("任务进度", f"执行目标{i+1}/{len(targets)}: {name}")
            
            if not self.fly_to_target(x, y, z, name):
                self.log_event("任务警告", f"目标{name}未完全到达")
            
            # 在目标点停留
            self.log_event("目标到达", f"在{name}停留3秒")
        
        # 降落
        self.land()
        
        self.log_event("任务完成", "所有任务目标已完成")
        return True
    
    def print_mission_summary(self):
        """打印任务总结"""
        print("\n" + "="*60)
        print("任务执行日志")
        print("="*60)
        for log_entry in self.flight_log:
            print(log_entry)
        print("="*60)


def main():
    """主演示函数"""
    print("VTOL无人机完整任务演示")
    print("="*60)
    print("规则:")
    print("✅ 只有在旋翼区内才能使用旋翼模式")
    print("❌ 旋翼区外必须使用固定翼模式")
    print("❌ 居民区完全禁止飞行")
    print("🔄 智能路径规划避开居民区")
    print("="*60)
    
    # 创建任务模拟器
    simulator = VTOLMissionSimulator()
    
    # 打印地图信息
    simulator.map.print_map_summary()
    
    # 定义任务目标点
    mission_targets = [
        (1600, 200, 20, "北侧目标点"),
        (1600, -200, 20, "南侧目标点"),
        (0, 0, 0, "返回降落点")
    ]
    
    print(f"\n任务目标点分析:")
    for i, (x, y, z, name) in enumerate(mission_targets):
        zone_info = simulator.map.get_zone_info(x, y)
        safe = "✅" if zone_info['type'] != ZoneType.RESIDENTIAL else "❌"
        multirotor_ok = "✅" if zone_info['type'] == ZoneType.MULTIROTOR else "❌"
        print(f"  {i+1}. {name}: ({x}, {y}, {z}) - {zone_info['name']} - 安全:{safe} 旋翼:{multirotor_ok}")
    
    # 执行任务
    print(f"\n开始执行VTOL无人机演示任务...")
    print("="*60)
    
    success = simulator.execute_mission(mission_targets)
    
    # 打印任务总结
    simulator.print_mission_summary()
    
    print(f"\n任务结果: {'✅ 成功' if success else '❌ 失败'}")
    print("="*60)


if __name__ == "__main__":
    main()
