#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
给定起点和终点，根据map中定义的区域信息，利用A*算法进行路径规划，避开居民区和其他障碍物。
将规划后的路径可视化，默认起点在(0, 0, 30)，目标点在(1600, 200, 30)。
'''

import heapq
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from vtol_map import VTOLMap, ZoneType


class Node:
    """A*算法节点类"""
    def __init__(self, x, y, g_cost=0, h_cost=0, parent=None):
        self.x = x
        self.y = y
        self.g_cost = g_cost  # 从起点到当前节点的实际代价
        self.h_cost = h_cost  # 从当前节点到终点的启发式代价
        self.f_cost = g_cost + h_cost  # 总代价
        self.parent = parent
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __repr__(self):
        return f"Node({self.x}, {self.y}, f={self.f_cost:.1f})"


class VTOLAstarPlanner:
    """VTOL无人机A*路径规划器"""
    
    def __init__(self, grid_size=10):
        """
        初始化路径规划器
        
        Args:
            grid_size: 网格大小（米），影响路径精度和计算复杂度
        """
        self.map = VTOLMap()
        self.grid_size = grid_size
        
        # 8方向移动（包括对角线）
        self.directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),      # 四个基本方向
            (1, 1), (-1, 1), (1, -1), (-1, -1)     # 四个对角线方向
        ]
        
        # 对角线移动的代价更高
        self.move_costs = [
            self.grid_size, self.grid_size, self.grid_size, self.grid_size,  # 基本方向
            self.grid_size * 1.414, self.grid_size * 1.414,  # 对角线方向
            self.grid_size * 1.414, self.grid_size * 1.414
        ]
    
    def world_to_grid(self, x, y):
        """世界坐标转换为网格坐标"""
        grid_x = round(x / self.grid_size)
        grid_y = round(y / self.grid_size)
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x, grid_y):
        """网格坐标转换为世界坐标"""
        x = grid_x * self.grid_size
        y = grid_y * self.grid_size
        return x, y
    
    def heuristic(self, node, goal):
        """启发式函数（欧几里德距离）"""
        dx = abs(node.x - goal.x)
        dy = abs(node.y - goal.y)
        return math.sqrt(dx * dx + dy * dy) * self.grid_size
    
    def is_valid_position(self, grid_x, grid_y):
        """检查网格位置是否有效（不在障碍物内）"""
        # 转换为世界坐标
        world_x, world_y = self.grid_to_world(grid_x, grid_y)
        
        # 检查是否在地图边界内
        if not self.map.is_in_bounds(world_x, world_y):
            return False
        
        # 检查是否在居民区内
        zone_info = self.map.get_zone_info(world_x, world_y)
        if zone_info['type'] == ZoneType.RESIDENTIAL:
            return False
        
        return True
    
    def get_neighbors(self, node):
        """获取节点的有效邻居"""
        neighbors = []
        
        for i, (dx, dy) in enumerate(self.directions):
            new_grid_x = node.x + dx
            new_grid_y = node.y + dy
            
            # 检查新位置是否有效
            if self.is_valid_position(new_grid_x, new_grid_y):
                move_cost = self.move_costs[i]
                new_g_cost = node.g_cost + move_cost
                neighbor = Node(new_grid_x, new_grid_y, new_g_cost, 0, node)
                neighbors.append(neighbor)
        
        return neighbors
    
    def reconstruct_path(self, goal_node):
        """重构路径"""
        path = []
        current = goal_node
        
        while current is not None:
            world_x, world_y = self.grid_to_world(current.x, current.y)
            path.append((world_x, world_y))
            current = current.parent
        
        return list(reversed(path))
    
    def smooth_path(self, path):
        """路径平滑处理，去除不必要的中间点"""
        if len(path) < 3:
            return path
        
        smoothed_path = [path[0]]  # 起点
        
        i = 0
        while i < len(path) - 1:
            # 尝试从当前点直接连接到更远的点
            j = len(path) - 1
            found_direct_connection = False
            
            while j > i + 1:
                if self.is_line_clear(path[i], path[j]):
                    smoothed_path.append(path[j])
                    i = j
                    found_direct_connection = True
                    break
                j -= 1
            
            if not found_direct_connection:
                i += 1
                if i < len(path):
                    smoothed_path.append(path[i])
        
        return smoothed_path
    
    def is_line_clear(self, start, end):
        """检查两点之间的直线是否畅通（不穿过障碍物）"""
        x1, y1 = start
        x2, y2 = end
        
        # 计算线段长度
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        # 沿线段采样检查
        num_samples = max(10, int(distance / self.grid_size))
        
        for i in range(num_samples + 1):
            t = i / num_samples
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            
            zone_info = self.map.get_zone_info(x, y)
            if zone_info['type'] == ZoneType.RESIDENTIAL or not self.map.is_in_bounds(x, y):
                return False
        
        return True
    
    def plan_path(self, start_pos, goal_pos):
        """
        使用A*算法规划路径
        
        Args:
            start_pos: 起点坐标 (x, y)
            goal_pos: 终点坐标 (x, y)
            
        Returns:
            路径点列表 [(x1, y1), (x2, y2), ...] 或 None（如果无法找到路径）
        """
        print(f"开始A*路径规划:")
        print(f"起点: ({start_pos[0]}, {start_pos[1]})")
        print(f"终点: ({goal_pos[0]}, {goal_pos[1]})")
        print(f"网格大小: {self.grid_size}m")
        
        # 转换为网格坐标
        start_grid = self.world_to_grid(start_pos[0], start_pos[1])
        goal_grid = self.world_to_grid(goal_pos[0], goal_pos[1])
        
        # 检查起点和终点是否有效
        if not self.is_valid_position(start_grid[0], start_grid[1]):
            print(f"❌ 起点 {start_pos} 位置无效（可能在障碍物内）")
            return None
        
        if not self.is_valid_position(goal_grid[0], goal_grid[1]):
            print(f"❌ 终点 {goal_pos} 位置无效（可能在障碍物内）")
            return None
        
        # 初始化A*算法
        start_node = Node(start_grid[0], start_grid[1])
        goal_node = Node(goal_grid[0], goal_grid[1])
        
        open_list = []
        closed_set = set()
        
        # 计算起点的启发式代价
        start_node.h_cost = self.heuristic(start_node, goal_node)
        start_node.f_cost = start_node.g_cost + start_node.h_cost
        
        heapq.heappush(open_list, start_node)
        nodes_visited = 0
        
        while open_list:
            # 获取f值最小的节点
            current_node = heapq.heappop(open_list)
            nodes_visited += 1
            
            # 将当前节点加入已访问集合
            closed_set.add((current_node.x, current_node.y))
            
            # 检查是否到达目标
            if current_node.x == goal_node.x and current_node.y == goal_node.y:
                print(f"✅ 路径规划成功！访问了 {nodes_visited} 个节点")
                path = self.reconstruct_path(current_node)
                
                # 路径平滑处理
                smoothed_path = self.smooth_path(path)
                print(f"原始路径点数: {len(path)}, 平滑后: {len(smoothed_path)}")
                
                return smoothed_path
            
            # 探索邻居节点
            neighbors = self.get_neighbors(current_node)
            
            for neighbor in neighbors:
                # 跳过已访问的节点
                if (neighbor.x, neighbor.y) in closed_set:
                    continue
                
                # 计算启发式代价
                neighbor.h_cost = self.heuristic(neighbor, goal_node)
                neighbor.f_cost = neighbor.g_cost + neighbor.h_cost
                
                # 检查是否需要更新或添加到开放列表
                existing_node = None
                for node in open_list:
                    if node.x == neighbor.x and node.y == neighbor.y:
                        existing_node = node
                        break
                
                if existing_node is None:
                    # 新节点，加入开放列表
                    heapq.heappush(open_list, neighbor)
                elif neighbor.g_cost < existing_node.g_cost:
                    # 找到更好的路径，更新节点
                    existing_node.g_cost = neighbor.g_cost
                    existing_node.f_cost = neighbor.f_cost
                    existing_node.parent = neighbor.parent
            
            # 防止无限循环
            if nodes_visited > 100000:
                print(f"⚠️ 路径搜索节点数超限，停止搜索")
                break
        
        print(f"❌ 无法找到路径！访问了 {nodes_visited} 个节点")
        return None
    
    def visualize_path(self, start_pos, goal_pos, path=None, save_file=None):
        """
        可视化路径规划结果
        
        Args:
            start_pos: 起点坐标
            goal_pos: 终点坐标
            path: 规划的路径
            save_file: 保存图片的文件名
        """
        fig, ax = plt.subplots(1, 1, figsize=(12, 8))
        
        # 绘制地图区域
        self.map.draw_map(ax)
        
        # 绘制起点和终点
        ax.plot(start_pos[0], start_pos[1], 'go', markersize=10, label='起点')
        ax.plot(goal_pos[0], goal_pos[1], 'ro', markersize=10, label='终点')
        
        # 绘制路径
        if path:
            path_x = [p[0] for p in path]
            path_y = [p[1] for p in path]
            ax.plot(path_x, path_y, 'b-', linewidth=3, label=f'A*路径 ({len(path)}点)')
            
            # 在路径点上标记序号
            for i, (x, y) in enumerate(path):
                if i % 3 == 0 or i == len(path) - 1:  # 每3个点标记一次
                    ax.text(x, y + 15, str(i), fontsize=8, ha='center', 
                           bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
            
            # 计算路径长度
            total_distance = 0
            for i in range(len(path) - 1):
                dx = path[i+1][0] - path[i][0]
                dy = path[i+1][1] - path[i][1]
                total_distance += math.sqrt(dx*dx + dy*dy)
            
            ax.set_title(f'VTOL A*路径规划 (网格:{self.grid_size}m, 长度:{total_distance:.1f}m)')
        else:
            ax.plot([start_pos[0], goal_pos[0]], [start_pos[1], goal_pos[1]], 
                   'r--', alpha=0.5, label='直线距离')
            ax.set_title('VTOL A*路径规划 - 无可行路径')
        
        ax.set_xlabel('X坐标 (米)')
        ax.set_ylabel('Y坐标 (米)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # 保存图片
        if save_file:
            plt.savefig(save_file, dpi=300, bbox_inches='tight')
            print(f"路径可视化已保存到: {save_file}")
        
        plt.show()
    
    def compare_with_direct_path(self, start_pos, goal_pos):
        """比较A*路径与直线路径"""
        print("\n🔍 路径比较分析:")
        print("="*50)
        
        # 直线距离
        dx = goal_pos[0] - start_pos[0]
        dy = goal_pos[1] - start_pos[1]
        direct_distance = math.sqrt(dx*dx + dy*dy)
        
        # 检查直线路径是否可行
        direct_feasible = self.is_line_clear(start_pos, goal_pos)
        
        print(f"直线距离: {direct_distance:.1f}m")
        print(f"直线路径可行性: {'✅ 可行' if direct_feasible else '❌ 不可行（穿越障碍物）'}")
        
        # A*路径
        astar_path = self.plan_path(start_pos, goal_pos)
        
        if astar_path:
            astar_distance = 0
            for i in range(len(astar_path) - 1):
                dx = astar_path[i+1][0] - astar_path[i][0]
                dy = astar_path[i+1][1] - astar_path[i][1]
                astar_distance += math.sqrt(dx*dx + dy*dy)
            
            print(f"A*路径距离: {astar_distance:.1f}m")
            print(f"路径增长率: {(astar_distance/direct_distance-1)*100:.1f}%")
            print(f"A*路径点数: {len(astar_path)}")
            
            return astar_path, astar_distance, direct_distance
        else:
            print("A*路径: 无可行路径")
            return None, None, direct_distance


def test_astar_planning():
    """测试A*路径规划功能"""
    print("VTOL A*路径规划测试")
    print("="*60)
    
    # 创建路径规划器
    planner = VTOLAstarPlanner(grid_size=20)  # 20米网格
    
    # 显示地图信息
    planner.map.print_map_summary()
    
    # 测试案例
    test_cases = [
        {
            'name': '基本测试：旋翼区到北侧目标',
            'start': (0, 0),
            'goal': (1600, 200),
            'description': '从旋翼区中心飞往北侧目标点，需要绕过居民区'
        },
        {
            'name': '挑战测试：旋翼区到南侧目标',
            'start': (0, 0),
            'goal': (1600, -200),
            'description': '从旋翼区中心飞往南侧目标点，需要绕过居民区'
        },
        {
            'name': '穿越测试：横穿居民区',
            'start': (1000, 0),
            'goal': (1400, 0),
            'description': '尝试穿越居民区中心，测试避障能力'
        },
        {
            'name': '边界测试：沿地图边缘',
            'start': (100, 900),
            'goal': (1900, 900),
            'description': '沿地图北边界飞行'
        }
    ]
    
    for i, test_case in enumerate(test_cases, 1):
        print(f"\n🧪 测试 {i}: {test_case['name']}")
        print(f"描述: {test_case['description']}")
        print("-" * 50)
        
        start_pos = test_case['start']
        goal_pos = test_case['goal']
        
        # 执行路径规划和比较
        path, astar_dist, direct_dist = planner.compare_with_direct_path(start_pos, goal_pos)
        
        # 可视化结果
        save_filename = f"astar_test_{i}_{test_case['name'].split('：')[0]}.png"
        planner.visualize_path(start_pos, goal_pos, path, save_filename)


def main():
    """主函数：默认测试案例"""
    print("VTOL A*路径规划主程序")
    print("="*60)
    
    # 创建路径规划器
    planner = VTOLAstarPlanner(grid_size=15)  # 15米网格，平衡精度和性能
    
    # 默认测试：从旋翼区到北侧目标点
    start_pos = (0, 0)      # 旋翼区中心
    goal_pos = (1600, 200)  # 北侧目标点
    
    print(f"默认测试案例:")
    print(f"起点: {start_pos} (旋翼区中心)")
    print(f"终点: {goal_pos} (北侧目标点)")
    
    # 显示地图信息
    planner.map.print_map_summary()
    
    # 执行路径规划
    path, astar_dist, direct_dist = planner.compare_with_direct_path(start_pos, goal_pos)
    
    # 可视化结果
    planner.visualize_path(start_pos, goal_pos, path, "vtol_astar_default.png")
    
    print(f"\n🎯 规划总结:")
    if path:
        print(f"✅ 成功规划出安全路径")
        print(f"✅ 路径避开了所有居民区")
        print(f"✅ 路径可视化已生成")
    else:
        print(f"❌ 无法找到安全路径")
    
    print(f"\n💡 提示:")
    print(f"- 调整grid_size参数可以平衡路径精度和计算效率")
    print(f"- 使用test_astar_planning()函数可以运行更多测试案例")
    print(f"- 路径会自动避开居民区和地图边界")


if __name__ == "__main__":
    # 可以选择运行默认测试或完整测试套件
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == '--test':
        test_astar_planning()
    else:
        main()