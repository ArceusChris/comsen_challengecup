#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
VTOL无人机智能路径规划测试脚本
测试A*算法和高级路径规划功能
"""

import os
import sys

# 添加工作目录到Python路径
workspace_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(workspace_dir)

from workspace.vtol_control.vtol_map import VTOLMap
import matplotlib.pyplot as plt


def test_basic_pathfinding():
    """测试基础路径规划功能"""
    print("=" * 60)
    print("测试1: 基础路径规划功能")
    print("=" * 60)
    
    vtol_map = VTOLMap()
    
    # 测试场景1: 简单直线路径（无障碍）
    print("\n场景1: 简单直线路径")
    start = (100, 100)
    end = (500, 500)
    
    path = vtol_map.plan_safe_path(start[0], start[1], end[0], end[1])
    vtol_map.print_path_report(path, start)
    
    # 测试场景2: 需要绕过居民区的路径
    print("\n场景2: 绕过downtown居民区")
    start = (800, 0)
    end = (1600, 0)
    
    path = vtol_map.plan_safe_path(start[0], start[1], end[0], end[1])
    vtol_map.print_path_report(path, start)
    
    return vtol_map, path, start, end


def test_complex_scenarios():
    """测试复杂场景"""
    print("\n" + "=" * 60)
    print("测试2: 复杂路径规划场景")
    print("=" * 60)
    
    vtol_map = VTOLMap()
    
    # 场景1: 从旋翼区到两个居民区之间
    print("\n场景1: 旋翼区到居民区之间")
    start = (0, 0)
    end = (1500, 0)
    
    path1 = vtol_map.plan_safe_path(start[0], start[1], end[0], end[1])
    vtol_map.print_path_report(path1, start)
    
    # 场景2: 穿越两个居民区
    print("\n场景2: 穿越两个居民区")
    start = (900, 200)
    end = (1900, -200)
    
    path2 = vtol_map.plan_safe_path(start[0], start[1], end[0], end[1])
    vtol_map.print_path_report(path2, start)
    
    # 场景3: 从地图一端到另一端
    print("\n场景3: 地图一端到另一端")
    start = (100, -800)
    end = (1900, 800)
    
    path3 = vtol_map.plan_safe_path(start[0], start[1], end[0], end[1])
    vtol_map.print_path_report(path3, start)
    
    return vtol_map, [(path1, (0, 0), (1500, 0)),
                      (path2, (900, 200), (1900, -200)),
                      (path3, (100, -800), (1900, 800))]


def test_yaml_targets():
    """测试YAML目标点的路径规划"""
    print("\n" + "=" * 60)
    print("测试3: YAML目标点路径规划")
    print("=" * 60)
    
    vtol_map = VTOLMap()
    
    # 从YAML文件获取目标点
    yaml_file = os.path.join(workspace_dir, "workspace/vtol_control/vtol_target.yaml")
    
    if not os.path.exists(yaml_file):
        print(f"YAML文件不存在: {yaml_file}")
        return None
    
    try:
        import yaml
        with open(yaml_file, 'r', encoding='utf-8') as f:
            targets_data = yaml.safe_load(f)
        
        targets = targets_data.get('targets', [])
        
        if not targets:
            print("YAML文件中没有找到目标点")
            return None
        
        print(f"从YAML文件加载了 {len(targets)} 个目标点")
        
        # 规划从旋翼区到各个目标点的路径
        start_point = (0, 0)  # 旋翼区起点
        all_paths = []
        
        for i, target in enumerate(targets):
            pos = target['position']
            end_point = (pos[0], pos[1])
            
            print(f"\n规划到目标点 {target['name']} 的路径:")
            path = vtol_map.plan_safe_path(start_point[0], start_point[1], end_point[0], end_point[1])
            vtol_map.print_path_report(path, start_point)
            
            all_paths.append((path, start_point, end_point, target['name']))
            
            # 下一段路径的起点是当前目标点
            start_point = end_point
        
        return vtol_map, all_paths
        
    except ImportError:
        print("需要安装pyyaml: pip install pyyaml")
        return None
    except Exception as e:
        print(f"读取YAML文件出错: {e}")
        return None


def visualize_all_paths():
    """可视化所有测试路径"""
    print("\n" + "=" * 60)
    print("路径可视化")
    print("=" * 60)
    
    # 测试复杂场景
    vtol_map, paths = test_complex_scenarios()
    
    # 为每个路径创建可视化
    for i, (path, start, end) in enumerate(paths):
        if path:
            print(f"\n可视化路径 {i+1}")
            save_path = f"/tmp/vtol_path_{i+1}.png"
            vtol_map.visualize_path(path, start, end, save_path)


def performance_test():
    """性能测试"""
    print("\n" + "=" * 60)
    print("测试4: 路径规划性能测试")
    print("=" * 60)
    
    import time
    
    vtol_map = VTOLMap()
    
    # 测试不同距离的路径规划性能
    test_cases = [
        ((0, 0), (500, 500), "短距离"),
        ((0, 0), (1000, 1000), "中距离"),
        ((0, 0), (1900, 900), "长距离"),
        ((100, -900), (1900, 900), "最长距离")
    ]
    
    for start, end, description in test_cases:
        print(f"\n{description}路径规划: {start} -> {end}")
        
        start_time = time.time()
        path = vtol_map.plan_safe_path(start[0], start[1], end[0], end[1])
        end_time = time.time()
        
        planning_time = end_time - start_time
        print(f"规划时间: {planning_time:.3f} 秒")
        
        if path:
            metrics = vtol_map.analyze_path_metrics(path, start)
            print(f"路径长度: {metrics.get('total_distance', 0):.1f} 米")
            print(f"绕行系数: {metrics.get('detour_ratio', 1):.2f}")
            print(f"航点数量: {len(path)}")


def main():
    """主测试函数"""
    print("VTOL无人机智能路径规划测试")
    print("=" * 60)
    
    # 创建地图并显示概览
    vtol_map = VTOLMap()
    vtol_map.print_map_summary()
    
    # 运行各项测试
    try:
        # 基础功能测试
        test_basic_pathfinding()
        
        # 复杂场景测试
        test_complex_scenarios()
        
        # YAML目标点测试
        yaml_result = test_yaml_targets()
        
        # 性能测试
        performance_test()
        
        # 可视化（可选）
        try:
            import matplotlib
            matplotlib.use('Agg')  # 使用非交互式后端
            print("\n准备生成路径可视化图...")
            visualize_all_paths()
        except ImportError:
            print("\n跳过可视化（需要matplotlib）")
        except Exception as e:
            print(f"\n可视化出错: {e}")
        
        print("\n" + "=" * 60)
        print("所有测试完成!")
        print("=" * 60)
        
    except Exception as e:
        print(f"\n测试过程中出现错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
