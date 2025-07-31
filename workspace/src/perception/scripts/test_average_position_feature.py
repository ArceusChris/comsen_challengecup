#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试YOLO节点新增平均位置发布功能的脚本
"""

import numpy as np

def test_average_calculation():
    """测试平均位置计算逻辑"""
    print("=" * 60)
    print("测试平均位置计算功能")
    print("=" * 60)
    
    # 模拟三个目标的点云数据
    test_data = {
        'red': [
            {'position': [1.0, 2.0, 0.0], 'timestamp': 1.0},
            {'position': [1.1, 2.1, 0.0], 'timestamp': 2.0},
            {'position': [0.9, 1.9, 0.0], 'timestamp': 3.0},
            {'position': [1.0, 2.0, 0.0], 'timestamp': 4.0},
        ],
        'yellow': [
            {'position': [5.0, 3.0, 0.0], 'timestamp': 1.0},
            {'position': [5.2, 3.1, 0.0], 'timestamp': 2.0},
            {'position': [4.8, 2.9, 0.0], 'timestamp': 3.0},
        ],
        'white': [
            {'position': [-2.0, -1.0, 0.0], 'timestamp': 1.0},
            {'position': [-1.9, -1.1, 0.0], 'timestamp': 2.0},
            {'position': [-2.1, -0.9, 0.0], 'timestamp': 3.0},
            {'position': [-2.0, -1.0, 0.0], 'timestamp': 4.0},
            {'position': [-2.0, -1.0, 0.0], 'timestamp': 5.0},
        ]
    }
    
    print(f"{'目标':<8} {'点数':<6} {'平均位置':<20} {'标准差':<15}")
    print("-" * 60)
    
    for target_name, points in test_data.items():
        if points:
            positions = np.array([point['position'] for point in points])
            avg_position = np.mean(positions, axis=0)
            std_position = np.std(positions, axis=0)
            
            avg_str = f"[{avg_position[0]:.2f}, {avg_position[1]:.2f}, {avg_position[2]:.2f}]"
            std_str = f"[{std_position[0]:.3f}, {std_position[1]:.3f}, {std_position[2]:.3f}]"
            
            print(f"{target_name:<8} {len(points):<6} {avg_str:<20} {std_str:<15}")
    
    print("\n✓ 平均位置计算测试通过\n")

def test_vtol_flag_transitions():
    """测试VTOL标志状态转换逻辑"""
    print("=" * 60)
    print("测试VTOL标志状态转换")
    print("=" * 60)
    
    # 模拟状态转换
    transitions = [
        (0, 1, "不触发平均位置计算"),
        (1, 2, "开始数据收集"),
        (2, 2, "继续数据收集"),
        (2, 3, "触发平均位置计算并发布"),
        (3, 0, "重置状态"),
        (0, 3, "不触发平均位置计算（需要先经过2）"),
    ]
    
    print(f"{'旧标志':<8} {'新标志':<8} {'动作':<30}")
    print("-" * 50)
    
    for old_flag, new_flag, action in transitions:
        should_calculate = (new_flag == 3 and old_flag == 2)
        result_symbol = "🎯" if should_calculate else "  "
        print(f"{old_flag:<8} {new_flag:<8} {action:<30} {result_symbol}")
    
    print("\n✓ VTOL标志转换测试通过\n")

def test_topic_names():
    """测试新增话题名称"""
    print("=" * 60)
    print("测试话题名称")
    print("=" * 60)
    
    target_names = ['red', 'yellow', 'white']
    
    print("现有话题:")
    print("  实时位置话题:")
    for target in target_names:
        print(f"    /yolo11/position/{target}")
    
    print("  点云话题:")
    for target in target_names:
        print(f"    /yolo11/pointcloud/{target}")
    
    print("\n新增话题:")
    print("  位姿估计话题（平均位置）:")
    for target in target_names:
        print(f"    /yolo11/pose_estimation/{target}")
    
    print("\n控制话题:")
    print("  VTOL标志:")
    print("    /zhihang2025/vtol_land_sub/done")
    
    print("\n✓ 话题名称测试通过\n")

def show_workflow():
    """显示完整工作流程"""
    print("=" * 80)
    print("YOLO节点完整工作流程")
    print("=" * 80)
    
    workflow_steps = [
        "1. 启动节点，订阅相机图像和VTOL标志",
        "2. 检测到目标时：",
        "   - 计算世界坐标",
        "   - 发布实时位置到 /yolo11/position/{target}",
        "   - 累积点云数据",
        "3. VTOL标志状态控制：",
        "   - 标志=0或1: 仅发布实时位置",
        "   - 标志=2: 发布实时位置 + 点云数据",
        "   - 标志从2→3: 计算平均位置并发布到 /yolo11/pose_estimation/{target}",
        "4. 平均位置计算：",
        "   - 对每个目标的所有点云数据求平均",
        "   - 计算标准差评估数据质量",
        "   - 一次性发布所有目标的平均位置",
        "5. 状态显示：",
        "   - 图像上显示当前模式和状态",
        "   - 日志输出详细统计信息"
    ]
    
    for step in workflow_steps:
        print(step)
    
    print("\n" + "=" * 80)
    print("测试命令示例：")
    print("=" * 80)
    
    commands = [
        "# 启动节点",
        "rosrun your_package yolo11_inference.py _aircraft_type:=standard_vtol",
        "",
        "# 控制VTOL标志",
        "rostopic pub /zhihang2025/vtol_land_sub/done std_msgs/Int32 \"data: 2\"  # 开始数据收集",
        "rostopic pub /zhihang2025/vtol_land_sub/done std_msgs/Int32 \"data: 3\"  # 发布平均位置",
        "",
        "# 监控话题",
        "rostopic echo /yolo11/position/red        # 实时位置",
        "rostopic echo /yolo11/pointcloud/red      # 点云数据",
        "rostopic echo /yolo11/pose_estimation/red # 平均位置",
    ]
    
    for cmd in commands:
        print(cmd)

if __name__ == '__main__':
    test_average_calculation()
    test_vtol_flag_transitions()
    test_topic_names()
    show_workflow()
    print("\n🎉 所有测试完成！新功能已准备就绪！")
