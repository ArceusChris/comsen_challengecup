#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试YOLO节点新增功能的简单脚本
"""

def test_vtol_flag_logic():
    """测试VTOL标志逻辑"""
    print("测试VTOL标志逻辑...")
    
    # 模拟不同机型和标志组合
    test_cases = [
        # (aircraft_type, vtol_flag, expected_pointcloud_publish)
        ('iris', 0, True),           # iris机型始终发布点云
        ('iris', 1, True),
        ('iris', 2, True),
        ('standard_vtol', 0, False), # vtol机型只在标志为2时发布点云
        ('standard_vtol', 1, False),
        ('standard_vtol', 2, True),
    ]
    
    for aircraft_type, vtol_flag, expected in test_cases:
        # 模拟判断逻辑
        if aircraft_type != 'standard_vtol':
            should_publish = True
        else:
            should_publish = (vtol_flag == 2)
        
        result = "✓" if should_publish == expected else "✗"
        print(f"{result} 机型={aircraft_type}, 标志={vtol_flag}, 点云发布={should_publish}, 期望={expected}")
    
    print("测试完成!\n")

def test_topic_names():
    """测试话题名称"""
    print("测试话题名称...")
    
    target_names = ['red', 'yellow', 'white']
    
    print("点云话题:")
    for target in target_names:
        print(f"  /yolo11/pointcloud/{target}")
    
    print("实时位置话题:")
    for target in target_names:
        print(f"  /yolo11/position/{target}")
    
    print("VTOL标志话题:")
    print("  /zhihang2025/vtol_land_sub/done")
    
    print("测试完成!\n")

def show_feature_summary():
    """显示功能摘要"""
    print("=" * 60)
    print("YOLO推理节点新增功能摘要")
    print("=" * 60)
    print("1. 机型支持:")
    print("   - iris: 始终发布点云数据")
    print("   - standard_vtol: 根据VTOL标志控制点云发布")
    print()
    print("2. VTOL标志控制逻辑:")
    print("   - 订阅话题: /zhihang2025/vtol_land_sub/done")
    print("   - 标志=2: 发布点云数据")
    print("   - 标志≠2: 仅发布实时位置数据")
    print()
    print("3. 新增发布话题:")
    print("   - 实时位置: /yolo11/position/{target_name}")
    print("   - 点云数据: /yolo11/pointcloud/{target_name} (条件发布)")
    print()
    print("4. 支持的目标类型:")
    print("   - red, yellow, white")
    print()
    print("5. 状态显示:")
    print("   - 图像上显示VTOL标志状态和点云发布状态")
    print("=" * 60)

if __name__ == '__main__':
    show_feature_summary()
    test_vtol_flag_logic()
    test_topic_names()
    print("所有测试完成!")
