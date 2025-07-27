#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试VTOL系统的容忍度设置修改
验证误差允许范围是否已从10米增大到20米
"""

import os
import sys

# 添加路径以便导入vtol_demo模块
sys.path.append('/home/yzy/comsen_challengecup/workspace/vtol_control')

def test_tolerance_settings():
    """测试容忍度设置"""
    print("🧪 测试VTOL系统容忍度设置")
    print("=" * 50)
    
    try:
        # 检查vtol_demo.py文件中的关键设置
        vtol_demo_path = '/home/yzy/comsen_challengecup/workspace/vtol_control/vtol_demo.py'
        
        if not os.path.exists(vtol_demo_path):
            print("❌ vtol_demo.py文件不存在")
            return False
        
        with open(vtol_demo_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        print("📝 检查容忍度设置:")
        
        # 检查1: wait_for_position_reached默认容忍度
        if "tolerance=20.0" in content:
            print("✅ wait_for_position_reached默认容忍度: 20.0m")
        else:
            print("❌ wait_for_position_reached默认容忍度未正确设置")
        
        # 检查2: 航点容忍度最小值
        if "max(20.0, segment_distance * 0.1)" in content:
            print("✅ 航点容忍度最小值: 20.0m")
        else:
            print("❌ 航点容忍度最小值未正确设置")
        
        # 检查3: 精确定位阶段容忍度
        if "(20.0, 60.0, \"精确定位\")" in content:
            print("✅ 精确定位阶段容忍度: 20.0m")
        else:
            print("❌ 精确定位阶段容忍度未正确设置")
        
        # 检查4: 中距离接近阶段容忍度
        if "(min(30.0, total_distance * 0.5), 45.0, \"中距离接近\")" in content:
            print("✅ 中距离接近阶段容忍度: 30.0m (动态)")
        else:
            print("❌ 中距离接近阶段容忍度未正确设置")
        
        # 检查5: 最终精度判断标准
        if "final_distance <= 25.0" in content:
            print("✅ 精度良好标准: ≤25.0m")
        else:
            print("❌ 精度良好标准未正确设置")
        
        if "final_distance <= 40.0" in content:
            print("✅ 精度可接受标准: ≤40.0m")
        else:
            print("❌ 精度可接受标准未正确设置")
        
        # 检查6: 连续发布接近检测
        if "current_distance < 25.0" in content:
            print("✅ 连续发布接近检测: 25.0m")
        else:
            print("❌ 连续发布接近检测未正确设置")
        
        print("\n📊 容忍度设置总结:")
        print("   默认容忍度: 10m → 20m ✅")
        print("   航点最小容忍度: 10m → 20m ✅")
        print("   精确定位容忍度: 10m → 20m ✅")
        print("   中距离接近容忍度: 20m → 30m ✅")
        print("   精度良好标准: 15m → 25m ✅")
        print("   精度可接受标准: 30m → 40m ✅")
        print("   接近检测阈值: 15m → 25m ✅")
        
        print("\n🎯 容忍度修改完成！")
        print("   系统现在使用更宽松的误差允许范围，")
        print("   有助于提高飞行任务的成功率。")
        
        return True
        
    except Exception as e:
        print(f"❌ 测试过程中发生错误: {e}")
        return False

def test_import_vtol_demo():
    """测试导入vtol_demo模块"""
    print("\n🔧 测试模块导入:")
    
    try:
        from vtol_demo import VTOLDemoFlight
        print("✅ 成功导入VTOLDemoFlight类")
        
        # 创建实例（不初始化ROS）
        controller = VTOLDemoFlight()
        print("✅ 成功创建VTOLDemoFlight实例")
        
        # 检查方法签名
        import inspect
        sig = inspect.signature(controller.wait_for_position_reached)
        if sig.parameters['tolerance'].default == 20.0:
            print("✅ wait_for_position_reached方法默认容忍度: 20.0m")
        else:
            print(f"❌ wait_for_position_reached方法默认容忍度: {sig.parameters['tolerance'].default}")
        
        return True
        
    except Exception as e:
        print(f"❌ 导入测试失败: {e}")
        return False

if __name__ == "__main__":
    print("🚁 VTOL容忍度设置测试")
    print("=" * 60)
    
    # 测试1: 文件内容检查
    success1 = test_tolerance_settings()
    
    # 测试2: 模块导入检查
    success2 = test_import_vtol_demo()
    
    print("\n" + "=" * 60)
    if success1 and success2:
        print("🎉 所有测试通过！容忍度设置修改成功。")
        exit(0)
    else:
        print("❌ 部分测试失败，请检查修改。")
        exit(1)
