#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
简单的模块导入测试
'''

def test_imports():
    """测试模块导入"""
    print("测试模块导入...")
    
    try:
        # 测试基础模块
        from vtol_map import VTOLMap, ZoneType
        print("✅ vtol_map 导入成功")
        
        from vtol_Astar import VTOLAstarPlanner
        print("✅ vtol_Astar 导入成功")
        
        from vtol_ros import VTOLROSCommunicator
        print("✅ vtol_ros 导入成功")
        
        from vtol_fly import VTOLFlightController
        print("✅ vtol_fly 导入成功")
        
        from vtol_demo import VTOLDemoFlight
        print("✅ vtol_demo 导入成功")
        
        return True
        
    except Exception as e:
        print(f"❌ 导入失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_object_creation():
    """测试对象创建"""
    print("\n测试对象创建...")
    
    try:
        # 初始化ROS节点
        import rospy
        if not rospy.core.is_initialized():
            rospy.init_node('test_creation', anonymous=True)
        
        from vtol_map import VTOLMap
        map_obj = VTOLMap()
        print("✅ VTOLMap 创建成功")
        
        from vtol_Astar import VTOLAstarPlanner
        planner = VTOLAstarPlanner()
        print("✅ VTOLAstarPlanner 创建成功")
        
        from vtol_ros import VTOLROSCommunicator
        ros_comm = VTOLROSCommunicator("standard_vtol", "0")
        print("✅ VTOLROSCommunicator 创建成功")
        
        from vtol_fly import VTOLFlightController
        controller = VTOLFlightController()
        print("✅ VTOLFlightController 创建成功")
        
        return True
        
    except Exception as e:
        print(f"❌ 对象创建失败: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("VTOL模块验证测试")
    print("=" * 40)
    
    # 测试导入
    import_ok = test_imports()
    
    # 测试对象创建
    creation_ok = test_object_creation()
    
    print("\n" + "=" * 40)
    print("测试结果:")
    print(f"模块导入: {'✅ 通过' if import_ok else '❌ 失败'}")
    print(f"对象创建: {'✅ 通过' if creation_ok else '❌ 失败'}")
    
    if import_ok and creation_ok:
        print("🎉 模块化拆分验证通过!")
    else:
        print("❌ 模块化拆分存在问题")
