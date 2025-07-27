#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
调试版本的VTOL演示脚本
逐步检查每个初始化步骤
"""

import rospy
import time
import os
import yaml
from vtol_map import VTOLMap, ZoneType
from vtol_Astar import VTOLAstarPlanner

def test_basic_imports():
    """测试基础导入"""
    print("🧪 测试基础导入...")
    try:
        print("✅ rospy 导入成功")
        print("✅ vtol_map 导入成功")
        print("✅ vtol_Astar 导入成功")
        return True
    except Exception as e:
        print(f"❌ 导入失败: {e}")
        return False

def test_file_access():
    """测试文件访问"""
    print("\n🧪 测试文件访问...")
    
    files_to_check = [
        "vtol_target.yaml",
        "vtol_map.py", 
        "vtol_Astar.py"
    ]
    
    all_exist = True
    for file_name in files_to_check:
        if os.path.exists(file_name):
            print(f"✅ {file_name} 存在")
        else:
            print(f"❌ {file_name} 不存在")
            all_exist = False
    
    # 测试YAML文件内容
    try:
        with open("vtol_target.yaml", 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
            print(f"✅ vtol_target.yaml 解析成功，包含 {len(data.get('targets', []))} 个目标点")
    except Exception as e:
        print(f"❌ vtol_target.yaml 解析失败: {e}")
        all_exist = False
    
    return all_exist

def test_map_initialization():
    """测试地图初始化"""
    print("\n🧪 测试地图初始化...")
    try:
        vtol_map = VTOLMap()
        print("✅ 地图对象创建成功")
        
        # 测试基本功能
        zone_info = vtol_map.get_zone_info(0, 0)
        print(f"✅ 地图查询成功: 原点位于 {zone_info['name']}")
        
        return True
    except Exception as e:
        print(f"❌ 地图初始化失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_astar_initialization():
    """测试A*初始化"""
    print("\n🧪 测试A*路径规划器初始化...")
    try:
        astar_planner = VTOLAstarPlanner(grid_size=20)
        print("✅ A*路径规划器创建成功")
        
        # 测试简单路径规划
        path = astar_planner.plan_path((0, 0), (100, 100))
        if path:
            print(f"✅ A*路径规划测试成功，路径长度: {len(path)}")
        else:
            print("⚠️ A*路径规划返回空路径")
        
        return True
    except Exception as e:
        print(f"❌ A*初始化失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_ros_environment():
    """测试ROS环境"""
    print("\n🧪 测试ROS环境...")
    
    # 检查ROS环境变量
    ros_master_uri = os.environ.get('ROS_MASTER_URI')
    ros_distro = os.environ.get('ROS_DISTRO')
    
    print(f"ROS_MASTER_URI: {ros_master_uri}")
    print(f"ROS_DISTRO: {ros_distro}")
    
    if ros_master_uri is None:
        print("❌ ROS_MASTER_URI 未设置")
        return False
    
    return True

def test_ros_node_init():
    """测试ROS节点初始化"""
    print("\n🧪 测试ROS节点初始化...")
    try:
        # 尝试初始化节点
        rospy.init_node('vtol_debug_test', anonymous=True)
        print("✅ ROS节点初始化成功")
        
        # 等待一秒确保节点完全启动
        time.sleep(1)
        
        # 检查节点状态
        if rospy.is_shutdown():
            print("❌ ROS节点已关闭")
            return False
        else:
            print("✅ ROS节点运行正常")
            return True
            
    except Exception as e:
        print(f"❌ ROS节点初始化失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """主测试函数"""
    print("🚁 VTOL演示脚本调试模式")
    print("=" * 50)
    
    test_results = []
    
    # 逐步测试各个组件
    test_results.append(("基础导入", test_basic_imports()))
    test_results.append(("文件访问", test_file_access()))
    test_results.append(("地图初始化", test_map_initialization()))
    test_results.append(("A*初始化", test_astar_initialization()))
    test_results.append(("ROS环境", test_ros_environment()))
    test_results.append(("ROS节点初始化", test_ros_node_init()))
    
    # 总结
    print("\n📊 测试结果总结:")
    print("-" * 30)
    
    all_passed = True
    for test_name, result in test_results:
        status = "✅ 通过" if result else "❌ 失败"
        print(f"{test_name:15} : {status}")
        if not result:
            all_passed = False
    
    print("-" * 30)
    
    if all_passed:
        print("🎉 所有测试通过！可以运行完整的VTOL演示脚本")
        
        # 如果测试都通过，尝试创建简单的VTOL对象
        print("\n🧪 尝试创建VTOL飞行对象...")
        try:
            from vtol_demo import VTOLDemoFlight
            print("✅ VTOLDemoFlight 类导入成功")
            print("准备创建实例...")
            
            # 注意：不初始化完整的飞行对象，避免重复初始化ROS节点
            print("（跳过对象创建以避免ROS节点重复初始化）")
            
        except Exception as e:
            print(f"❌ VTOLDemoFlight 创建失败: {e}")
            import traceback
            traceback.print_exc()
    else:
        print("❌ 部分测试失败，请检查环境配置")
    
    return all_passed

if __name__ == "__main__":
    try:
        success = main()
        exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n🛑 用户中断测试")
        exit(1)
    except Exception as e:
        print(f"\n💥 测试过程中发生意外错误: {e}")
        import traceback
        traceback.print_exc()
        exit(1)
