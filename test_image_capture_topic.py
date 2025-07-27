#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试图像采集脚本的话题修改
验证所有引用都已从iris_0改为standard_vtol_0
"""

import os
import re

def test_topic_changes():
    """测试话题修改"""
    print("🧪 测试图像采集脚本话题修改")
    print("=" * 50)
    
    image_capture_path = '/home/yzy/comsen_challengecup/workspace/vtol_control/image_capture.py'
    
    if not os.path.exists(image_capture_path):
        print("❌ image_capture.py文件不存在")
        return False
    
    with open(image_capture_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    print("📝 检查话题引用:")
    
    # 检查1: 确认没有iris_0引用
    iris_count = len(re.findall(r'iris_0', content))
    if iris_count == 0:
        print("✅ 无iris_0残留引用")
    else:
        print(f"❌ 发现{iris_count}个iris_0引用")
        return False
    
    # 检查2: 确认standard_vtol_0引用
    standard_vtol_count = len(re.findall(r'standard_vtol_0', content))
    if standard_vtol_count >= 4:
        print(f"✅ 发现{standard_vtol_count}个standard_vtol_0引用")
    else:
        print(f"❌ standard_vtol_0引用数量不足: {standard_vtol_count}")
        return False
    
    # 检查3: 具体的关键位置
    key_checks = [
        ('文档字符串', '/standard_vtol_0/camera/image_raw话题获取图像'),
        ('功能描述', '订阅/standard_vtol_0/camera/image_raw话题'),
        ('订阅者创建', "rospy.Subscriber('/standard_vtol_0/camera/image_raw', Image"),
        ('打印信息', "Topic: /standard_vtol_0/camera/image_raw"),
        ('等待消息', "rospy.wait_for_message('/standard_vtol_0/camera/image_raw', Image")
    ]
    
    for check_name, check_pattern in key_checks:
        if check_pattern in content:
            print(f"✅ {check_name}: 正确")
        else:
            print(f"❌ {check_name}: 未找到")
            return False
    
    print("\n📊 修改总结:")
    print("   ✅ 文档字符串已更新")
    print("   ✅ 功能描述已更新")
    print("   ✅ ROS订阅者话题已更新")
    print("   ✅ 打印信息已更新")
    print("   ✅ 等待消息话题已更新")
    print("   ✅ 所有iris_0引用已清除")
    
    print(f"\n🎯 话题修改完成！")
    print(f"   图像采集脚本现在订阅: /standard_vtol_0/camera/image_raw")
    print(f"   这与VTOL无人机的相机话题保持一致。")
    
    return True

def show_modified_lines():
    """显示修改的关键行"""
    print("\n📋 关键修改内容:")
    
    image_capture_path = '/home/yzy/comsen_challengecup/workspace/vtol_control/image_capture.py'
    
    with open(image_capture_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    # 查找包含standard_vtol_0的行
    for i, line in enumerate(lines, 1):
        if 'standard_vtol_0' in line:
            print(f"   第{i:3d}行: {line.strip()}")

if __name__ == "__main__":
    print("🛸 图像采集话题修改测试")
    print("=" * 60)
    
    success = test_topic_changes()
    
    if success:
        show_modified_lines()
        print("\n" + "=" * 60)
        print("🎉 所有测试通过！话题修改成功。")
        print("💡 现在可以使用以下命令测试图像采集:")
        print("   cd /home/yzy/comsen_challengecup/workspace/vtol_control")
        print("   python3 image_capture.py")
        exit(0)
    else:
        print("\n" + "=" * 60)
        print("❌ 测试失败，请检查修改。")
        exit(1)
