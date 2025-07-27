#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
测试精准定位超时时间修改
验证所有精准定位阶段的超时时间已改为5秒
"""

import os
import re

def test_precision_timeout_changes():
    """测试精准定位超时时间修改"""
    print("🧪 测试精准定位超时时间修改")
    print("=" * 50)
    
    vtol_demo_path = '/home/yzy/comsen_challengecup/workspace/vtol_control/vtol_demo.py'
    
    if not os.path.exists(vtol_demo_path):
        print("❌ vtol_demo.py文件不存在")
        return False
    
    with open(vtol_demo_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    print("📝 检查精准定位超时设置:")
    
    # 查找approach_stages定义
    approach_stages_pattern = r'approach_stages\s*=\s*\[(.*?)\]'
    match = re.search(approach_stages_pattern, content, re.DOTALL)
    
    if not match:
        print("❌ 未找到approach_stages定义")
        return False
    
    stages_content = match.group(1)
    print(f"📋 找到approach_stages定义:")
    
    # 检查每个阶段的超时时间
    expected_timeouts = ["5.0", "5.0", "5.0"]
    timeout_pattern = r'\(\s*[^,]+,\s*(\d+\.?\d*),\s*"([^"]+)"\s*\)'
    timeout_matches = re.findall(timeout_pattern, stages_content)
    
    if len(timeout_matches) != 3:
        print(f"❌ 预期3个阶段，实际找到{len(timeout_matches)}个")
        return False
    
    all_correct = True
    for i, (timeout, stage_name) in enumerate(timeout_matches):
        expected_timeout = expected_timeouts[i]
        if timeout == expected_timeout:
            print(f"✅ {stage_name}: {timeout}秒 (正确)")
        else:
            print(f"❌ {stage_name}: {timeout}秒 (预期: {expected_timeout}秒)")
            all_correct = False
    
    # 显示完整的approach_stages定义
    print(f"\n📋 完整的approach_stages定义:")
    for line in stages_content.strip().split('\n'):
        line = line.strip()
        if line:
            print(f"   {line}")
    
    if all_correct:
        print(f"\n🎯 精准定位超时时间修改完成！")
        print(f"   所有阶段超时时间已设置为5秒：")
        print(f"   - 远距离接近: 5秒")
        print(f"   - 中距离接近: 5秒")
        print(f"   - 精确定位: 5秒")
        print(f"\n⚡ 这将大大加快精准定位的响应速度。")
        return True
    else:
        print(f"\n❌ 精准定位超时时间修改未完成，请检查。")
        return False

def show_timeout_summary():
    """显示所有超时设置总结"""
    print("\n📊 当前所有超时设置总结:")
    
    timeout_settings = [
        ("ROS连接超时", "10秒"),
        ("默认位置到达超时", "60秒"),
        ("航点等待超时", "30-90秒 (动态)"),
        ("精准定位 - 远距离接近", "5秒 ⚡ 已修改"),
        ("精准定位 - 中距离接近", "5秒 ⚡ 已修改"),
        ("精准定位 - 精确定位", "5秒 ⚡ 已修改"),
        ("模式切换完成", "3秒"),
        ("返航响应时间", "3秒"),
    ]
    
    for setting_name, timeout_value in timeout_settings:
        if "已修改" in timeout_value:
            print(f"   🔥 {setting_name:25} : {timeout_value}")
        else:
            print(f"   📄 {setting_name:25} : {timeout_value}")

if __name__ == "__main__":
    print("⚡ 精准定位超时时间修改测试")
    print("=" * 60)
    
    success = test_precision_timeout_changes()
    
    if success:
        show_timeout_summary()
        print("\n" + "=" * 60)
        print("🎉 精准定位超时时间修改成功！")
        print("💡 现在精准定位每个阶段只等待5秒，响应更快速。")
        print("📝 建议测试飞行以验证5秒超时是否足够。")
        exit(0)
    else:
        print("\n" + "=" * 60)
        print("❌ 测试失败，请检查修改。")
        exit(1)
