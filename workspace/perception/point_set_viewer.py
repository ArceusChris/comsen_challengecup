#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
点集数据查看器
用于查看和管理YOLO推理节点收集的目标位置点集
"""

import rospy
import pickle
import json
import numpy as np
from datetime import datetime
import os

class PointSetViewer:
    def __init__(self):
        """初始化点集查看器"""
        self.data_file = "/root/workspace/perception/target_points_data.json"
        
    def load_data_from_node(self):
        """从运行中的YOLO节点获取数据（需要节点正在运行）"""
        try:
            # 这里可以通过ROS服务或话题来获取节点中的数据
            # 目前作为示例，我们创建一个模拟的数据结构
            rospy.loginfo("尝试从YOLO推理节点获取数据...")
            rospy.logwarn("注意：此功能需要YOLO推理节点正在运行")
            return None
        except Exception as e:
            rospy.logerr(f"从节点获取数据失败: {str(e)}")
            return None
    
    def save_data_to_file(self, data, filename=None):
        """保存点集数据到文件"""
        if filename is None:
            filename = self.data_file
        
        try:
            # 转换numpy数组为列表，便于JSON序列化
            serializable_data = {}
            for target_name, points in data.items():
                serializable_data[target_name] = []
                for point in points:
                    serializable_data[target_name].append({
                        'position': point['position'] if isinstance(point['position'], list) else point['position'].tolist(),
                        'timestamp': point['timestamp'],
                        'pixel_coords': point['pixel_coords']
                    })
            
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(serializable_data, f, indent=2, ensure_ascii=False)
            
            print(f"数据已保存到: {filename}")
            return True
            
        except Exception as e:
            print(f"保存数据失败: {str(e)}")
            return False
    
    def load_data_from_file(self, filename=None):
        """从文件加载点集数据"""
        if filename is None:
            filename = self.data_file
        
        try:
            if not os.path.exists(filename):
                print(f"文件不存在: {filename}")
                return None
            
            with open(filename, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            print(f"数据已从文件加载: {filename}")
            return data
            
        except Exception as e:
            print(f"加载数据失败: {str(e)}")
            return None
    
    def analyze_point_set(self, data):
        """分析点集数据"""
        if data is None:
            print("没有数据可分析")
            return
        
        print("\n=== 点集数据分析 ===")
        
        for target_name, points in data.items():
            if not points:
                print(f"\n{target_name}: 无数据")
                continue
            
            print(f"\n{target_name}:")
            print(f"  总点数: {len(points)}")
            
            # 提取位置数据
            positions = np.array([point['position'] for point in points])
            
            if len(positions) > 0:
                # 统计信息
                mean_pos = np.mean(positions, axis=0)
                std_pos = np.std(positions, axis=0)
                min_pos = np.min(positions, axis=0)
                max_pos = np.max(positions, axis=0)
                
                print(f"  平均位置: [{mean_pos[0]:.3f}, {mean_pos[1]:.3f}, {mean_pos[2]:.3f}]")
                print(f"  标准差: [{std_pos[0]:.3f}, {std_pos[1]:.3f}, {std_pos[2]:.3f}]")
                print(f"  范围: X[{min_pos[0]:.3f}, {max_pos[0]:.3f}], Y[{min_pos[1]:.3f}, {max_pos[1]:.3f}]")
                
                # 时间范围
                timestamps = [point['timestamp'] for point in points]
                time_range = max(timestamps) - min(timestamps)
                print(f"  时间跨度: {time_range:.2f} 秒")
                
                # 最新几个点
                if len(points) >= 3:
                    print("  最新3个位置:")
                    for i, point in enumerate(points[-3:]):
                        pos = point['position']
                        timestamp = datetime.fromtimestamp(point['timestamp'])
                        print(f"    {i+1}. [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] at {timestamp.strftime('%H:%M:%S')}")
    
    def export_to_csv(self, data, filename=None):
        """导出点集数据到CSV文件"""
        if filename is None:
            filename = "/root/workspace/perception/target_points.csv"
        
        try:
            import csv
            
            with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
                fieldnames = ['target', 'x', 'y', 'z', 'timestamp', 'pixel_x', 'pixel_y']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                
                writer.writeheader()
                
                for target_name, points in data.items():
                    for point in points:
                        writer.writerow({
                            'target': target_name,
                            'x': point['position'][0],
                            'y': point['position'][1],
                            'z': point['position'][2],
                            'timestamp': point['timestamp'],
                            'pixel_x': point['pixel_coords'][0],
                            'pixel_y': point['pixel_coords'][1]
                        })
            
            print(f"数据已导出到CSV文件: {filename}")
            return True
            
        except Exception as e:
            print(f"导出CSV失败: {str(e)}")
            return False
    
    def interactive_menu(self):
        """交互式菜单"""
        while True:
            print("\n=== 点集数据查看器 ===")
            print("1. 从文件加载数据")
            print("2. 分析点集数据")
            print("3. 导出到CSV")
            print("4. 创建示例数据")
            print("5. 退出")
            
            choice = input("\n请选择操作 (1-5): ").strip()
            
            if choice == '1':
                filename = input("输入文件名 (回车使用默认): ").strip()
                if not filename:
                    filename = None
                self.current_data = self.load_data_from_file(filename)
                
            elif choice == '2':
                if hasattr(self, 'current_data'):
                    self.analyze_point_set(self.current_data)
                else:
                    print("请先加载数据")
                    
            elif choice == '3':
                if hasattr(self, 'current_data'):
                    filename = input("输入CSV文件名 (回车使用默认): ").strip()
                    if not filename:
                        filename = None
                    self.export_to_csv(self.current_data, filename)
                else:
                    print("请先加载数据")
                    
            elif choice == '4':
                self.create_sample_data()
                
            elif choice == '5':
                print("退出程序")
                break
                
            else:
                print("无效选择，请重试")
    
    def create_sample_data(self):
        """创建示例数据用于测试"""
        import random
        import time
        
        sample_data = {
            'red': [],
            'yellow': [],
            'white': []
        }
        
        # 为每个目标生成一些示例点
        base_positions = {
            'red': [2.0, 3.0, 0.0],
            'yellow': [-1.0, 2.5, 0.0],
            'white': [0.5, -1.0, 0.0]
        }
        
        current_time = time.time()
        
        for target_name, base_pos in base_positions.items():
            for i in range(10):
                # 添加一些噪声
                noise_x = random.uniform(-0.2, 0.2)
                noise_y = random.uniform(-0.2, 0.2)
                
                position = [
                    base_pos[0] + noise_x,
                    base_pos[1] + noise_y,
                    0.0
                ]
                
                pixel_coords = [
                    random.uniform(200, 600),
                    random.uniform(150, 450)
                ]
                
                sample_data[target_name].append({
                    'position': position,
                    'timestamp': current_time + i * 0.1,
                    'pixel_coords': pixel_coords
                })
        
        # 保存示例数据
        self.save_data_to_file(sample_data)
        self.current_data = sample_data
        print("已创建并保存示例数据")

def main():
    """主函数"""
    print("点集数据查看器")
    print("用于查看和管理YOLO推理节点收集的目标位置数据")
    
    viewer = PointSetViewer()
    viewer.interactive_menu()

if __name__ == '__main__':
    main()
