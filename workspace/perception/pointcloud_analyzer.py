#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
点云数据分析工具
用于分析和处理YOLO11推理节点发布的PointCloud2数据
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
import struct
import threading
from collections import defaultdict
import time

class PointCloudAnalyzer:
    def __init__(self):
        """初始化点云分析器"""
        rospy.init_node('pointcloud_analyzer', anonymous=True)
        
        # 数据存储
        self.pointcloud_data = defaultdict(list)
        self.data_lock = threading.Lock()
        
        # 目标类型
        self.target_types = ['red', 'yellow', 'white']
        
        # 创建订阅者
        self.subscribers = {}
        for target_type in self.target_types:
            topic = f"/yolo11/pointcloud/{target_type}"
            self.subscribers[target_type] = rospy.Subscriber(
                topic, PointCloud2, 
                lambda msg, t=target_type: self.pointcloud_callback(msg, t)
            )
            rospy.loginfo(f"订阅点云话题: {topic}")
        
        # 统计信息
        self.last_analysis_time = time.time()
        self.analysis_interval = 5.0  # 5秒分析一次
        
        rospy.loginfo("点云分析器已启动")
    
    def pointcloud_callback(self, msg, target_type):
        """点云回调函数"""
        try:
            # 解析点云数据
            points = list(pc2.read_points(msg, field_names=('x', 'y', 'z', 'timestamp', 'intensity')))
            
            with self.data_lock:
                self.pointcloud_data[target_type] = {
                    'header': msg.header,
                    'points': points,
                    'last_update': time.time()
                }
            
            rospy.logdebug(f"接收到{target_type}点云数据，包含{len(points)}个点")
            
        except Exception as e:
            rospy.logerr(f"解析{target_type}点云数据失败: {str(e)}")
    
    def analyze_pointcloud_statistics(self, target_type):
        """分析单个目标的点云统计信息"""
        with self.data_lock:
            if target_type not in self.pointcloud_data:
                return None
            
            data = self.pointcloud_data[target_type]
            points = data['points']
            
            if not points:
                return {
                    'target_type': target_type,
                    'count': 0,
                    'valid': False
                }
            
            # 提取坐标
            positions = np.array([[p[0], p[1], p[2]] for p in points])
            timestamps = np.array([p[3] for p in points])
            intensities = np.array([p[4] for p in points])
            
            # 计算统计信息
            stats = {
                'target_type': target_type,
                'count': len(points),
                'valid': True,
                'positions': {
                    'mean': np.mean(positions, axis=0).tolist(),
                    'std': np.std(positions, axis=0).tolist(),
                    'min': np.min(positions, axis=0).tolist(),
                    'max': np.max(positions, axis=0).tolist(),
                    'center': np.mean(positions, axis=0).tolist(),
                    'spread': np.max(positions, axis=0) - np.min(positions, axis=0)
                },
                'timestamps': {
                    'earliest': float(np.min(timestamps)),
                    'latest': float(np.max(timestamps)),
                    'span': float(np.max(timestamps) - np.min(timestamps))
                },
                'intensity': {
                    'mean': float(np.mean(intensities)),
                    'unique_values': np.unique(intensities).tolist()
                },
                'data_age': time.time() - data['last_update']
            }
            
            return stats
    
    def get_comprehensive_analysis(self):
        """获取全面的点云分析"""
        analysis = {
            'timestamp': time.time(),
            'targets': {},
            'summary': {}
        }
        
        total_points = 0
        active_targets = 0
        
        for target_type in self.target_types:
            stats = self.analyze_pointcloud_statistics(target_type)
            if stats:
                analysis['targets'][target_type] = stats
                if stats['valid'] and stats['count'] > 0:
                    total_points += stats['count']
                    active_targets += 1
        
        # 整体摘要
        analysis['summary'] = {
            'total_points': total_points,
            'active_targets': active_targets,
            'target_types': self.target_types
        }
        
        return analysis
    
    def detect_clustering(self, target_type, eps=0.5, min_samples=3):
        """检测目标点的聚类"""
        with self.data_lock:
            if target_type not in self.pointcloud_data:
                return None
            
            points = self.pointcloud_data[target_type]['points']
            
            if len(points) < min_samples:
                return None
            
            try:
                from sklearn.cluster import DBSCAN
                
                # 提取位置坐标
                positions = np.array([[p[0], p[1], p[2]] for p in points])
                
                # 只使用X, Y坐标进行聚类（忽略Z，因为都在地面）
                clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(positions[:, :2])
                
                labels = clustering.labels_
                n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
                n_noise = list(labels).count(-1)
                
                cluster_info = {
                    'n_clusters': n_clusters,
                    'n_noise_points': n_noise,
                    'labels': labels.tolist(),
                    'cluster_centers': []
                }
                
                # 计算每个聚类的中心
                for cluster_id in set(labels):
                    if cluster_id != -1:  # 忽略噪声点
                        cluster_points = positions[labels == cluster_id]
                        center = np.mean(cluster_points, axis=0)
                        cluster_info['cluster_centers'].append({
                            'cluster_id': int(cluster_id),
                            'center': center.tolist(),
                            'size': len(cluster_points)
                        })
                
                return cluster_info
                
            except ImportError:
                rospy.logwarn("sklearn未安装，无法进行聚类分析")
                return None
            except Exception as e:
                rospy.logerr(f"聚类分析失败: {str(e)}")
                return None
    
    def export_analysis_to_file(self, filename=None):
        """导出分析结果到文件"""
        if filename is None:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"/root/workspace/perception/pointcloud_analysis_{timestamp}.txt"
        
        analysis = self.get_comprehensive_analysis()
        
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                f.write("YOLO11点云数据分析报告\n")
                f.write("=" * 50 + "\n")
                f.write(f"分析时间: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(analysis['timestamp']))}\n\n")
                
                # 总体摘要
                summary = analysis['summary']
                f.write("总体摘要:\n")
                f.write(f"  活跃目标数: {summary['active_targets']}/{len(summary['target_types'])}\n")
                f.write(f"  总点数: {summary['total_points']}\n\n")
                
                # 各目标详细信息
                for target_type, stats in analysis['targets'].items():
                    f.write(f"{target_type.upper()}目标分析:\n")
                    f.write("-" * 30 + "\n")
                    
                    if not stats['valid'] or stats['count'] == 0:
                        f.write("  无数据\n\n")
                        continue
                    
                    f.write(f"  点数: {stats['count']}\n")
                    f.write(f"  数据年龄: {stats['data_age']:.2f}秒\n")
                    
                    pos = stats['positions']
                    f.write(f"  位置统计:\n")
                    f.write(f"    中心: [{pos['center'][0]:.3f}, {pos['center'][1]:.3f}, {pos['center'][2]:.3f}]\n")
                    f.write(f"    标准差: [{pos['std'][0]:.3f}, {pos['std'][1]:.3f}, {pos['std'][2]:.3f}]\n")
                    f.write(f"    范围: X[{pos['min'][0]:.3f}, {pos['max'][0]:.3f}], Y[{pos['min'][1]:.3f}, {pos['max'][1]:.3f}]\n")
                    
                    ts = stats['timestamps']
                    f.write(f"  时间跨度: {ts['span']:.2f}秒\n")
                    
                    # 聚类分析
                    cluster_info = self.detect_clustering(target_type)
                    if cluster_info:
                        f.write(f"  聚类分析:\n")
                        f.write(f"    聚类数: {cluster_info['n_clusters']}\n")
                        f.write(f"    噪声点: {cluster_info['n_noise_points']}\n")
                        for cluster in cluster_info['cluster_centers']:
                            center = cluster['center']
                            f.write(f"    聚类{cluster['cluster_id']}: 中心[{center[0]:.3f}, {center[1]:.3f}], 大小{cluster['size']}\n")
                    
                    f.write("\n")
            
            rospy.loginfo(f"分析报告已保存到: {filename}")
            return True
            
        except Exception as e:
            rospy.logerr(f"保存分析报告失败: {str(e)}")
            return False
    
    def print_real_time_analysis(self):
        """打印实时分析结果"""
        analysis = self.get_comprehensive_analysis()
        
        print("\n" + "="*60)
        print("实时点云分析")
        print("="*60)
        print(f"时间: {time.strftime('%H:%M:%S')}")
        print(f"活跃目标: {analysis['summary']['active_targets']}/{len(analysis['summary']['target_types'])}")
        print(f"总点数: {analysis['summary']['total_points']}")
        
        for target_type, stats in analysis['targets'].items():
            print(f"\n{target_type.upper()}:")
            if stats['valid'] and stats['count'] > 0:
                pos = stats['positions']['center']
                print(f"  点数: {stats['count']}, 中心: [{pos[0]:.2f}, {pos[1]:.2f}], 数据年龄: {stats['data_age']:.1f}s")
            else:
                print("  无数据")
    
    def run(self):
        """运行分析器"""
        rospy.loginfo("开始实时分析...")
        
        while not rospy.is_shutdown():
            current_time = time.time()
            
            if current_time - self.last_analysis_time >= self.analysis_interval:
                self.print_real_time_analysis()
                self.last_analysis_time = current_time
            
            rospy.sleep(1.0)

def main():
    """主函数"""
    try:
        analyzer = PointCloudAnalyzer()
        
        print("点云分析器选项:")
        print("1. 实时分析模式（默认）")
        print("2. 单次分析并导出")
        
        choice = input("选择模式 (1/2): ").strip()
        
        if choice == '2':
            rospy.sleep(2.0)  # 等待数据
            analyzer.export_analysis_to_file()
        else:
            analyzer.run()
            
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\n正在关闭分析器...")
    except Exception as e:
        rospy.logerr(f"分析器运行失败: {str(e)}")

if __name__ == '__main__':
    main()
