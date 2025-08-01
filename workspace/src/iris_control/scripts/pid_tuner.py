#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PID参数动态调整工具

该脚本提供了一个命令行界面来动态调整多旋翼无人机的PID参数。
可以在无人机运行过程中实时调整参数而不需要重启节点。

使用方法：
python3 pid_tuner.py [节点名称]

如果不指定节点名称，默认使用 'multirotor_control_node'
"""

import rospy
import sys

class PIDTuner:
    def __init__(self, node_name="multirotor_control_node"):
        self.node_name = node_name
        self.param_prefix = f"/{node_name}/"
        
        # 初始化ROS节点
        rospy.init_node('pid_tuner', anonymous=True)
        
        print(f"PID参数调整工具已启动，目标节点: {node_name}")
        print("输入 'help' 查看可用命令")
        
    def show_help(self):
        """显示帮助信息"""
        print("\n=== PID参数调整工具帮助 ===")
        print("可用命令：")
        print("  show                    - 显示当前所有参数")
        print("  set <param> <value>     - 设置参数值")
        print("  get <param>             - 获取参数值")
        print("  preset <name>           - 加载预设参数")
        print("  help                    - 显示此帮助")
        print("  quit/exit               - 退出程序")
        print("\n可设置的参数：")
        print("  pid_x_kp, pid_x_ki, pid_x_kd    - X轴PID参数")
        print("  pid_y_kp, pid_y_ki, pid_y_kd    - Y轴PID参数")
        print("  pid_z_kp, pid_z_ki, pid_z_kd    - Z轴PID参数")
        print("  max_vel_xy, max_vel_z            - 速度限制")
        print("  landing_threshold                - 降落阈值")
        print("  min_altitude                     - 最小高度")
        print("  descent_rate                     - 降落速率")
        print("  target_timeout                   - 目标超时")
        print("\n预设参数：")
        print("  conservative  - 保守参数（慢速稳定）")
        print("  aggressive    - 激进参数（快速响应）")
        print("  default       - 默认参数")
        print("========================\n")
    
    def param_name_to_ros_param(self, param_name):
        """将简化的参数名转换为ROS参数路径"""
        mapping = {
            'pid_x_kp': 'pid_x/kp',
            'pid_x_ki': 'pid_x/ki', 
            'pid_x_kd': 'pid_x/kd',
            'pid_y_kp': 'pid_y/kp',
            'pid_y_ki': 'pid_y/ki',
            'pid_y_kd': 'pid_y/kd',
            'pid_z_kp': 'pid_z/kp',
            'pid_z_ki': 'pid_z/ki',
            'pid_z_kd': 'pid_z/kd',
            'max_vel_xy': 'max_vel_xy',
            'max_vel_z': 'max_vel_z',
            'landing_threshold': 'landing_threshold',
            'min_altitude': 'min_altitude',
            'descent_rate': 'descent_rate',
            'target_timeout': 'target_timeout'
        }
        
        if param_name in mapping:
            return self.param_prefix + mapping[param_name]
        else:
            return self.param_prefix + param_name
    
    def show_all_params(self):
        """显示所有当前参数"""
        print("\n=== 当前PID参数 ===")
        
        params = [
            ('pid_x_kp', 'X轴比例增益'),
            ('pid_x_ki', 'X轴积分增益'), 
            ('pid_x_kd', 'X轴微分增益'),
            ('pid_y_kp', 'Y轴比例增益'),
            ('pid_y_ki', 'Y轴积分增益'),
            ('pid_y_kd', 'Y轴微分增益'),
            ('pid_z_kp', 'Z轴比例增益'),
            ('pid_z_ki', 'Z轴积分增益'),
            ('pid_z_kd', 'Z轴微分增益'),
            ('max_vel_xy', 'XY最大速度'),
            ('max_vel_z', 'Z最大速度'),
            ('landing_threshold', '降落阈值'),
            ('min_altitude', '最小高度'),
            ('descent_rate', '降落速率'),
            ('target_timeout', '目标超时')
        ]
        
        for param_name, description in params:
            ros_param = self.param_name_to_ros_param(param_name)
            try:
                value = rospy.get_param(ros_param, "未设置")
                print(f"  {param_name:15} ({description:8}): {value}")
            except Exception as e:
                print(f"  {param_name:15} ({description:8}): 读取失败 - {e}")
        
        print("===================\n")
    
    def set_param(self, param_name, value):
        """设置参数值"""
        ros_param = self.param_name_to_ros_param(param_name)
        try:
            # 尝试转换为数字
            if '.' in str(value):
                value = float(value)
            else:
                try:
                    value = int(value)
                except:
                    value = float(value)
            
            rospy.set_param(ros_param, value)
            print(f"✓ 参数设置成功: {param_name} = {value}")
            
        except Exception as e:
            print(f"✗ 参数设置失败: {e}")
    
    def get_param(self, param_name):
        """获取参数值"""
        ros_param = self.param_name_to_ros_param(param_name)
        try:
            value = rospy.get_param(ros_param)
            print(f"{param_name} = {value}")
        except Exception as e:
            print(f"✗ 获取参数失败: {e}")
    
    def load_preset(self, preset_name):
        """加载预设参数"""
        presets = {
            'default': {
                'pid_x_kp': 2.0, 'pid_x_ki': 0.0, 'pid_x_kd': 0.0,
                'pid_y_kp': 2.0, 'pid_y_ki': 0.0, 'pid_y_kd': 0.0,
                'pid_z_kp': 1.0, 'pid_z_ki': 0.0, 'pid_z_kd': 0.0,
                'max_vel_xy': 1.5, 'max_vel_z': 1.0,
                'landing_threshold': 30, 'min_altitude': 0.65,
                'descent_rate': 0.25, 'target_timeout': 2.0
            },
            'conservative': {
                'pid_x_kp': 1.0, 'pid_x_ki': 0.0, 'pid_x_kd': 0.1,
                'pid_y_kp': 1.0, 'pid_y_ki': 0.0, 'pid_y_kd': 0.1,
                'pid_z_kp': 0.5, 'pid_z_ki': 0.0, 'pid_z_kd': 0.05,
                'max_vel_xy': 1.0, 'max_vel_z': 0.5,
                'landing_threshold': 20, 'min_altitude': 0.8,
                'descent_rate': 0.15, 'target_timeout': 3.0
            },
            'aggressive': {
                'pid_x_kp': 3.0, 'pid_x_ki': 0.1, 'pid_x_kd': 0.0,
                'pid_y_kp': 3.0, 'pid_y_ki': 0.1, 'pid_y_kd': 0.0,
                'pid_z_kp': 1.5, 'pid_z_ki': 0.05, 'pid_z_kd': 0.0,
                'max_vel_xy': 2.0, 'max_vel_z': 1.5,
                'landing_threshold': 40, 'min_altitude': 0.5,
                'descent_rate': 0.35, 'target_timeout': 1.5
            }
        }
        
        if preset_name in presets:
            print(f"加载预设参数: {preset_name}")
            preset = presets[preset_name]
            for param_name, value in preset.items():
                self.set_param(param_name, value)
            print(f"✓ 预设参数 '{preset_name}' 加载完成")
        else:
            print(f"✗ 未知的预设参数: {preset_name}")
            print(f"可用预设: {list(presets.keys())}")
    
    def run(self):
        """运行交互式界面"""
        while not rospy.is_shutdown():
            try:
                cmd = input("PID调优> ").strip().split()
                
                if not cmd:
                    continue
                    
                command = cmd[0].lower()
                
                if command in ['quit', 'exit', 'q']:
                    print("退出PID调优工具")
                    break
                    
                elif command == 'help':
                    self.show_help()
                    
                elif command == 'show':
                    self.show_all_params()
                    
                elif command == 'set':
                    if len(cmd) >= 3:
                        param_name = cmd[1]
                        value = cmd[2]
                        self.set_param(param_name, value)
                    else:
                        print("用法: set <参数名> <值>")
                        
                elif command == 'get':
                    if len(cmd) >= 2:
                        param_name = cmd[1]
                        self.get_param(param_name)
                    else:
                        print("用法: get <参数名>")
                        
                elif command == 'preset':
                    if len(cmd) >= 2:
                        preset_name = cmd[1]
                        self.load_preset(preset_name)
                    else:
                        print("用法: preset <预设名>")
                        print("可用预设: default, conservative, aggressive")
                        
                else:
                    print(f"未知命令: {command}")
                    print("输入 'help' 查看可用命令")
                    
            except KeyboardInterrupt:
                print("\n退出PID调优工具")
                break
            except EOFError:
                print("\n退出PID调优工具")
                break
            except Exception as e:
                print(f"错误: {e}")

def main():
    node_name = "multirotor_control_node"
    
    if len(sys.argv) > 1:
        node_name = sys.argv[1]
    
    tuner = PIDTuner(node_name)
    tuner.run()

if __name__ == "__main__":
    main()
