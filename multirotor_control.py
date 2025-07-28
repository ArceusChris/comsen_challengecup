from drone_control import DroneController
import sys
import math
import time
import rospy
from geometry_msgs.msg import PoseStamped

MAX_LINEAR = 20.0

class MultirotorControl:
    def __init__(self, controller):
        self.controller = controller
        self.MAX_LINEAR = MAX_LINEAR

    def takeoff(self, altitude=20):
        """无人机起飞到指定高度"""
        print(f"开始起飞到{altitude}米高度...")
        self.controller.auto_takeoff_sequence()
        
        flag = True  # 修复：添加缺失的flag变量
        last_check_time = time.time()
        check_interval = 0.5  # 0.5秒检查一次高度
        rate = rospy.Rate(10)  # 10Hz循环频率

        while flag and not rospy.is_shutdown():
            current_time = time.time()

            if current_time - last_check_time >= check_interval:
                position = self.controller.get_position_xyz()
                if position is None:
                    print("等待位置信息...")
                else:  
                    x, y, z = position
                    print(f"当前高度: {z:.2f}m")
                    if z > altitude:
                        self.controller.current_twist.linear.z = 0.0
                        self.controller.set_hover_mode()
                        print("达到目标高度，准备水平移动")
                        flag = False
                
                last_check_time = current_time
            
            rate.sleep()  
        
        return True


    def land(self, altitude=0.65):
        """无人机降落到指定高度"""
        print(f"开始降落到{altitude}米高度...")
        self.controller.auto_landing_sequence()
        
        flag = True
        last_check_time = time.time()
        check_interval = 0.5  # 0.5秒检查一次高度
        rate = rospy.Rate(10)  # 10Hz循环频率       
        
        # 添加速度控制变量
        last_update_time = time.time()
        update_interval = 0.5  # 0.5秒更新一次速度
    
        while flag and not rospy.is_shutdown():
            current_time = time.time()

            # 检查是否到了更新速度的时间
            if current_time - last_update_time >= update_interval:
                position = self.controller.get_position_xyz()
                if position is None:
                    print("等待位置信息...")
                else:  
                    x, y, z = position
                    
                    # 计算当前高度与目标高度的差值
                    height_diff = z - altitude
                    print(f"当前高度: {z:.2f}m, 目标高度: {altitude:.2f}m, 高度差: {height_diff:.2f}m")
                    
                    if height_diff <= 0.1:  # 接近目标高度
                        self.controller.current_twist.linear.z = 0.0
                        self.controller.set_hover_mode()
                        print("达到目标降落高度，准备悬停")
                        flag = False
                    else:
                        # 应用速度衰减函数，与go_to_position中相同的逻辑
                        if height_diff > 0.01:
                            # 使用1米作为开始减速的距离
                            speed_factor = min(1.0, height_diff / 1.0)  # 1米内开始减速
                            if speed_factor != 1.0:
                                speed_factor = 1.0 / math.exp(1.0 / speed_factor)  # 使用指数衰减函数
                            
                            # 设置下降速度（负值表示下降）
                            descent_speed = -self.MAX_LINEAR * speed_factor * 0.5  # 降落速度为最大速度的一半
                            self.controller.current_twist.linear.z = descent_speed
                            
                            print(f"更新下降速度: vz={descent_speed:.2f}, 速度系数: {speed_factor:.3f}")
                        else:
                            self.controller.current_twist.linear.z = 0.0
                
                last_update_time = current_time
            
            # 兼容原有的时间检查逻辑（保持0.5秒的状态输出）
            if current_time - last_check_time >= check_interval:
                last_check_time = current_time
            
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                print("收到ROS关闭信号，退出降落循环")
                break
    
        return True

    def go_to_position(self, target_position):
        """
        控制无人机移动到指定位置
        该函数分为2种情况：
        1.无人机在悬停
        2.无人机正在运动
        target_position应该为[x, y]
        """
        if not self.controller.is_armed:
            self.controller.arm()

        pose_info, _ = self.controller.get_current_position()
        if pose_info is None:
            print("无法获取当前位置信息")
            return False
        
        position = self.controller.get_position_xyz()
        if position is None:
            print("无法获取位置坐标")
            return False
        
        x, y, z = position
        dx = target_position[0] - x
        dy = target_position[1] - y
        # 避免除零错误
        distance = math.sqrt(dx * dx + dy * dy + 1e-6)  # 添加一个小值以避免除零错误
        if distance < 0.1:
            print("已接近目标位置")
            return True
        
        print(f'当前无人机位置: x={x}, y={y}, z={z}')
        self.controller.current_twist.linear.x = self.MAX_LINEAR * dx / distance
        self.controller.current_twist.linear.y = self.MAX_LINEAR * dy / distance
        self.controller.current_twist.linear.z = 0.0
        self.controller.current_twist.angular.z = 0.0

        done = False
        
        # 添加计时器变量
        last_update_time = time.time()
        update_interval = 0.5  # 0.5秒更新一次速度
        rate = rospy.Rate(20)

        while not done and not rospy.is_shutdown():
            current_time = time.time()
            
            # 检查是否到了更新速度的时间
            if current_time - last_update_time >= update_interval:
                position = self.controller.get_position_xyz()
                if position is None:
                    print("位置信息丢失")
                    break
                    
                x, y, z = position
                dx = target_position[0] - x
                dy = target_position[1] - y
                
                # 计算当前距离
                distance = math.sqrt(dx * dx + dy * dy)
                print(f'当前无人机位置: x={x:.2f}, y={y:.2f}, z={z:.2f}, 距离目标: {distance:.2f}m')

                # 检查是否到达目标
                if distance < 1.0:  # 修正到达判断逻辑
                    print(f"无人机到达目标位置: x={x:.2f}, y={y:.2f}, z={z:.2f}")
                    self.controller.current_twist.linear.x = 0.0
                    self.controller.current_twist.linear.y = 0.0
                    self.controller.current_twist.linear.z = 0.0
                    self.controller.current_twist.angular.z = 0.0
                    done = True  # 停止运动
                else:
                    # 更新速度命令（避免除零错误）
                    if distance > 0.01:
                        speed_factor = min(1.0, distance / 5.0)  # 5米内开始减速
                        if speed_factor != 1.0:
                            speed_factor = 1.0 / math.exp(1.0 / speed_factor)  # 使用指数衰减函数
                        self.controller.current_twist.linear.x = self.MAX_LINEAR * speed_factor * dx / distance
                        self.controller.current_twist.linear.y = self.MAX_LINEAR * speed_factor * dy / distance
                        print(f"更新速度: vx={self.controller.current_twist.linear.x:.2f}, vy={self.controller.current_twist.linear.y:.2f}")
                    else:
                        self.controller.current_twist.linear.x = 0.0
                        self.controller.current_twist.linear.y = 0.0
                
                # 更新最后一次更新时间
                last_update_time = current_time
            
            # 控制循环频率
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                print("收到ROS关闭信号，退出位置控制循环")
                break
        
        return done

    def go_to_target(self, target_position):
        """移动到目标位置的便捷方法"""
        print(f"开始移动到目标位置 {target_position}...")
        success = self.go_to_position(target_position)
        self.controller.set_hover_mode()
        return success

    def execute_mission(self, altitude_high=20, altitude_low=0.65, target_position=[1495, -105]):
        """执行完整任务：起飞 -> 移动到目标 -> 悬停"""
        print("开始执行任务...")
        
        # 步骤1：起飞
        takeoff_success = self.takeoff(altitude_high)
        if not takeoff_success:
            print("起飞失败")
            return False
        
        # 步骤2：移动到目标位置
        move_success = self.go_to_target(target_position)

        landing_success = self.land(altitude_low)
        self.controller.set_hover_mode()
        if not landing_success:
            print("降落失败")
            return False
        else:
            print("任务执行成功")
            return True

def main():
    if len(sys.argv) < 4:
        print("用法: python3 multirotor_control.py <multirotor_type> <multirotor_id> <control_type>")
        return
        
    multirotor_type = sys.argv[1]
    multirotor_id = int(sys.argv[2])
    control_type = sys.argv[3]
    
    # 创建控制器和控制类
    controller = DroneController(multirotor_type, multirotor_id, control_type)
    multirotor_control = MultirotorControl(controller)
    
    # 执行任务
    success = multirotor_control.execute_mission(altitude_high=20, altitude_low=0.65, target_position=[1495, -105])

    if success:
        print("保持悬停，按Ctrl+C退出...")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("收到退出信号")
    
    print("程序退出")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n收到Ctrl+C信号，正在退出...")
    except Exception as e:
        print(f"程序异常: {e}")
    finally:
        print("清理完成")

