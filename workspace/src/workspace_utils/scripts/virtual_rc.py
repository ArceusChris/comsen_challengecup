#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import RCIn
import time

def publish_virtual_rc():
    rospy.init_node('virtual_rc_publisher', anonymous=True)
    
    # 为两架无人机发布RC信号
    pub_vtol = rospy.Publisher('/standard_vtol_0/mavros/rc/in', RCIn, queue_size=10)
    pub_iris = rospy.Publisher('/iris_0/mavros/rc/in', RCIn, queue_size=10)
    
    rate = rospy.Rate(50)  # 50Hz
    
    while not rospy.is_shutdown():
        # 创建RC消息
        rc_msg = RCIn()
        rc_msg.header.stamp = rospy.Time.now()
        rc_msg.rssi = 255
        
        # 设置通道值 (1000-2000, 中位1500)
        # 通道1-4: 摇杆输入, 通道5: 模式切换
        rc_msg.channels = [1500, 1500, 1500, 1500, 2000, 1500, 1500, 1500]
        
        # 发布到两架无人机
        pub_vtol.publish(rc_msg)
        pub_iris.publish(rc_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.loginfo('Starting virtual RC publisher...')
        publish_virtual_rc()
    except rospy.ROSInterruptException:
        pass
