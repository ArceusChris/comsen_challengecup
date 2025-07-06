import rospy
import random
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Pose, TwistStamped

def Point():
    first_point_x = 1500 
    first_point_y = 0
    
    return first_point_x, first_point_y

#话题中position表示mountain1的中心点，orientation表示mountain2的中心点
def First_point_Pub():
    x1, y1 = Point()
    first_point = rospy.Publisher('/zhihang/first_point', Pose, queue_size=1)
    f = 10.0
    rate = rospy.Rate(f)
    first_point_pub = Pose()
    first_point_pub.position.x, first_point_pub.position.y, first_point_pub.position.z =  x1, y1, 0.0
    first_point_pub.orientation.x, first_point_pub.orientation.y, first_point_pub.orientation.z, first_point_pub.orientation.w = 0.0, 0.0, 0.0, 0.0
    while not rospy.is_shutdown():
        try:
            first_point.publish(first_point_pub)
        except:
            continue
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node('zhihang_first_point')
    First_point_Pub()
    rospy.spin()
