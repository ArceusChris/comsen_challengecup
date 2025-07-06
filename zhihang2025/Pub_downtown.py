import rospy
import random
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, Pose, TwistStamped

def Point():
    mountain1_x = 1200 #mountain1的中心点
    mountain1_y = 0
    mountain2_x = 1800 #mountain２的中心点
    mountain2_y = 0
    return mountain1_x, mountain1_y, mountain2_x, mountain2_y

#话题中position表示mountain1的中心点，orientation表示mountain2的中心点
def Downtown_Pub():
    x1, y1, x2, y2= Point()
    downtown = rospy.Publisher('/zhihang/downtown', Pose, queue_size=1)
    f = 10.0
    rate = rospy.Rate(f)
    mountain_pub = Pose()
    mountain_pub.position.x, mountain_pub.position.y, mountain_pub.position.z =  x1, y1, 0.0
    mountain_pub.orientation.x, mountain_pub.orientation.y, mountain_pub.orientation.z, mountain_pub.orientation.w = x2, y2, 0.0, 0.0
    while not rospy.is_shutdown():
        try:
            downtown.publish(mountain_pub)
        except:
            continue
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node('zhihang_downtown')
    Downtown_Pub()
    rospy.spin()