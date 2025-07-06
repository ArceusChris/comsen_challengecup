import random
import numpy as np
from scipy.interpolate import interp1d

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Twist, Quaternion
from std_msgs.msg import Float32
from gazebo_msgs.srv import GetLinkState
import tf
import math
import time

# 运动参数
man_z = 0.2
radius = 3.0
angular_velocity = 0.5  # rad/s
landing_offset_x = -1.23  # landing 比 person 多的 x 偏移量

def generate_valid_centers(num_points=2, x_fixed=1495, y_range=(-150, 150), min_dist=20):
    centers = []
    attempts = 0
    while len(centers) < num_points and attempts < 1000:
        y_candidate = random.uniform(*y_range)
        is_valid = True
        for _, y in centers:
            if abs(y - y_candidate) < min_dist:
                is_valid = False
                break
        if is_valid:
            centers.append((x_fixed, y_candidate))
        attempts += 1
    if len(centers) < num_points:
        raise ValueError("无法在给定范围内生成满足条件的中心点，请调整参数")
    return centers

def pose_publisher():
    centers = generate_valid_centers()  # 2个中心点，对应 yellow 和 white
    model_state_pub = rospy.Publisher('/gazebo/set_model_states', ModelStates, queue_size=1)
    rate = rospy.Rate(100)
    
    poses_msg = ModelStates()
    poses_msg.name = ['person_yellow', 'person_white', 'landing_yellow', 'landing_white']
    poses_msg.pose = [Pose() for _ in range(4)]
    poses_msg.twist = [Twist() for _ in range(4)]

    # 不同模型使用不同朝向
    quat_person = tf.transformations.quaternion_from_euler(0, 0, 0)         # yaw = 0
    quat_landing = tf.transformations.quaternion_from_euler(0, -1.57, 0)    # yaw = -1.57


    start_time = time.time()

    try:
        while not rospy.is_shutdown():
            t = time.time() - start_time

            for i, (cx, cy) in enumerate(centers):  # i = 0 for yellow, i = 1 for white
                theta = angular_velocity * t + i * 2 * math.pi / len(centers)
                x = cx + radius * math.cos(theta)
                y = cy + radius * math.sin(theta)

                # person 模型
                poses_msg.pose[i].position.x = x
                poses_msg.pose[i].position.y = y
                poses_msg.pose[i].position.z = man_z
                poses_msg.pose[i].orientation = Quaternion(*quat_person)

                # landing 模型
                poses_msg.pose[i + 2].position.x = x + landing_offset_x
                poses_msg.pose[i + 2].position.y = y
                poses_msg.pose[i + 2].position.z = man_z
                poses_msg.pose[i + 2].orientation = Quaternion(*quat_landing)

            model_state_pub.publish(poses_msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    rospy.init_node('control_targets')
    get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    pose_publisher()
