#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped

class MavrosPoseToTF:
    def __init__(self):
        rospy.init_node('mavros_pose_to_tf', anonymous=True)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 订阅两个无人机的位姿话题
        self.iris_pose_sub = rospy.Subscriber(
            '/iris_0/mavros/local_position/pose', 
            PoseStamped, 
            self.iris_pose_callback
        )
        
        self.vtol_pose_sub = rospy.Subscriber(
            '/standard_vtol_0/mavros/local_position/pose', 
            PoseStamped, 
            self.vtol_pose_callback
        )
        
        rospy.loginfo("MAVROS Pose to TF converter started")
        rospy.loginfo("Subscribing to:")
        rospy.loginfo("  - /iris_0/mavros/local_position/pose")
        rospy.loginfo("  - /standard_vtol_0/mavros/local_position/pose")
        
    def pose_to_transform(self, pose_stamped, parent_frame, child_frame):
        """将PoseStamped消息转换为TransformStamped"""
        transform = TransformStamped()
        
        # 设置header
        transform.header.stamp = pose_stamped.header.stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        
        # 设置变换
        transform.transform.translation.x = pose_stamped.pose.position.x
        transform.transform.translation.y = pose_stamped.pose.position.y
        transform.transform.translation.z = pose_stamped.pose.position.z
        
        transform.transform.rotation.x = pose_stamped.pose.orientation.x
        transform.transform.rotation.y = pose_stamped.pose.orientation.y
        transform.transform.rotation.z = pose_stamped.pose.orientation.z
        transform.transform.rotation.w = pose_stamped.pose.orientation.w
        
        return transform
        
    def iris_pose_callback(self, msg):
        """处理iris_0的位姿消息"""
        try:
            # 发布从map到iris_0/base_link的变换
            transform = self.pose_to_transform(msg, "map", "iris_0/base_link")
            self.tf_broadcaster.sendTransform(transform)
            
            # 同时发布相机坐标系变换（静态偏移）
            self.publish_iris_camera_transforms(msg.header.stamp)
            
        except Exception as e:
            rospy.logwarn(f"Error processing iris pose: {e}")
            
    def vtol_pose_callback(self, msg):
        """处理standard_vtol_0的位姿消息"""
        try:
            # 发布从map到standard_vtol_0/base_link的变换
            transform = self.pose_to_transform(msg, "map", "standard_vtol_0/base_link")
            self.tf_broadcaster.sendTransform(transform)
            
            # 同时发布相机坐标系变换（静态偏移）
            self.publish_vtol_camera_transforms(msg.header.stamp)
            
        except Exception as e:
            rospy.logwarn(f"Error processing vtol pose: {e}")
            
    def publish_iris_camera_transforms(self, stamp):
        """发布iris_0相机坐标系变换"""
        transforms = []
        
        # RealSense 前视相机变换 (前方0.1m)
        # 机体坐标系(x前y左z上) -> 相机坐标系(x右y下z前)
        # 需要的变换: 机体x(前)->相机z(前), 机体y(左)->相机-x(右), 机体z(上)->相机-y(下)
        # 四元数: (0.5, -0.5, -0.5, 0.5) - 正确的前视相机变换
        realsense_tf = TransformStamped()
        realsense_tf.header.stamp = stamp
        realsense_tf.header.frame_id = "iris_0/base_link"
        realsense_tf.child_frame_id = "iris_0/stereo_camera_frame"
        realsense_tf.transform.translation.x = 0.1
        realsense_tf.transform.translation.y = 0.0
        realsense_tf.transform.translation.z = 0.0
        # 四元数: (0.5, -0.5, -0.5, 0.5) - z轴指向前方
        realsense_tf.transform.rotation.x = 0.5
        realsense_tf.transform.rotation.y = -0.5
        realsense_tf.transform.rotation.z = -0.5
        realsense_tf.transform.rotation.w = 0.5
        transforms.append(realsense_tf)
        
        # 下视相机变换 (下方0.03m)
        # 机体坐标系(x前y左z上) -> 下视相机坐标系(x右y下z朝地面)
        # 需要的变换: 机体x(前)->相机-z(下), 机体y(左)->相机-x(右), 机体z(上)->相机-y(下)
        # 四元数: (0.5, -0.5, 0.5, 0.5) - 正确的下视相机变换
        camera_tf = TransformStamped()
        camera_tf.header.stamp = stamp
        camera_tf.header.frame_id = "iris_0/base_link"
        camera_tf.child_frame_id = "iris_0/camera_link"
        camera_tf.transform.translation.x = 0.0
        camera_tf.transform.translation.y = 0.0
        camera_tf.transform.translation.z = -0.03
        # 四元数: (0.5, -0.5, 0.5, 0.5) - z轴指向地面(下方)
        camera_tf.transform.rotation.x = 0.707107
        camera_tf.transform.rotation.y = -0.707107
        camera_tf.transform.rotation.z = 0.0
        camera_tf.transform.rotation.w = 0.0
        transforms.append(camera_tf)
        
        self.tf_broadcaster.sendTransform(transforms)
        
    def publish_vtol_camera_transforms(self, stamp):
        """发布standard_vtol_0相机坐标系变换"""
        # 下视相机变换 (下方0.03m)
        # 机体坐标系(x前y左z上) -> 相机坐标系(x右y下z前，但朝向地面)
        # 需要绕y轴旋转180度使z轴朝下，然后绕z轴旋转-90度使x轴朝右
        camera_tf = TransformStamped()
        camera_tf.header.stamp = stamp
        camera_tf.header.frame_id = "standard_vtol_0/base_link"
        camera_tf.child_frame_id = "standard_vtol_0/camera_link"
        camera_tf.transform.translation.x = 0.0
        camera_tf.transform.translation.y = 0.0
        camera_tf.transform.translation.z = -0.03
        # 四元数: (0.5, -0.5, 0.5, 0.5) - 正确的下视相机变换
        camera_tf.transform.rotation.x = 0.707107
        camera_tf.transform.rotation.y = -0.707107
        camera_tf.transform.rotation.z = 0.0
        camera_tf.transform.rotation.w = 0.0
        
        self.tf_broadcaster.sendTransform(camera_tf)
        
    def run(self):
        """运行节点"""
        rospy.spin()

if __name__ == '__main__':
    try:
        converter = MavrosPoseToTF()
        converter.run()
    except rospy.ROSInterruptException:
        pass
