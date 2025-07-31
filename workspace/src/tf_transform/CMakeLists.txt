cmake_minimum_required(VERSION 3.0.2)
project(tf_transform)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  mavros_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tf_transform
#  CATKIN_DEPENDS rospy std_msgs geometry_msgs tf2_ros tf2_geometry_msgs mavros_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/mavros_pose_to_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
