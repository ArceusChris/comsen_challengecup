cmake_minimum_required(VERSION 3.0.2)
project(perception_yolov12)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perception_yolov12
#  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs cv_bridge image_transport
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/yolo12_bridge_node.py
  scripts/yolo12_inference.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
