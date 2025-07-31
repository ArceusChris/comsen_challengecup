cmake_minimum_required(VERSION 3.0.2)
project(perception)

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
#  LIBRARIES perception
#  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs cv_bridge image_transport
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/yolo11_inference.py
  scripts/simple_test.py
  scripts/test_average_position_feature.py
  scripts/test_yolo_functionality.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
