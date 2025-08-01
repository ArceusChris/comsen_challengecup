cmake_minimum_required(VERSION 3.0.2)
project(vtol_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  mavros_msgs
  sensor_msgs
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vtol_control
#  CATKIN_DEPENDS rospy std_msgs geometry_msgs mavros_msgs sensor_msgs nav_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/vtol_demo.py
  scripts/vtol_fly.py
  scripts/vtol_ros.py
  scripts/vtol_Astar.py
  scripts/vtol_map.py
  scripts/vtol_smoothpath.py
  scripts/image_capture.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
