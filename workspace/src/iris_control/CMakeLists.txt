cmake_minimum_required(VERSION 3.0.2)
project(iris_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  mavros_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES iris_control
#  CATKIN_DEPENDS rospy std_msgs geometry_msgs mavros_msgs sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/multirotor_control.py
  scripts/drone_control.py
  scripts/vtol_Astar.py
  scripts/vtol_map.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
