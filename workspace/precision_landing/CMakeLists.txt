cmake_minimum_required(VERSION 3.0.2)
project(precision_landing)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs mavros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/precision_landing_controller.py
  scripts/landing_state_machine.py
  scripts/visual_servo_controller.py
  scripts/test_precision_landing.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
