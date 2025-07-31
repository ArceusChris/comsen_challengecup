cmake_minimum_required(VERSION 3.0.2)
project(workspace_utils)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  mavros_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES workspace_utils
#  CATKIN_DEPENDS rospy std_msgs geometry_msgs sensor_msgs mavros_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/get_person_positions.py
  scripts/image_saver.py
  scripts/labelme_to_yolo.py
  scripts/virtual_rc.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(FILES
  zhihang2025_fixed.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install shell scripts
install(PROGRAMS
  scripts/start.sh
  scripts/stop.sh
  scripts/fix_spawn_issue.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
