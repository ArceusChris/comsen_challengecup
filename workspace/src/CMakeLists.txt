# toplevel cmake file for a catkin workspace
# see http://ros.org/doc/api/catkin/html/user_guide/supposed.html

cmake_minimum_required(VERSION 3.0.2)
project(workspace)

find_package(catkin REQUIRED)
catkin_workspace()
