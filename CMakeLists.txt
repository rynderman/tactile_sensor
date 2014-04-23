cmake_minimum_required(VERSION 2.8.3)
project(tactile_sensor)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  moveit_ros_planning_interface
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs 
  tf
  tf_conversions
  rospy
)

catkin_python_setup()

## compiler optimization flags
set(CMAKE_CXX_FLAGS "-march=corei7-avx -DNDEBUG -O3 -fopenmp")

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(insertion src/insertion_vision.cpp)
#add_executable(insertion_py scripts/insertion_vision.py)

#target_link_libraries(insertion ${catkin_LIBRARIES})

install(PROGRAMS scripts/insertion_vision
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)
