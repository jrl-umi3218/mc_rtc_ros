cmake_minimum_required(VERSION 3.22)
project(mc_rtc_ros)

# Default to ROS2 if ROS_VERSION is not set
if(NOT DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION 2)
else()
  set(ROS_VERSION $ENV{ROS_VERSION})
endif()

if(ROS_VERSION EQUAL 1)
  find_package(catkin REQUIRED)
  catkin_metapackage()
elseif(ROS_VERSION EQUAL 2)
  find_package(ament_cmake REQUIRED)
  ament_package()
else()
  message(FATAL_ERROR "Unsupported ROS_VERSION: ${ROS_VERSION}")
endif()
