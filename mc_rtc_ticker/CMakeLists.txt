#
# Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_NAME mc_rtc_ticker)
project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.6.1)

if(NOT DEFINED ENV{ROS_VERSION})
  message(FATAL_ERROR "Expected to find ROS_VERSION in the environment")
endif()
set(ROS_VERSION $ENV{ROS_VERSION})

if(ROS_VERSION EQUAL 1)
  find_package(catkin REQUIRED)
  catkin_package()

  install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
elseif(ROS_VERSION EQUAL 2)
  find_package(ament_cmake REQUIRED)

  install(
    FILES launch/display.2.launch
    DESTINATION share/${PROJECT_NAME}/launch/
    RENAME display.launch)
  install(
    FILES launch/display.2.rviz
    DESTINATION share/${PROJECT_NAME}/launch/
    RENAME display.rviz)

  ament_package()
else()
  message(FATAL_ERROR "ROS_VERSION ${ROS_VERSION} is not supported yet")
endif()
