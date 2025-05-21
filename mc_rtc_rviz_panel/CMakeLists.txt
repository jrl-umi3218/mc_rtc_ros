#
# Copyright 2016-2023 CNRS-UM LIRMM, CNRS-AIST JRL
#

# Require 3.14 to use simplified linking with OBJECT libraries
cmake_minimum_required(VERSION 3.14)

set(PROJECT_NAME mc_rtc_rviz_panel)
set(PROJECT_DESCRIPTION "mc_rtc RVIz panel for prototyping controllers")
set(PROJECT_URL "")
set(CMAKE_CXX_STANDARD 17)

project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.6.1)

if(NOT DEFINED ENV{ROS_VERSION})
  message(FATAL_ERROR "Expected to find ROS_VERSION in the environment")
endif()
set(ROS_VERSION $ENV{ROS_VERSION})

find_package(mc_rtc REQUIRED)
if(NOT TARGET mc_rtc::mc_rtc_ros)
  message(FATAL_ERROR "mc_rtc ROS plugin is required to build this project")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)
find_package(Qwt)

macro(add_ros2_dependency PKG TARGET)
  find_package(${PKG} REQUIRED)
  target_link_libraries(ROSDependencies INTERFACE ${PKG}::${TARGET})
endmacro()

if(ROS_VERSION EQUAL 1)
  find_package(catkin REQUIRED COMPONENTS roscpp tf rviz roslib)

  catkin_package(CATKIN_DEPENDS roscpp tf rviz roslib)

  include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS})

  install(FILES rviz_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
elseif(ROS_VERSION EQUAL 2)
  cmake_minimum_required(VERSION 3.22)
  find_package(ament_cmake REQUIRED)

  add_library(ROSDependencies INTERFACE IMPORTED)
  add_ros2_dependency(rclcpp rclcpp)
  add_ros2_dependency(tf2_ros tf2_ros)
  add_ros2_dependency(visualization_msgs
                      visualization_msgs__rosidl_typesupport_cpp)
  add_ros2_dependency(interactive_markers interactive_markers)
  add_ros2_dependency(pluginlib pluginlib)
  add_ros2_dependency(rviz_common rviz_common)

  pluginlib_export_plugin_description_file(rviz_common plugin/rviz_plugin.xml)
else()
  message(FATAL_ERROR "ROS_VERSION ${ROS_VERSION} is not supported yet")
endif()

add_subdirectory(src)

if(ROS_VERSION EQUAL 2)
  ament_package()
endif()
