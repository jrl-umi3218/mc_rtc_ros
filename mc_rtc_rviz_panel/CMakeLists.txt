#
# Copyright 2016-2026 CNRS-UM LIRMM, CNRS-AIST JRL
#

cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME mc_rtc_rviz_panel)
set(PROJECT_DESCRIPTION "mc_rtc RVIz panel for prototyping controllers")
set(PROJECT_URL "")
set(CMAKE_CXX_STANDARD 17)

project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.6.1)

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

add_subdirectory(src)

ament_package()
