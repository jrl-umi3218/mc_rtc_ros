cmake_minimum_required(VERSION 2.8)

set(CXX_DISABLE_WERROR 0)

include(cmake/base.cmake)

set(PROJECT_NAME mc_rtc_rviz_panel)
set(PROJECT_DESCRIPTION "mc_rtc RVIz panel for prototyping controllers")
set(PROJECT_URL "")

add_definitions(-std=c++0x -Wno-error -Wno-pedantic)
add_required_dependency(mc_rtc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  rviz
  roslib
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  tf
  rviz
  roslib
)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(src)
