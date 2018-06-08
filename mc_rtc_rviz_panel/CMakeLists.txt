cmake_minimum_required(VERSION 2.8)

set(PROJECT_NAME mc_rtc_rviz_panel)
set(PROJECT_DESCRIPTION "mc_rtc RVIz panel for prototyping controllers")
set(PROJECT_URL "")
project(${PROJECT_NAME})

set(CXX_DISABLE_WERROR 0)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  rviz
  roslib
)

if(${catkin_FOUND})
  SET(DISABLE_ROS 0)
  catkin_package(
    CATKIN_DEPENDS
    roscpp
    tf
    rviz
    roslib
  )

  include_directories(${catkin_INCLUDE_DIRS})
  link_directories(${catkin_LIBRARY_DIRS})
else()
  SET(DISABLE_ROS 1)
  add_definitions(-DDISABLE_ROS)
  set(CATKIN_PACKAGE_BIN_DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
  set(CATKIN_PACKAGE_LIB_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
endif()

include(cmake/base.cmake)

add_definitions(-std=c++0x -Wno-error -Wno-pedantic)
add_required_dependency(mc_rtc)

add_subdirectory(src)
