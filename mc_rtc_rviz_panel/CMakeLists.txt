#
# Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cmake_minimum_required(VERSION 3.1)

set(PROJECT_NAME mc_rtc_rviz_panel)
set(PROJECT_DESCRIPTION "mc_rtc RVIz panel for prototyping controllers")
set(PROJECT_URL "")
project(${PROJECT_NAME} LANGUAGES CXX VERSION 1.1.0)

set(CMAKE_CXX_STANDARD 11)

find_package(mc_rtc REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)
find_package(Qwt)

find_package(catkin)

if(${catkin_FOUND})
  option(BUILD_ROS_FREE_GUI "Build a ROS free GUI" OFF)
  set(DISABLE_ROS 0)
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

  if(NOT ${DISABLE_ROS})
    install(
      FILES
      rviz_plugin.xml
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
  endif()
else()
  set(BUILD_ROS_FREE_GUI ON)
  set(DISABLE_ROS 1)
  set(CATKIN_PACKAGE_BIN_DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
  set(CATKIN_PACKAGE_LIB_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
endif()

add_subdirectory(src)
