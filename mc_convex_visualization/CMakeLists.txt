#
# Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
set(PROJECT_NAME mc_convex_visualization)
project(${PROJECT_NAME} LANGUAGES CXX VERSION 1.1.0)

find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  roscpp
  std_msgs
)

catkin_package(CATKIN_DEPENDS roscpp)

find_package(mc_rtc 1.3.0 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

set(src_files
  src/main.cpp
)

add_executable(sch_visualizer ${src_files})
target_link_libraries(sch_visualizer PUBLIC mc_rtc::mc_rbdyn mc_rtc::mc_rtc_ros ${catkin_LIBRARIES})

install(TARGETS sch_visualizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
