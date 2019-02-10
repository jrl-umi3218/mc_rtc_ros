cmake_minimum_required(VERSION 2.8.3)
project(mc_log_visualization)

include(cmake/base.cmake)

find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  roscpp
  std_msgs
)

catkin_package()

add_required_dependency("mc_rtc" "mc_rtc_d")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

set(src_files
  src/mc_log_visualization.cpp
  src/LogRobot.h
  src/LogRobot.cpp
)

add_definitions(-std=c++0x)

add_executable(mc_log_visualization ${src_files})
pkg_config_use_dependency(mc_log_visualization mc_rtc)
target_link_libraries(mc_log_visualization ${catkin_LIBRARIES})
