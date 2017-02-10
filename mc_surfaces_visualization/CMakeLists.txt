cmake_minimum_required(VERSION 2.8.3)
project(mc_surfaces_visualization)

include(cmake/base.cmake)

find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  geometry_msgs
  roscpp
  std_msgs
  tf
)

catkin_package()

add_required_dependency("mc_rtc", "mc_rtc_d")

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(src_files
  src/main.cpp
)

# build in C++11 (std::shared_ptr)
add_definitions(-std=c++0x)

add_executable(surfaces_publisher ${src_files})
pkg_config_use_dependency(surfaces_publisher mc_rtc)

target_link_libraries(surfaces_publisher
   ${catkin_LIBRARIES}
)
