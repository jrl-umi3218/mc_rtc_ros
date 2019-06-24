cmake_minimum_required(VERSION 3.1)
project(mc_surfaces_visualization)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  geometry_msgs
  roscpp
  std_msgs
  tf
)

catkin_package()

find_package(mc_rtc REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(src_files
  src/main.cpp
)

add_executable(surfaces_publisher ${src_files})
target_link_libraries(surfaces_publisher PUBLIC mc_rtc::mc_rbdyn ${catkin_LIBRARIES})
