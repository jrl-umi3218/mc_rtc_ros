cmake_minimum_required(VERSION 2.8.3)
project(sch_visualization_ros)

include(cmake/base.cmake)

find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  roscpp
  std_msgs
)

catkin_package()

ADD_REQUIRED_DEPENDENCY("mc_rtc")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

find_package(PkgConfig)

set(src_files
  src/main.cpp
)

# build in C++11 (std::shared_ptr)
add_definitions(-std=c++0x)

add_executable(sch_visualizer ${src_files})
PKG_CONFIG_USE_DEPENDENCY(sch_visualizer mc_rtc)

target_link_libraries(sch_visualizer
   ${catkin_LIBRARIES}
)
