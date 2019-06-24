cmake_minimum_required(VERSION 3.1)
project(mc_convex_visualization)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  roscpp
  std_msgs
)

catkin_package(CATKIN_DEPENDS roscpp)

find_package(mc_rtc REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

set(src_files
  src/main.cpp
)

add_executable(sch_visualizer ${src_files})
target_link_libraries(sch_visualizer PUBLIC mc_rtc::mc_rbdyn ${catkin_LIBRARIES})
