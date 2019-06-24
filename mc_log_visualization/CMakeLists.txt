cmake_minimum_required(VERSION 3.1)
project(mc_log_visualization)
set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  roscpp
  std_msgs
)

catkin_package()

find_package(mc_rtc REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

set(src_files
  src/mc_log_visualization.cpp
  src/LogPublisher.h
  src/LogPublisher.cpp
  src/LogRobot.h
  src/LogRobot.cpp
)

add_executable(mc_log_visualization ${src_files})
target_link_libraries(mc_log_visualization PUBLIC ${catkin_LIBRARIES} mc_rtc::mc_control)
