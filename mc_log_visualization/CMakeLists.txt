cmake_minimum_required(VERSION 2.8.3)
project(mc_log_visualization)

include(cmake/base.cmake)

find_package(catkin REQUIRED COMPONENTS
  visualization_msgs
  roscpp
  std_msgs
)

catkin_package()

find_package(OpenGL REQUIRED)
ADD_REQUIRED_DEPENDENCY("mc_rtc" "mc_rtc_d")
ADD_REQUIRED_DEPENDENCY("glfw3")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

set(src_files
  src/mc_log_visualization.cpp
  src/LogReader.h
  src/LogReader.cpp
)

file(GLOB imgui_SRC src/imgui/*)

add_definitions(-std=c++0x)

add_definitions("-D_DATA_PATH=\"${CMAKE_CURRENT_SOURCE_DIR}/data\"")

add_executable(mc_log_visualization ${src_files} ${imgui_SRC})
PKG_CONFIG_USE_DEPENDENCY(mc_log_visualization mc_rtc)
PKG_CONFIG_USE_DEPENDENCY(mc_log_visualization glfw3)
target_link_libraries(mc_log_visualization ${OPENGL_gl_LIBRARY})

target_link_libraries(mc_log_visualization
   ${catkin_LIBRARIES}
)
