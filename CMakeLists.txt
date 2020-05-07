cmake_minimum_required(VERSION 3.1)
cmake_policy(SET CMP0048 NEW)
project(mc_rtc_ros VERSION 1.1.0)

enable_testing()

find_package(catkin)

if(${catkin_FOUND})
  add_subdirectory(mc_log_visualization)
  add_subdirectory(mc_convex_visualization)
  add_subdirectory(mc_rtc_ticker)
  add_subdirectory(mc_surfaces_visualization)
endif()

add_subdirectory(mc_rtc_rviz_panel)
