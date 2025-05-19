cmake_minimum_required(VERSION 3.1)
cmake_policy(SET CMP0048 NEW)
project(mc_rtc_ros VERSION 1.6.1)

enable_testing()

find_package(catkin)

if(${catkin_FOUND})
  add_subdirectory(mc_rtc_ticker)
endif()

add_subdirectory(mc_rtc_rviz_panel)
