#
# Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_NAME mc_rtc_ticker)
project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.6.1)

find_package(ament_cmake REQUIRED)

install(
  FILES launch/display.2.launch
  DESTINATION share/${PROJECT_NAME}/launch/
  RENAME display.launch)
install(
  FILES launch/display.2.rviz
  DESTINATION share/${PROJECT_NAME}/launch/
  RENAME display.rviz)

ament_package()
