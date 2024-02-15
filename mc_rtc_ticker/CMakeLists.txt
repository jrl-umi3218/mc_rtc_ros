#
# Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)
set(PROJECT_NAME mc_rtc_ticker)
project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.5.0)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
