#
# Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
set(PROJECT_NAME mc_rtc_ticker)
project(${PROJECT_NAME} LANGUAGES CXX VERSION 1.1.0)

find_package(catkin REQUIRED)
catkin_package()

find_package(mc_rtc 1.3.0 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

set(MC_RTC_TICKER_SRC
      src/mc_rtc_ticker.cpp
      src/ContactForcePublisher.cpp
)

set(MC_RTC_TICKER_HDR
      src/ContactForcePublisher.h
)

add_executable(mc_rtc_ticker ${MC_RTC_TICKER_SRC} ${MC_RTC_TICKER_HDR})
target_link_libraries(mc_rtc_ticker mc_rtc::mc_control mc_rtc::mc_rtc_ros)

install(TARGETS mc_rtc_ticker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
