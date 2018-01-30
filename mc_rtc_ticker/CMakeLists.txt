cmake_minimum_required(VERSION 2.8.3)
project(mc_rtc_ticker)

include(cmake/base.cmake)

find_package(catkin REQUIRED)
catkin_package()

add_required_dependency("mc_rtc")

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

add_definitions(-std=c++11)

set(MC_RTC_TICKER_SRC
      src/mc_rtc_ticker.cpp
      src/ContactForcePublisher.cpp
)

set(MC_RTC_TICKER_HDR
      src/ContactForcePublisher.h
)

add_executable(mc_rtc_ticker ${MC_RTC_TICKER_SRC} ${MC_RTC_TICKER_HDR})
pkg_config_use_dependency(mc_rtc_ticker mc_rtc)

install(TARGETS mc_rtc_ticker
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

