/*
 * Copyright 2016-2025 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once
#include "utils.h"
#include <mc_rtc_ros/ros.h>

namespace mc_rtc_rviz
{

struct PanelImpl
{
  PanelImpl();

#ifdef MC_RTC_ROS_IS_ROS2
  mc_rtc::NodeHandlePtr nh_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_array_pub_;
#else
  ros::NodeHandle nh_;
  ros::Publisher marker_array_pub_;
#endif
  std::shared_ptr<InteractiveMarkerServer> int_server_;
  MarkerArray marker_array_;
};

} // namespace mc_rtc_rviz
