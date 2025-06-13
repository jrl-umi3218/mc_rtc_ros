/*
 * Copyright 2016-2025 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "PanelImpl.h"

namespace mc_rtc_rviz
{
PanelImpl::PanelImpl()
{
#ifdef MC_RTC_ROS_IS_ROS2
  nh_ = rclcpp::Node::make_shared("mc_rtc_rviz_panel");
  marker_array_pub_ = nh_->create_publisher<MarkerArray>("/mc_rtc_rviz", 0);
  int_server_ = std::make_shared<InteractiveMarkerServer>("mc_rtc_rviz_interactive_markers", nh_);
#else
  int_server_ = std::make_shared<InteractiveMarkerServer>("mc_rtc_rviz_interactive_markers");
  marker_array_pub_ = nh_.advertise<MarkerArray>("/mc_rtc_rviz", 0);
#endif
}

} // namespace mc_rtc_rviz
