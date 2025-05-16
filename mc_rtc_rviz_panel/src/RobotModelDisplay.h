/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ClientWidget.h"

#ifdef MC_RTC_ROS_IS_ROS2
using Display = rviz_common::Display;
const std::string display_class_id = "rviz_default_plugins/RobotModel";
#else
using Display = rviz::Display;
const std::string display_class_id = "rviz/RobotModel";
#endif

namespace mc_rtc_rviz
{
/** Handle label and display mangement of Robot */
struct RobotModelDisplay : public ClientWidget
{
  RobotModelDisplay(const ClientWidgetParam & param, DisplayContext *display_context, DisplayGroup *display_group);

  void update(const std::vector<std::string> & in);

protected:
  QHBoxLayout * layout_;
  QLabel * label_;
  Display* robot_model_display = nullptr;
};
} // namespace mc_rtc_rviz
