/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ClientWidget.h"

namespace mc_rtc_rviz
{
/** Handle label and display mangement of Robot */
struct RobotModelDisplay : public ClientWidget
{
  RobotModelDisplay(const ClientWidgetParam & param, rviz_common::DisplayContext *display_context, rviz_common::DisplayGroup *display_group);

  void update(const std::vector<std::string> &in);

protected:
  QHBoxLayout * layout_;
  QLabel * label_;
  rviz_common::Display* robot_model_display = nullptr;
};
} // namespace mc_rtc_rviz
