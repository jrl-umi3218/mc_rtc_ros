/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ClientWidget.h"
#include "utils.h"

#include <mc_rtc/GUITypes.h>
#include <mc_rtc/ros.h>

#include <visualization_msgs/MarkerArray.h>

namespace mc_rtc_rviz
{

struct PointMarkerWidget : public ClientWidget
{
  Q_OBJECT
public:
  PointMarkerWidget(const ClientWidgetParam & params, visualization_msgs::MarkerArray & markers, ClientWidget * label);

  void update(const Eigen::Vector3d & pos, const mc_rtc::gui::PointConfig & c);

private:
  visualization_msgs::MarkerArray & markers_;
  QPushButton * button_;
  bool visible_ = true;
  bool was_visible_ = true;
private slots:
  void toggled(bool);
};

} // namespace mc_rtc_rviz
