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

struct ForceMarkerWidget : public ClientWidget
{
  Q_OBJECT
public:
  ForceMarkerWidget(const ClientWidgetParam & params,
                    const WidgetId & requestId,
                    visualization_msgs::MarkerArray & markers,
                    ClientWidget * label);

  void update(const sva::ForceVecd & force, const sva::PTransformd & surface, const mc_rtc::gui::ForceConfig & c);

private:
  WidgetId request_id_;
  visualization_msgs::MarkerArray & markers_;
  bool visible_;
  bool was_visible_;
  QPushButton * button_;
private slots:
  void toggled(bool);
};

} // namespace mc_rtc_rviz
