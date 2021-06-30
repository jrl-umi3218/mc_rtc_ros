/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ClientWidget.h"
#include "utils.h"

#include <visualization_msgs/MarkerArray.h>

namespace mc_rtc_rviz
{

struct VisualWidget : public ClientWidget
{
  Q_OBJECT
public:
  VisualWidget(const ClientWidgetParam & params, visualization_msgs::MarkerArray & markers);

  ~VisualWidget() override;

  void update(const rbd::parsers::Visual & visual, const sva::PTransformd & pose);

private:
  visualization_msgs::MarkerArray & markers_;
  visualization_msgs::Marker marker_;
  bool visible_;
  bool was_visible_;
  QPushButton * button_;
private slots:
  void toggled(bool);
};

} // namespace mc_rtc_rviz
