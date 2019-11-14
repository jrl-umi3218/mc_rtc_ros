/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ClientWidget.h"
#include "utils.h"

#include <mc_rtc/gui/types.h>
#include <mc_rtc/ros.h>

#include <visualization_msgs/MarkerArray.h>

namespace mc_rtc_rviz
{

struct PolygonMarkerWidget : public ClientWidget
{
  Q_OBJECT
public:
  PolygonMarkerWidget(const ClientWidgetParam & params, visualization_msgs::MarkerArray & markers);

  void update(const std::vector<std::vector<Eigen::Vector3d>> & polygons, const mc_rtc::gui::Color & c);

private:
  void update(const std::string & ns,
              const unsigned id,
              const std::vector<Eigen::Vector3d> & points,
              const mc_rtc::gui::Color & c);
  void clear();

private:
  visualization_msgs::MarkerArray & markers_;
  unsigned prevPolygonNum_ = 0;
  unsigned currPolygonNum_ = 0;
  bool visible_;
  bool was_visible_;
  QPushButton * button_;
private slots:
  void toggled(bool);
};

} // namespace mc_rtc_rviz
