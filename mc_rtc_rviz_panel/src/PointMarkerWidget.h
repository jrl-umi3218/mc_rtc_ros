#pragma once

#include <mc_rtc/ros.h>
#include "ClientWidget.h"
#include "utils.h"
#include <mc_rtc/GUITypes.h>
#include <visualization_msgs/MarkerArray.h>

namespace mc_rtc_rviz
{

struct PointMarkerWidget : public ClientWidget
{
  Q_OBJECT
public:
  PointMarkerWidget(const ClientWidgetParam & params,
                    visualization_msgs::MarkerArray & markers);

  void update(const Eigen::Vector3d & pos, const mc_rtc::gui::PointConfig & c);

private:
  visualization_msgs::MarkerArray & markers_;
};

}
