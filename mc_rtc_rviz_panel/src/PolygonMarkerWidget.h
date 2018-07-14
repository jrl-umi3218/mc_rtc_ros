#pragma once

#include <mc_rtc/ros.h>
#include "ClientWidget.h"
#include "utils.h"

namespace mc_rtc_rviz
{

struct PolygonMarkerWidget : public ClientWidget
{
  Q_OBJECT
public:
  PolygonMarkerWidget(const ClientWidgetParam & params,
                      const WidgetId & requestId);

  void update(const std::vector<Eigen::Vector3d> & t);
private:
  ros::Publisher pub;
  WidgetId request_id_;
};

}
