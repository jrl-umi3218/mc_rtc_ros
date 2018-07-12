#pragma once

#include "ClientWidget.h"
#include "utils.h"
#include <mc_rtc/ros.h>
#include <nav_msgs/Path.h>

namespace mc_rtc_rviz
{

struct DisplayTrajectoryWidget : public ClientWidget
{
  Q_OBJECT
public:
  DisplayTrajectoryWidget(const ClientWidgetParam & params,
                          const WidgetId & requestId);

  void update(const std::vector<Eigen::Vector3d>& points);
  void update(const std::vector<sva::PTransformd>& points);
private:
  WidgetId request_id_;
  ros::Publisher path_pub;
};

}
