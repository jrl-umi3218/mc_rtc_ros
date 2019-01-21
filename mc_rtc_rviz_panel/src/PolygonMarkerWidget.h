#pragma once

#include <mc_rtc/ros.h>
#include "ClientWidget.h"
#include "utils.h"
#include <mc_rtc/GUITypes.h>
#include <visualization_msgs/MarkerArray.h>

namespace mc_rtc_rviz
{

struct PolygonMarkerWidget : public ClientWidget
{
  Q_OBJECT
public:
  PolygonMarkerWidget(const ClientWidgetParam & params,
                      visualization_msgs::MarkerArray & markers);

  void update(const std::vector<std::vector<Eigen::Vector3d>>& polygons, const mc_rtc::gui::Color& c);
  void update(const std::string& ns, const std::vector<Eigen::Vector3d>& points, const mc_rtc::gui::Color& c);
private:
  visualization_msgs::MarkerArray & markers_;
};

}
