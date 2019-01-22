#include "PolygonMarkerWidget.h"

#include <mc_rbdyn/configuration_io.h>

namespace mc_rtc_rviz
{

PolygonMarkerWidget::PolygonMarkerWidget(const ClientWidgetParam & params,
                                         visualization_msgs::MarkerArray & markers)
: ClientWidget(params),
  markers_(markers)
{
}

void PolygonMarkerWidget::update(const std::vector<std::vector<Eigen::Vector3d>>& polygons, const mc_rtc::gui::Color& c)
{
  for (size_t i = 0; i < polygons.size(); ++i)
  {
    const auto& polygon = polygons[i];
    update(id2name(id()) + "/" + std::to_string(i), polygon, c);
  }
}

void PolygonMarkerWidget::update(const std::string& ns, const std::vector<Eigen::Vector3d>& points, const mc_rtc::gui::Color& c)
{
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::LINE_STRIP;
  m.action = visualization_msgs::Marker::ADD;
  m.lifetime = ros::Duration(1);
  for(const auto& point : points)
  {
    geometry_msgs::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    m.points.push_back(p);
  }
  m.points.push_back(m.points.front());
  m.scale.x = 0.005;
  m.color.r = c.r;
  m.color.g = c.g;
  m.color.b = c.b;
  m.color.a = c.a;
  m.header.stamp = ros::Time::now();
  m.header.frame_id = "robot_map";
  m.ns = ns;
  markers_.markers.push_back(m);
}

}
