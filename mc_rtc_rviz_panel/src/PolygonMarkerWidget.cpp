#include "PolygonMarkerWidget.h"

#include <mc_rbdyn/configuration_io.h>

namespace mc_rtc_rviz
{

namespace
{
  std::string id2name(const WidgetId & id)
  {
    std::string ret;
    for(auto & c : id.category)
    {
      ret += c + "/";
    }
    ret += id.name;
    return ret;
  }
}

PolygonMarkerWidget::PolygonMarkerWidget(const ClientWidgetParam & params,
                                         visualization_msgs::MarkerArray& markers)
: ClientWidget(params),
  markers_(markers)
{
}

void PolygonMarkerWidget::update(const std::vector<Eigen::Vector3d>& points, const mc_rtc::gui::Color& c)
{
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::LINE_STRIP;
  m.action = visualization_msgs::Marker::ADD;
  for(const auto& point : points)
  {
    geometry_msgs::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    m.points.push_back(p);
  }
  m.scale.x = 0.005;
  m.color.r = c.r;
  m.color.g = c.g;
  m.color.b = c.b;
  m.color.a = c.a;
  m.header.stamp = ros::Time();
  m.header.frame_id = "robot_map";
  m.ns = id2name(id());
  markers_.markers.push_back(m);
}

}
