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
                                         const WidgetId & requestId)
: ClientWidget(params),
  request_id_(requestId)
{
  pub = mc_rtc::ROSBridge::get_node_handle()->advertise<visualization_msgs::Marker>( "/mc_rtc_rviz/"+id2name(requestId), 0 );

}

void PolygonMarkerWidget::update(const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& c = {0., 1., 0.})
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
  m.color.a = 1.0;
  m.color.r = c.x();
  m.color.g = c.y();
  m.color.b = c.z();
  m.header.stamp = ros::Time();
  m.header.frame_id = "robot_map";
  pub.publish(m);
}

}
