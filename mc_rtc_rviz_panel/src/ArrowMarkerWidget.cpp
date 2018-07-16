#include "ArrowMarkerWidget.h"

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

ArrowMarkerWidget::ArrowMarkerWidget(const ClientWidgetParam & params,
                                         const WidgetId & requestId,
                                         visualization_msgs::MarkerArray& markers)
: ClientWidget(params),
  request_id_(requestId),
  markers_(markers)
{
}

void ArrowMarkerWidget::update(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const mc_rtc::gui::Arrow& c)
{
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    auto rosPoint = [](const Eigen::Vector3d & vec)
    {
      geometry_msgs::Point p;
      p.x = vec.x();
      p.y = vec.y();
      p.z = vec.z();
      return p;
    };
    m.points.push_back(rosPoint(start));
    m.points.push_back(rosPoint(end));
    m.scale.x = c.arrow_shaft_diam;
    m.scale.y = c.arrow_head_diam;
    m.scale.z = c.arrow_head_len;
    m.color.a = c.color.a;
    m.color.r = c.color.r;
    m.color.g = c.color.g;
    m.color.b = c.color.b;
    m.header.stamp = ros::Time();
    m.header.frame_id = "robot_map";
    m.ns = id2name(request_id_);
    markers_.markers.push_back(m);
}

}
