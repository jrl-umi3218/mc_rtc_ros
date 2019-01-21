#include "ArrowMarkerWidget.h"

#include <mc_rbdyn/configuration_io.h>
#include "utils.h"

namespace mc_rtc_rviz
{

ArrowMarkerWidget::ArrowMarkerWidget(const ClientWidgetParam & params,
                                         visualization_msgs::MarkerArray & markers)
: ClientWidget(params),
  markers_(markers)
{
}

void ArrowMarkerWidget::update(const Eigen::Vector3d & start, const Eigen::Vector3d & end, const mc_rtc::gui::ArrowConfig & c)
{
    visualization_msgs::Marker m;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(1);
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
    m.header.stamp = ros::Time::now();
    m.header.frame_id = "robot_map";
    m.ns = id2name(id());
    markers_.markers.push_back(m);

    if(c.arrow_start_point_scale > 0)
    {
      markers_.markers.push_back(getPointMarker(id2name(id()) + "_start_point", start, c.color, c.arrow_start_point_scale));
    }

    if(c.arrow_end_point_scale > 0)
    {
      markers_.markers.push_back(getPointMarker(id2name(id()) + "_end_point", end, c.color, c.arrow_end_point_scale));
    }
}

}
