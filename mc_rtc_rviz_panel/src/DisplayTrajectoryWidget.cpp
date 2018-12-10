#include "DisplayTrajectoryWidget.h"

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

DisplayTrajectoryWidget::DisplayTrajectoryWidget(const ClientWidgetParam & params,
                                                 visualization_msgs::MarkerArray & markers,
                                                 const mc_rtc::gui::LineConfig & config)
: ClientWidget(params),
  markers_(markers),
  config_(config)
{
}

void DisplayTrajectoryWidget::update(const std::vector<Eigen::Vector3d>& points)
{
  visualization_msgs::Marker path;
  configure(path);
  for(const auto & p : points)
  {
    geometry_msgs::Point pose;
    pose.x = p.x();
    pose.y = p.y();
    pose.z = p.z();
    path.points.push_back(pose);
  }
  markers_.markers.push_back(path);
}

void DisplayTrajectoryWidget::update(const std::vector<sva::PTransformd>& points)
{
  visualization_msgs::Marker path;
  configure(path);
  for(const auto&p : points)
  {
    geometry_msgs::Point pose;
    pose.x = p.translation().x();
    pose.y = p.translation().y();
    pose.z = p.translation().z();
    path.points.push_back(pose);
  }
  markers_.markers.push_back(path);
}

void DisplayTrajectoryWidget::configure(visualization_msgs::Marker & path)
{
  path.type = config_.style == mc_rtc::gui::LineStyle::Dotted ? visualization_msgs::Marker::POINTS : visualization_msgs::Marker::LINE_STRIP;
   path.header.frame_id="/robot_map";
   path.header.stamp = ros::Time::now();
   path.scale.x = config_.width;
   path.scale.y = config_.width;
   path.color.r = config_.color.r;
   path.color.g = config_.color.g;
   path.color.b = config_.color.b;
   path.color.a = config_.color.a;
   path.action = visualization_msgs::Marker::ADD;
   path.lifetime = ros::Duration(1);
   path.ns = id2name(id());
}

}
