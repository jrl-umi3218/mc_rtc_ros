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

DisplayTrajectoryWidget::DisplayTrajectoryWidget(const ClientWidgetParam & params)
: ClientWidget(params)
{
  path_pub = mc_rtc::ROSBridge::get_node_handle()->advertise<nav_msgs::Path>("/mc_rtc_rviz/"+id2name(params.id), 1);
}

void DisplayTrajectoryWidget::update(const std::vector<Eigen::Vector3d>& points)
{
   // Publish path
   nav_msgs::Path path;
   path.header.frame_id="/robot_map";
   path.header.stamp = ros::Time::now();
   for(const auto&p : points)
   {
     geometry_msgs::PoseStamped pose;
     pose.header.stamp = ros::Time::now();
     pose.header.frame_id = "/robot_map";
     pose.pose.position.x = p.x();
     pose.pose.position.y = p.y();
     pose.pose.position.z = p.z();
     path.poses.push_back(pose);
   }
   path_pub.publish(path);
}

void DisplayTrajectoryWidget::update(const std::vector<sva::PTransformd>& points)
{
   // Publish path
   nav_msgs::Path path;
   path.header.frame_id="/robot_map";
   path.header.stamp = ros::Time::now();
   for(const auto&p : points)
   {
     geometry_msgs::PoseStamped pose;
     pose.header.stamp = ros::Time::now();
     pose.header.frame_id = "/robot_map";
     pose.pose.position.x = p.translation().x();
     pose.pose.position.y = p.translation().y();
     pose.pose.position.z = p.translation().z();
     Eigen::Quaterniond q(p.rotation());
     Eigen::Quaterniond qi = q.inverse();
     pose.pose.orientation.w = qi.w();
     pose.pose.orientation.x = qi.x();
     pose.pose.orientation.y = qi.y();
     pose.pose.orientation.z = qi.z();
     path.poses.push_back(pose);
   }
   path_pub.publish(path);
}

}
