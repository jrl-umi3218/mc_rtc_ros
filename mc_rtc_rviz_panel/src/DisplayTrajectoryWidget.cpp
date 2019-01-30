#include "DisplayTrajectoryWidget.h"

#include <mc_rbdyn/configuration_io.h>

namespace mc_rtc_rviz
{

DisplayTrajectoryWidget::DisplayTrajectoryWidget(const ClientWidgetParam & params,
                                                 visualization_msgs::MarkerArray & markers,
                                                 const mc_rtc::gui::LineConfig & config)
: ClientWidget(params), markers_(markers), config_(config), visible_(visible()), was_visible_(visible_)
{
  auto layout = new QHBoxLayout(this);
  layout->addWidget(new QLabel(id().name.c_str()));
  button_ = new QPushButton(this);
  button_->setCheckable(true);
  button_->setChecked(!visible_);
  toggled(!visible_);
  layout->addWidget(button_);
}

void DisplayTrajectoryWidget::update(const std::vector<Eigen::Vector3d> & points)
{
  configure();
  path_.points.clear();
  for(const auto & p : points)
  {
    geometry_msgs::Point pose;
    pose.x = p.x();
    pose.y = p.y();
    pose.z = p.z();
    path_.points.push_back(pose);
  }
  publish();
}

void DisplayTrajectoryWidget::update(const std::vector<sva::PTransformd> & points)
{
  configure();
  path_.points.clear();
  for(const auto & p : points)
  {
    geometry_msgs::Point pose;
    pose.x = p.translation().x();
    pose.y = p.translation().y();
    pose.z = p.translation().z();
    path_.points.push_back(pose);
  }
  publish();
}

void DisplayTrajectoryWidget::update(const Eigen::Vector3d & point)
{
  configure();
  geometry_msgs::Point pose;
  pose.x = point.x();
  pose.y = point.y();
  pose.z = point.z();
  path_.points.push_back(pose);
  publish();
}

void DisplayTrajectoryWidget::update(const sva::PTransformd & point)
{
  configure();
  geometry_msgs::Point pose;
  pose.x = point.translation().x();
  pose.y = point.translation().y();
  pose.z = point.translation().z();
  path_.points.push_back(pose);
  publish();
}

void DisplayTrajectoryWidget::configure()
{
  path_.type = config_.style == mc_rtc::gui::LineStyle::Dotted ? visualization_msgs::Marker::POINTS
                                                               : visualization_msgs::Marker::LINE_STRIP;
  path_.header.frame_id = "/robot_map";
  path_.header.stamp = ros::Time::now();
  path_.scale.x = config_.width;
  path_.scale.y = config_.width;
  path_.color.r = config_.color.r;
  path_.color.g = config_.color.g;
  path_.color.b = config_.color.b;
  path_.color.a = config_.color.a;
  path_.action = visualization_msgs::Marker::ADD;
  path_.lifetime = ros::Duration(1);
  path_.ns = id2name(id());
}

void DisplayTrajectoryWidget::publish()
{
  if(visible_ || was_visible_)
  {
    markers_.markers.push_back(path_);
    if(!visible_)
    {
      markers_.markers.back().action = visualization_msgs::Marker::DELETE;
    }
  }
  was_visible_ = visible_;
}

void DisplayTrajectoryWidget::toggled(bool hide)
{
  visible_ = !hide;
  button_->setText(hide ? "Show" : "Hide");
  visible(!hide);
}

} // namespace mc_rtc_rviz
