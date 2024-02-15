/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "DisplayTrajectoryWidget.h"

#include <mc_rbdyn/configuration_io.h>

namespace mc_rtc_rviz
{

DisplayTrajectoryWidget::DisplayTrajectoryWidget(const ClientWidgetParam & params, MarkerArray & markers)
: ClientWidget(params), markers_(markers), visible_(visible()), was_visible_(visible_)
{
  auto layout = new QHBoxLayout(this);
  if(!secret()) { layout->addWidget(new QLabel(id().name.c_str())); }
  button_ = new QPushButton(this);
  button_->setCheckable(true);
  button_->setChecked(!visible_);
  toggled(!visible_);
  layout->addWidget(button_);
  connect(button_, SIGNAL(toggled(bool)), this, SLOT(toggled(bool)));
}

DisplayTrajectoryWidget::~DisplayTrajectoryWidget()
{
  configure({});
  path_.action = Marker::DELETE;
  publish();
}

void DisplayTrajectoryWidget::update(const std::vector<Eigen::Vector3d> & points,
                                     const mc_rtc::gui::LineConfig & config)
{
  path_.points.clear();
  for(const auto & p : points)
  {
    if(!is_in_range(p))
    {
      mc_rtc::log::error("Could not display trajectory {}: invalid value in coordinates ({})", id2name(id()),
                         p.transpose());
      return;
    }
    geometry_msgs::Point pose;
    pose.x = p.x();
    pose.y = p.y();
    pose.z = p.z();
    path_.points.push_back(pose);
  }
  configure(config);
  publish();
}

void DisplayTrajectoryWidget::update(const std::vector<sva::PTransformd> & points,
                                     const mc_rtc::gui::LineConfig & config)
{
  path_.points.clear();
  for(const auto & point : points)
  {
    const auto & p = point.translation();
    if(!is_in_range(p))
    {
      mc_rtc::log::error("Could not display trajectory {}: invalid value in coordinates ({})", id2name(id()),
                         p.transpose());
      return;
    }
    geometry_msgs::Point pose;
    pose.x = p.x();
    pose.y = p.y();
    pose.z = p.z();
    path_.points.push_back(pose);
  }
  configure(config);
  publish();
}

void DisplayTrajectoryWidget::update(const Eigen::Vector3d & point, const mc_rtc::gui::LineConfig & config)
{
  if(!is_in_range(point))
  {
    mc_rtc::log::error("Could not display trajectory {}: invalid value in coordinates ({})", id2name(id()),
                       point.transpose());
    return;
  }
  geometry_msgs::Point pose;
  pose.x = point.x();
  pose.y = point.y();
  pose.z = point.z();
  path_.points.push_back(pose);
  configure(config);
  publish();
}

void DisplayTrajectoryWidget::update(const sva::PTransformd & point, const mc_rtc::gui::LineConfig & config)
{
  const auto & p = point.translation();
  if(!is_in_range(p))
  {
    mc_rtc::log::error("Could not display trajectory {}: invalid value in coordinates ({})", id2name(id()),
                       p.transpose());
    return;
  }
  configure(config);
  geometry_msgs::Point pose;
  pose.x = p.x();
  pose.y = p.y();
  pose.z = p.z();
  path_.points.push_back(pose);
  configure(config);
  publish();
}

void DisplayTrajectoryWidget::configure(const mc_rtc::gui::LineConfig & config)
{
  path_.type = config.style == mc_rtc::gui::LineStyle::Dotted ? Marker::POINTS : Marker::LINE_STRIP;
  path_.header.frame_id = "robot_map";
  path_.header.stamp = ros::Time::now();
  path_.pose.orientation.w = 1.0;
  path_.pose.orientation.x = 0.0;
  path_.pose.orientation.y = 0.0;
  path_.pose.orientation.z = 0.0;
  path_.scale.x = config.width;
  path_.scale.y = config.width;
  path_.color.r = static_cast<float>(config.color.r);
  path_.color.g = static_cast<float>(config.color.g);
  path_.color.b = static_cast<float>(config.color.b);
  path_.color.a = static_cast<float>(config.color.a);
  path_.action = Marker::ADD;
  path_.ns = id2name(id());
}

void DisplayTrajectoryWidget::publish()
{
  if(visible_ || was_visible_)
  {
    markers_.markers.push_back(path_);
    if(!visible_ || path_.points.size() == 0) { markers_.markers.back().action = Marker::DELETE; }
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
