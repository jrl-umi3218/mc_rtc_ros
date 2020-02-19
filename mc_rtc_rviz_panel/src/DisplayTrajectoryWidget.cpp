/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

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
  if(!secret())
  {
    layout->addWidget(new QLabel(id().name.c_str()));
  }
  button_ = new QPushButton(this);
  button_->setCheckable(true);
  button_->setChecked(!visible_);
  toggled(!visible_);
  layout->addWidget(button_);
  connect(button_, SIGNAL(toggled(bool)), this, SLOT(toggled(bool)));
}

DisplayTrajectoryWidget::~DisplayTrajectoryWidget()
{
  configure();
  path_.action = visualization_msgs::Marker::DELETE;
  publish();
}

void DisplayTrajectoryWidget::update(const std::vector<Eigen::Vector3d> & points)
{
  path_.points.clear();
  for(const auto & p : points)
  {
    if(!is_in_range(p))
    {
      LOG_ERROR("Could not display trajectory " << id2name(id()) << ": invalid value in coordinates (" << p.transpose()
                                                << ")");
      return;
    }
    geometry_msgs::Point pose;
    pose.x = p.x();
    pose.y = p.y();
    pose.z = p.z();
    path_.points.push_back(pose);
  }
  configure();
  publish();
}

void DisplayTrajectoryWidget::update(const std::vector<sva::PTransformd> & points)
{
  path_.points.clear();
  for(const auto & point : points)
  {
    const auto & p = point.translation();
    if(!is_in_range(p))
    {
      LOG_ERROR("Could not display trajectory " << id2name(id()) << ": invalid value in coordinates (" << p.transpose()
                                                << ")");
      return;
    }
    geometry_msgs::Point pose;
    pose.x = p.x();
    pose.y = p.y();
    pose.z = p.z();
    path_.points.push_back(pose);
  }
  configure();
  publish();
}

void DisplayTrajectoryWidget::update(const Eigen::Vector3d & point)
{
  if(!is_in_range(point))
  {
    LOG_ERROR("Could not display trajectory " << id2name(id()) << ": invalid value in coordinates ("
                                              << point.transpose() << ")");
    return;
  }
  geometry_msgs::Point pose;
  pose.x = point.x();
  pose.y = point.y();
  pose.z = point.z();
  path_.points.push_back(pose);
  configure();
  publish();
}

void DisplayTrajectoryWidget::update(const sva::PTransformd & point)
{
  const auto & p = point.translation();
  if(!is_in_range(p))
  {
    LOG_ERROR("Could not display trajectory " << id2name(id()) << ": invalid value in coordinates (" << p.transpose()
                                              << ")");
    return;
  }
  configure();
  geometry_msgs::Point pose;
  pose.x = p.x();
  pose.y = p.y();
  pose.z = p.z();
  path_.points.push_back(pose);
  configure();
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
