/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "PolygonMarkerWidget.h"

#include <mc_rbdyn/configuration_io.h>

namespace mc_rtc_rviz
{

PolygonMarkerWidget::PolygonMarkerWidget(const ClientWidgetParam & params, visualization_msgs::MarkerArray & markers)
: ClientWidget(params), markers_(markers), visible_(visible()), was_visible_(visible_)
{
  auto layout = new QHBoxLayout(this);
  if(!secret()) { layout->addWidget(new QLabel(id().name.c_str())); }
  button_ = new QPushButton(this);
  button_->setCheckable(true);
  button_->setChecked(!visible_);
  toggled(!visible_);
  connect(button_, SIGNAL(toggled(bool)), this, SLOT(toggled(bool)));
  layout->addWidget(button_);
}

PolygonMarkerWidget::~PolygonMarkerWidget()
{
  visualization_msgs::Marker m;
  m.action = visualization_msgs::Marker::DELETE;
  m.ns = id2name(id());
  for(size_t i = 0; i < currPolygonNum_; ++i)
  {
    m.id = static_cast<int>(i);
    markers_.markers.push_back(m);
  }
}

void PolygonMarkerWidget::update(const std::vector<std::vector<Eigen::Vector3d>> & polygons,
                                 const mc_rtc::gui::LineConfig & c)
{
  currPolygonNum_ = polygons.size();
  if(prevPolygonNum_ > currPolygonNum_) { clear(); }

  for(size_t i = 0; i < polygons.size(); ++i)
  {
    const auto & polygon = polygons[i];
    update(id2name(id()), i, polygon, c);
  }
  prevPolygonNum_ = polygons.size();
}

void PolygonMarkerWidget::update(const std::string & ns,
                                 const size_t id,
                                 const std::vector<Eigen::Vector3d> & points,
                                 const mc_rtc::gui::LineConfig & c)
{
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::LINE_STRIP;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.position.x = 0;
  m.pose.position.y = 0;
  m.pose.position.z = 0;
  m.pose.orientation.w = 1;
  m.pose.orientation.x = 0;
  m.pose.orientation.y = 0;
  m.pose.orientation.z = 0;
  for(const auto & point : points)
  {
    if(!is_in_range(point))
    {
      mc_rtc::log::error("Could not display polygon {}: invalid value in coordinates ({})", ns, point.transpose());
      return;
    }
    geometry_msgs::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    m.points.push_back(p);
  }
  m.points.push_back(m.points.front());
  m.scale.x = c.width;
  m.color.r = static_cast<float>(c.color.r);
  m.color.g = static_cast<float>(c.color.g);
  m.color.b = static_cast<float>(c.color.b);
  m.color.a = static_cast<float>(c.color.a);
  m.header.stamp = ros::Time::now();
  m.header.frame_id = "robot_map";
  m.ns = ns;
  m.id = static_cast<int>(id);
  if(visible_ || was_visible_)
  {
    markers_.markers.push_back(m);
    if(!visible_) { markers_.markers.back().action = visualization_msgs::Marker::DELETE; }
  }
  was_visible_ = visible_;
}

void PolygonMarkerWidget::clear()
{
  for(size_t i = currPolygonNum_; i < prevPolygonNum_; ++i)
  {
    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::DELETE;
    m.ns = id2name(id());
    m.id = static_cast<int>(i);
    markers_.markers.push_back(m);
  }
}

void PolygonMarkerWidget::toggled(bool hide)
{
  visible_ = !hide;
  button_->setText(hide ? "Show" : "Hide");
  visible(!hide);
}

} // namespace mc_rtc_rviz
