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
  if(!secret())
  {
    layout->addWidget(new QLabel(id().name.c_str()));
  }
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
    m.id = i;
    markers_.markers.push_back(m);
  }
}

void PolygonMarkerWidget::update(const std::vector<std::vector<Eigen::Vector3d>> & polygons,
                                 const mc_rtc::gui::Color & c)
{
  currPolygonNum_ = polygons.size();
  if(prevPolygonNum_ > currPolygonNum_)
  {
    clear();
  }

  for(size_t i = 0; i < polygons.size(); ++i)
  {
    const auto & polygon = polygons[i];
    update(id2name(id()), i, polygon, c);
  }
  prevPolygonNum_ = polygons.size();
}

void PolygonMarkerWidget::update(const std::string & ns,
                                 const unsigned id,
                                 const std::vector<Eigen::Vector3d> & points,
                                 const mc_rtc::gui::Color & c)
{
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::LINE_STRIP;
  m.action = visualization_msgs::Marker::ADD;
  for(const auto & point : points)
  {
    if(!is_in_range(point))
    {
      LOG_ERROR("Could not display polygon " << ns << ": invalid value in coordinates (" << point.transpose() << ")");
      return;
    }
    geometry_msgs::Point p;
    p.x = point.x();
    p.y = point.y();
    p.z = point.z();
    m.points.push_back(p);
  }
  m.points.push_back(m.points.front());
  m.scale.x = 0.005;
  m.color.r = c.r;
  m.color.g = c.g;
  m.color.b = c.b;
  m.color.a = c.a;
  m.header.stamp = ros::Time::now();
  m.header.frame_id = "robot_map";
  m.ns = ns;
  m.id = id;
  if(visible_ || was_visible_)
  {
    markers_.markers.push_back(m);
    if(!visible_)
    {
      markers_.markers.back().action = visualization_msgs::Marker::DELETE;
    }
  }
  was_visible_ = visible_;
}

void PolygonMarkerWidget::clear()
{
  for(unsigned i = currPolygonNum_; i < prevPolygonNum_; ++i)
  {
    visualization_msgs::Marker m;
    m.action = visualization_msgs::Marker::DELETE;
    m.ns = id2name(id());
    m.id = i;
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
