/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "PolyhedronMarkerWidget.h"

#include <mc_rbdyn/configuration_io.h>

namespace mc_rtc_rviz
{

PolyhedronMarkerWidget::PolyhedronMarkerWidget(const ClientWidgetParam & params,
                                               visualization_msgs::MarkerArray & markers)
: ClientWidget(params), markers_(markers), visible_(visible()), was_visible_triangles_(visible_),
  was_visible_edges_(visible_), was_visible_vertices_(visible_)
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

  show_triangles_ = new QCheckBox("Triangles");
  show_triangles_->setChecked(was_visible_triangles_);
  show_edges_ = new QCheckBox("Edges");
  show_edges_->setChecked(was_visible_edges_);
  show_vertices_ = new QCheckBox("Vertices");
  show_vertices_->setChecked(was_visible_vertices_);
  layout->addWidget(show_triangles_);
  layout->addWidget(show_edges_);
  layout->addWidget(show_vertices_);

  connect(show_triangles_, SIGNAL(stateChanged(int)), this, SLOT(show_triangles_changed(int)));
  connect(show_edges_, SIGNAL(stateChanged(int)), this, SLOT(show_edges_changed(int)));
  connect(show_vertices_, SIGNAL(stateChanged(int)), this, SLOT(show_vertices_changed(int)));

  triangles_.type = visualization_msgs::Marker::TRIANGLE_LIST;
  triangles_.action = visualization_msgs::Marker::ADD;
  triangles_.header.frame_id = "robot_map";
  triangles_.id = 0;
  triangles_.scale.x = 1;
  triangles_.scale.y = 1;
  triangles_.scale.z = 1;
  triangles_.pose.position.x = 0;
  triangles_.pose.position.y = 0;
  triangles_.pose.position.z = 0;
  triangles_.pose.orientation.w = 1;
  triangles_.pose.orientation.x = 0;
  triangles_.pose.orientation.y = 0;
  triangles_.pose.orientation.z = 0;
  triangles_.color.r = 0;
  triangles_.color.g = 1;
  triangles_.color.b = 0;
  triangles_.color.a = 1;

  edges_ = triangles_;
  edges_.type = visualization_msgs::Marker::LINE_LIST;
  edges_.scale.x = 0.01;
  edges_.scale.y = 0;
  edges_.scale.z = 0;
  edges_.color.r = 0;
  edges_.color.g = 0;
  edges_.color.b = 0;
  edges_.color.a = 1;

  vertices_ = triangles_;
  vertices_.type = visualization_msgs::Marker::SPHERE_LIST;
  vertices_.id = 0;
  vertices_.scale.x = 0.02;
  vertices_.scale.y = 0.02;
  vertices_.scale.z = 0.02;
}

PolyhedronMarkerWidget::~PolyhedronMarkerWidget()
{
  clear();
}

void PolyhedronMarkerWidget::update_triangles(const std::string & ns,
                                              const std::vector<Eigen::Vector3d> & triangles,
                                              const std::vector<Eigen::Vector4d> & colors,
                                              const mc_rtc::gui::PolyhedronConfig & c)
{
  triangles_.ns = ns + "_triangles";
  triangles_.type = visualization_msgs::Marker::TRIANGLE_LIST;
  triangles_.action = visualization_msgs::Marker::ADD;
  triangles_.header.stamp = ros::Time::now();
  triangles_.color.r = c.triangle_color.r;
  triangles_.color.g = c.triangle_color.g;
  triangles_.color.b = c.triangle_color.b;
  triangles_.color.a = c.triangle_color.a;
  auto set_triangles = [&]() {
    for(const auto & point : triangles)
    {
      if(!is_in_range(point))
      {
        mc_rtc::log::error("Could not display point {}: invalid value in coordinates ({})", ns, point.transpose());
        clear();
        return;
      }
      geometry_msgs::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      triangles_.points.push_back(p);
    }
  };
  auto set_colors = [&]() {
    for(const auto & color : colors)
    {
      std_msgs::ColorRGBA c;
      c.r = color.x();
      c.g = color.y();
      c.b = color.z();
      c.a = color[3];
      triangles_.colors.push_back(c);
    }
  };
  set_triangles();
  if(!c.use_triangle_color && colors.size() && colors.size() == triangles.size())
  {
    set_colors();
  }
  if(visible_)
  {
    if(c.show_triangle || visible_triangles_)
    {
      markers_.markers.push_back(triangles_);
    }
    else if(was_visible_triangles_)
    {
      markers_.markers.push_back(triangles_);
      markers_.markers.back().action = visualization_msgs::Marker::DELETE;
    }
  }
  else if(was_visible_triangles_)
  {
    markers_.markers.push_back(triangles_);
    markers_.markers.back().action = visualization_msgs::Marker::DELETE;
  }
  was_visible_triangles_ = visible_ && visible_triangles_;
}

void PolyhedronMarkerWidget::update_edges(const std::string & ns,
                                          const std::vector<Eigen::Vector3d> & triangles,
                                          const std::vector<Eigen::Vector4d> & colors,
                                          const mc_rtc::gui::PolyhedronConfig & c)
{
  edges_.type = visualization_msgs::Marker::LINE_STRIP;
  edges_.action = visualization_msgs::Marker::ADD;
  edges_.header.stamp = ros::Time::now();
  edges_.ns = ns + "_edges";
  edges_.id = 0;
  edges_.scale.x = 0.02;
  edges_.scale.y = 0.0;
  edges_.scale.z = 0.0;
  edges_.color.r = 0;
  edges_.color.g = 0;
  edges_.color.b = 0;
  edges_.color.a = 1;
  edges_.points = triangles_.points;

  if(c.fixed_edge_color)
  {
    for(auto & color : edges_.colors)
    {
      color.r = 0;
      color.g = 0;
      color.b = 0;
      color.a = 1;
    }
  }
  else
  {
    edges_.colors = triangles_.colors;
  }

  if(visible_)
  {
    if(c.show_triangle || visible_edges_)
    {
      markers_.markers.push_back(edges_);
    }
    else if(was_visible_edges_)
    {
      markers_.markers.push_back(edges_);
      markers_.markers.back().action = visualization_msgs::Marker::DELETE;
    }
  }
  else if(was_visible_edges_)
  {
    markers_.markers.push_back(edges_);
    markers_.markers.back().action = visualization_msgs::Marker::DELETE;
  }
  was_visible_edges_ = visible_ && visible_edges_;
}

void PolyhedronMarkerWidget::update_vertices(const std::string & ns,
                                             const std::vector<Eigen::Vector3d> & triangles,
                                             const std::vector<Eigen::Vector4d> & colors,
                                             const mc_rtc::gui::PolyhedronConfig & c)
{
  vertices_.type = visualization_msgs::Marker::SPHERE_LIST;
  vertices_.action = visualization_msgs::Marker::ADD;
  vertices_.header.stamp = ros::Time::now();
  vertices_.ns = ns + "_vertices";
  vertices_.id = 0;
  vertices_.scale.x = 0.05;
  vertices_.scale.y = 0.05;
  vertices_.scale.z = 0.05;
  vertices_.color.r = 0;
  vertices_.color.g = 0;
  vertices_.color.b = 0;
  vertices_.color.a = 1;
  vertices_.points = triangles_.points;

  bool fixed_color = false;
  if(fixed_color)
  {
    for(auto & color : vertices_.colors)
    {
      color.r = 0;
      color.g = 0;
      color.b = 0;
      color.a = 1;
    }
  }
  else
  {
    vertices_.colors = triangles_.colors;
  }

  if(visible_)
  {
    if(c.show_triangle || visible_vertices_)
    {
      markers_.markers.push_back(vertices_);
    }
    else if(was_visible_vertices_)
    {
      markers_.markers.push_back(vertices_);
      markers_.markers.back().action = visualization_msgs::Marker::DELETE;
    }
  }
  else if(was_visible_vertices_)
  {
    markers_.markers.push_back(vertices_);
    markers_.markers.back().action = visualization_msgs::Marker::DELETE;
  }
  was_visible_vertices_ = visible_ && visible_vertices_;
}

void PolyhedronMarkerWidget::update(const std::vector<Eigen::Vector3d> & triangles,
                                    const std::vector<Eigen::Vector4d> & colors,
                                    const mc_rtc::gui::PolyhedronConfig & c)
{
  currPolyhedronNum_ = triangles.size();
  if(prevPolyhedronNum_ > currPolyhedronNum_)
  {
    clear();
  }

  update(id2name(id()), triangles, colors, c);
  prevPolyhedronNum_ = triangles.size();
}

void PolyhedronMarkerWidget::update(const std::string & ns,
                                    const std::vector<Eigen::Vector3d> & triangles,
                                    const std::vector<Eigen::Vector4d> & colors,
                                    const mc_rtc::gui::PolyhedronConfig & c)
{
  update_triangles(ns, triangles, colors, c);
  update_edges(ns, triangles, colors, c);
  update_vertices(ns, triangles, colors, c);
}

void PolyhedronMarkerWidget::clear()
{
  triangles_.action = visualization_msgs::Marker::DELETE;
  markers_.markers.push_back(triangles_);

  edges_.action = visualization_msgs::Marker::DELETE;
  markers_.markers.push_back(edges_);

  vertices_.action = visualization_msgs::Marker::DELETE;
  markers_.markers.push_back(vertices_);
}

void PolyhedronMarkerWidget::toggled(bool hide)
{
  visible_ = !hide;
  button_->setText(hide ? "Show" : "Hide");
  visible(!hide);
}

void PolyhedronMarkerWidget::show_triangles_changed(int)
{
  visible_triangles_ = show_triangles_->isChecked();
}

void PolyhedronMarkerWidget::show_edges_changed(int)
{
  visible_edges_ = show_edges_->isChecked();
}

void PolyhedronMarkerWidget::show_vertices_changed(int)
{
  visible_vertices_ = show_vertices_->isChecked();
}

} // namespace mc_rtc_rviz
