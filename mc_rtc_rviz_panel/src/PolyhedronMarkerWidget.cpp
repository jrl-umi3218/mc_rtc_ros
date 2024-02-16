/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "PolyhedronMarkerWidget.h"

#include <mc_rbdyn/configuration_io.h>

namespace mc_rtc_rviz
{

PolyhedronMarkerWidget::PolyhedronMarkerWidget(const ClientWidgetParam & params,
                                               MarkerArray & markers,
                                               const mc_rtc::gui::PolyhedronConfig & config)
: ClientWidget(params), markers_(markers), config_(config), visible_(visible()),
  visible_triangles_(config.show_triangle), visible_edges_(config.show_edges), visible_vertices_(config.show_vertices)
{
  auto layout = new QHBoxLayout(this);
  if(!secret()) { layout->addWidget(new QLabel(id().name.c_str())); }
  button_ = new QPushButton(this);
  button_->setCheckable(true);
  button_->setChecked(!visible_);
  toggled(!visible_);
  connect(button_, SIGNAL(toggled(bool)), this, SLOT(toggled(bool)));
  layout->addWidget(button_);

  show_triangles_ = new QCheckBox("Triangles");
  show_triangles_->setChecked(visible_triangles_);
  show_edges_ = new QCheckBox("Edges");
  show_edges_->setChecked(visible_edges_);
  show_vertices_ = new QCheckBox("Vertices");
  show_vertices_->setChecked(visible_vertices_);
  layout->addWidget(show_triangles_);
  layout->addWidget(show_edges_);
  layout->addWidget(show_vertices_);

  connect(show_triangles_, SIGNAL(stateChanged(int)), this, SLOT(show_triangles_changed(int)));
  connect(show_edges_, SIGNAL(stateChanged(int)), this, SLOT(show_edges_changed(int)));
  connect(show_vertices_, SIGNAL(stateChanged(int)), this, SLOT(show_vertices_changed(int)));

  triangles_.type = Marker::TRIANGLE_LIST;
  triangles_.action = Marker::ADD;
  triangles_.header.frame_id = "robot_map";
  triangles_.ns = id2name(id()) + "_triangles";
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
  triangles_.color.r = config_.triangle_color.r;
  triangles_.color.g = config_.triangle_color.g;
  triangles_.color.b = config_.triangle_color.b;
  triangles_.color.a = config_.triangle_color.a;

  edges_ = triangles_;
  edges_.ns = id2name(id()) + "_edges";
  edges_.type = Marker::LINE_LIST;
  edges_.scale.x = config_.edge_config.width;
  edges_.scale.y = config_.edge_config.width;
  edges_.scale.z = config_.edge_config.width;
  edges_.color.r = config_.edge_config.color.r;
  edges_.color.g = config_.edge_config.color.g;
  edges_.color.b = config_.edge_config.color.b;
  edges_.color.a = config_.edge_config.color.a;

  vertices_ = triangles_;
  vertices_.ns = id2name(id()) + "_vertices";
  vertices_.type = Marker::SPHERE_LIST;
  vertices_.id = 0;
  vertices_.scale.x = config_.vertices_config.scale;
  vertices_.scale.y = config_.vertices_config.scale;
  vertices_.scale.z = config_.vertices_config.scale;
  vertices_.color.r = config_.vertices_config.color.r;
  vertices_.color.g = config_.vertices_config.color.g;
  vertices_.color.b = config_.vertices_config.color.b;
  vertices_.color.a = config_.vertices_config.color.a;
}

PolyhedronMarkerWidget::~PolyhedronMarkerWidget()
{
  clear();
}

void PolyhedronMarkerWidget::update_triangles(const std::vector<std::array<Eigen::Vector3d, 3>> & triangles,
                                              const std::vector<std::array<mc_rtc::gui::Color, 3>> & colors)
{
  triangles_.points.clear();
  triangles_.colors.clear();
  triangles_.type = Marker::TRIANGLE_LIST;
  triangles_.action = Marker::ADD;
  triangles_.header.stamp = now();
  auto set_triangles = [&]()
  {
    for(const auto & triangle : triangles)
    {
      for(const auto & point : triangle)
      {
        if(!is_in_range(point))
        {
          mc_rtc::log::error("Could not display point {}: invalid value in coordinates ({})", triangles_.ns,
                             point.transpose());
          clear();
          return;
        }
        Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = point.z();
        triangles_.points.push_back(p);
      }
    }
  };
  auto set_colors = [&]()
  {
    for(const auto & triangle_colors : colors)
    {
      for(const auto & color : triangle_colors)
      {
        ColorRGBA c;
        c.r = color.r;
        c.g = color.g;
        c.b = color.b;
        c.a = color.a;
        triangles_.colors.push_back(c);
      }
    }
  };
  set_triangles();
  if(!config_.use_triangle_color && colors.size() && colors.size() == triangles.size()) { set_colors(); }
  if(visible_)
  {
    if(visible_triangles_) { markers_.markers.push_back(triangles_); }
    else if(was_visible_triangles_)
    {
      markers_.markers.push_back(triangles_);
      markers_.markers.back().action = Marker::DELETE;
    }
  }
  else if(was_visible_triangles_)
  {
    markers_.markers.push_back(triangles_);
    markers_.markers.back().action = Marker::DELETE;
  }
  was_visible_triangles_ = visible_ && visible_triangles_;
}

void PolyhedronMarkerWidget::update_edges(const std::vector<std::array<Eigen::Vector3d, 3>> & triangles,
                                          const std::vector<std::array<mc_rtc::gui::Color, 3>> & colors)
{
  edges_.points.clear();
  edges_.colors.clear();
  edges_.type = Marker::LINE_STRIP;
  edges_.action = Marker::ADD;
  edges_.header.stamp = now();
  edges_.points = triangles_.points;

  if(!config_.fixed_edge_color) { edges_.colors = triangles_.colors; }

  if(visible_)
  {
    if(visible_edges_) { markers_.markers.push_back(edges_); }
    else if(was_visible_edges_)
    {
      markers_.markers.push_back(edges_);
      markers_.markers.back().action = Marker::DELETE;
    }
  }
  else if(was_visible_edges_)
  {
    markers_.markers.push_back(edges_);
    markers_.markers.back().action = Marker::DELETE;
  }
  was_visible_edges_ = visible_ && visible_edges_;
}

void PolyhedronMarkerWidget::update_vertices(const std::vector<std::array<Eigen::Vector3d, 3>> & triangles,
                                             const std::vector<std::array<mc_rtc::gui::Color, 3>> & colors)
{
  vertices_.points.clear();
  vertices_.colors.clear();
  vertices_.type = Marker::SPHERE_LIST;
  vertices_.action = Marker::ADD;
  vertices_.header.stamp = now();
  vertices_.id = 0;
  vertices_.points = triangles_.points;

  if(!config_.fixed_vertices_color) { vertices_.colors = triangles_.colors; }

  if(visible_)
  {
    if(visible_vertices_) { markers_.markers.push_back(vertices_); }
    else if(was_visible_vertices_)
    {
      markers_.markers.push_back(vertices_);
      markers_.markers.back().action = Marker::DELETE;
    }
  }
  else if(was_visible_vertices_)
  {
    markers_.markers.push_back(vertices_);
    markers_.markers.back().action = Marker::DELETE;
  }
  was_visible_vertices_ = visible_ && visible_vertices_;
}

void PolyhedronMarkerWidget::update(const std::vector<std::array<Eigen::Vector3d, 3>> & triangles,
                                    const std::vector<std::array<mc_rtc::gui::Color, 3>> & colors)
{
  currPolyhedronNum_ = triangles.size();
  if(prevPolyhedronNum_ > currPolyhedronNum_) { clear(); }

  update_triangles(triangles, colors);
  update_edges(triangles, colors);
  update_vertices(triangles, colors);
  prevPolyhedronNum_ = triangles.size();
}

void PolyhedronMarkerWidget::clear()
{
  triangles_.action = Marker::DELETE;
  markers_.markers.push_back(triangles_);

  edges_.action = Marker::DELETE;
  markers_.markers.push_back(edges_);

  vertices_.action = Marker::DELETE;
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
