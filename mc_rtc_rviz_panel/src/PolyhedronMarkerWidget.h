/*
 * Copyright 2022 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ClientWidget.h"
#include "utils.h"

#include <mc_rtc/gui/types.h>
#include <mc_rtc/ros.h>

#include <visualization_msgs/MarkerArray.h>

namespace mc_rtc_rviz
{

struct PolyhedronMarkerWidget : public ClientWidget
{
  Q_OBJECT
public:
  PolyhedronMarkerWidget(const ClientWidgetParam & params, visualization_msgs::MarkerArray & markers, const mc_rtc::gui::PolyhedronConfig & config);

  void update(const std::vector<Eigen::Vector3d> & triangles,
              const std::vector<Eigen::Vector4d> & colors);

  ~PolyhedronMarkerWidget() override;

private:
  void update_triangles(const std::vector<Eigen::Vector3d> & triangles,
                        const std::vector<Eigen::Vector4d> & colors);
  void update_edges(const std::vector<Eigen::Vector3d> & triangles,
                    const std::vector<Eigen::Vector4d> & colors);
  void update_vertices(const std::vector<Eigen::Vector3d> & triangles,
                       const std::vector<Eigen::Vector4d> & colors);
  void clear();

private slots:
  void show_triangles_changed(int);
  void show_edges_changed(int);
  void show_vertices_changed(int);

private:
  visualization_msgs::MarkerArray & markers_;
  mc_rtc::gui::PolyhedronConfig config_;
  QCheckBox * show_triangles_ = nullptr;
  QCheckBox * show_edges_ = nullptr;
  QCheckBox * show_vertices_ = nullptr;
  visualization_msgs::Marker triangles_; // Triangles displayed as a TRIANGLE_LIST
  visualization_msgs::Marker edges_; // Edges displayed as a LINE_STRIP
  visualization_msgs::Marker vertices_; // Vertices displayed as a SPHERE_LIST
  size_t prevPolyhedronNum_ = 0;
  size_t currPolyhedronNum_ = 0;
  bool visible_ = true;
  bool visible_triangles_ = true;
  bool visible_edges_ = true;
  bool visible_vertices_ = true;
  bool was_visible_triangles_ = false;
  bool was_visible_edges_ = false;
  bool was_visible_vertices_ = false;
  QPushButton * button_;
private slots:
  void toggled(bool);
};

} // namespace mc_rtc_rviz
