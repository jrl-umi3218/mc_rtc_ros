/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ClientWidget.h"
#include "utils.h"

#include <visualization_msgs/MarkerArray.h>

namespace mc_rtc_rviz
{

struct InteractiveMarkerWidget : public ClientWidget
{
  Q_OBJECT

public:
  InteractiveMarkerWidget(const ClientWidgetParam & params,
                          const WidgetId & requestId,
                          std::shared_ptr<interactive_markers::InteractiveMarkerServer> & server,
                          const vm::InteractiveMarker & marker,
                          ClientWidget * label);

protected:
  virtual void handleRequest(const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback) = 0;

protected:
  WidgetId request_id_;
  SharedMarker marker_;
  QPushButton * button_;
  QVBoxLayout * layout_;

private slots:
  void toggled(bool);
};

struct ArrowInteractiveMarkerWidget : public ClientWidget
{
  ArrowInteractiveMarkerWidget(const ClientWidgetParam & params,
                               const WidgetId & requestId,
                               std::shared_ptr<interactive_markers::InteractiveMarkerServer> & server,
                               visualization_msgs::MarkerArray & markers,
                               const mc_rtc::gui::ArrowConfig & config,
                               bool ro,
                               ClientWidget * label);

  void update(const Eigen::Vector3d & start, const Eigen::Vector3d & end, const mc_rtc::gui::ArrowConfig & c);
  void update(const Eigen::Vector3d & start, const sva::ForceVecd & force, const mc_rtc::gui::ForceConfig & c);

protected:
  void handleStartRequest(const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback);
  void handleEndRequest(const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback);

protected:
  WidgetId request_id_;
  visualization_msgs::MarkerArray & markers_;
  SharedMarker start_marker_;
  SharedMarker end_marker_;
  Eigen::Vector3d start_, end_;
};

struct TransformInteractiveMarkerWidget : public InteractiveMarkerWidget
{
  Q_OBJECT
public:
  TransformInteractiveMarkerWidget(const ClientWidgetParam & params,
                                   const WidgetId & requestId,
                                   std::shared_ptr<interactive_markers::InteractiveMarkerServer> & server,
                                   bool control_orientation,
                                   bool control_position,
                                   ClientWidget * label);

  TransformInteractiveMarkerWidget(const ClientWidgetParam & params,
                                   const WidgetId & requestId,
                                   std::shared_ptr<interactive_markers::InteractiveMarkerServer> & server,
                                   const vm::InteractiveMarker & marker,
                                   bool control_orientation,
                                   bool control_position,
                                   ClientWidget * label);

  void handleRequest(const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback) override;

  void update(const Eigen::Vector3d & t)
  {
    marker_.update(t);
  }
  void update(const sva::PTransformd & pos)
  {
    marker_.update(pos);
  }

protected:
  bool control_orientation_;
  bool control_position_;
};

struct Point3DInteractiveMarkerWidget : public TransformInteractiveMarkerWidget
{
  Point3DInteractiveMarkerWidget(const ClientWidgetParam & params,
                                 const WidgetId & requestId,
                                 std::shared_ptr<interactive_markers::InteractiveMarkerServer> & server,
                                 const mc_rtc::gui::PointConfig & config,
                                 bool control_position,
                                 ClientWidget * label);
};

struct XYThetaInteractiveMarkerWidget : public InteractiveMarkerWidget
{
  Q_OBJECT
public:
  XYThetaInteractiveMarkerWidget(const ClientWidgetParam & params,
                                 const WidgetId & requestId,
                                 std::shared_ptr<interactive_markers::InteractiveMarkerServer> & server,
                                 const sva::PTransformd & pos,
                                 bool control_orientation,
                                 bool control_position,
                                 ClientWidget * label);

  void handleRequest(const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback) override;

  // update with X, Y, theta
  void update(const Eigen::Vector3d & vec, double altitude);

private slots:
  void control_state_changed(int);

protected:
  QCheckBox * coupled_checkbox_;
  vm::InteractiveMarker coupled_marker_;
  vm::InteractiveMarker decoupled_marker_;
};

} // namespace mc_rtc_rviz
