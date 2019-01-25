#pragma once

#include "ClientWidget.h"
#include "utils.h"

namespace mc_rtc_rviz
{

struct InteractiveMarkerWidget : public ClientWidget
{
  Q_OBJECT

 public:
  InteractiveMarkerWidget(const ClientWidgetParam & params,
                          const WidgetId & requestId,
                          interactive_markers::InteractiveMarkerServer & server,
                          const vm::InteractiveMarker& marker,
                          ClientWidget * label);

 protected:
  virtual void handleRequest(const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback) = 0;

protected:
  WidgetId request_id_;
  SharedMarker marker_;
  QPushButton * button_;

private slots:
  void toggled(bool);
};


struct TransformInteractiveMarkerWidget : public InteractiveMarkerWidget
{
  Q_OBJECT
public:
  TransformInteractiveMarkerWidget(const ClientWidgetParam & params,
                                   const WidgetId & requestId,
                                   interactive_markers::InteractiveMarkerServer & server,
                                   const sva::PTransformd & pos,
                                   bool control_orientation,
                                   bool control_position,
                                   ClientWidget * label);

  void handleRequest(const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback) override;

  void update(const Eigen::Vector3d & t) { marker_.update(t); }
  void update(const sva::PTransformd & pos) { marker_.update(pos); }

protected:
  bool control_orientation_;
  bool control_position_;
};



struct XYThetaInteractiveMarkerWidget : public InteractiveMarkerWidget
{
  Q_OBJECT
 public:
  XYThetaInteractiveMarkerWidget(const ClientWidgetParam & params,
                                 const WidgetId & requestId,
                                 interactive_markers::InteractiveMarkerServer & server,
                                 const sva::PTransformd & pos,
                                 bool control_orientation,
                                 bool control_position,
                                 ClientWidget * label);

  void handleRequest(const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback) override;

  // update with X, Y, theta
  void update(const Eigen::Vector3d & vec);

protected:
  bool control_orientation_;
  bool control_position_;
};

}
