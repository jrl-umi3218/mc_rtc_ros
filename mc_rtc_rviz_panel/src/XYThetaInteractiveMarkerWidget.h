#pragma once
#include "InteractiveMarkerWidget.h"

namespace mc_rtc_rviz
{

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
