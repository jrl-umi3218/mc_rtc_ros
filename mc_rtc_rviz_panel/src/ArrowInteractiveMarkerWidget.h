#pragma once

#include "ClientWidget.h"
#include "utils.h"

namespace mc_rtc_rviz
{

struct ArrowInteractiveMarkerWidget : public ClientWidget
{
  Q_OBJECT

public:
  ArrowInteractiveMarkerWidget(const ClientWidgetParam & params,
                               const WidgetId & requestId,
                               std::shared_ptr<InteractiveMarkerServer> & server,
                               MarkerArray & markers,
                               const Eigen::Vector3d & start,
                               const Eigen::Vector3d & end,
                               const mc_rtc::gui::ArrowConfig & config,
                               bool control_start,
                               bool control_end,
                               ClientWidget * label);

  void update(const Eigen::Vector3d & start, const Eigen::Vector3d & end, const mc_rtc::gui::ArrowConfig & c);

protected:
  virtual void handleStartRequest(const InteractiveMarkerFeedbackConstPtr & feedback);
  virtual void handleEndRequest(const InteractiveMarkerFeedbackConstPtr & feedback);

protected:
  WidgetId request_id_;
  MarkerArray & markers_;
  Eigen::Vector6d arrow_points_;
  SharedMarker start_marker_;
  SharedMarker end_marker_;
  Marker arrow_marker_;
  QPushButton * button_;

private slots:
  void toggled(bool);
};

} // namespace mc_rtc_rviz
