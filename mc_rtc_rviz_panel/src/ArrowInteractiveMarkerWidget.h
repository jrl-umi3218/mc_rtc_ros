#pragma once

#include "ClientWidget.h"
#include "utils.h"

#include <visualization_msgs/MarkerArray.h>

namespace mc_rtc_rviz
{

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

} // namespace mc_rtc_rviz
