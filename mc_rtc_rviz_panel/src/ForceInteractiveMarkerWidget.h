#pragma once

#include "ArrowInteractiveMarkerWidget.h"

namespace mc_rtc_rviz
{

struct ForceInteractiveMarkerWidget : public ArrowInteractiveMarkerWidget
{
  Q_OBJECT

public:
  ForceInteractiveMarkerWidget(const ClientWidgetParam & params,
                               const WidgetId & requestId,
                               std::shared_ptr<interactive_markers::InteractiveMarkerServer> & server,
                               const Eigen::Vector3d & start,
                               const sva::ForceVecd& force,
                               const mc_rtc::gui::ForceConfig & config,
                               bool ro,
                               ClientWidget * label);

  void update(const Eigen::Vector3d & start, const sva::ForceVecd & force, const mc_rtc::gui::ForceConfig & c);

protected:
  void handleEndRequest(const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback) override;

protected:
  mc_rtc::gui::ForceConfig config_;
  sva::ForceVecd force_;
};

} // namespace mc_rtc_rviz
