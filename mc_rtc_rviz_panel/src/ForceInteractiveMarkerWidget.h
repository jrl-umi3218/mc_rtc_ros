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
                               std::shared_ptr<InteractiveMarkerServer> & server,
                               const sva::PTransformd & surface,
                               const sva::ForceVecd & force,
                               const mc_rtc::gui::ForceConfig & config,
                               bool ro,
                               ClientWidget * label);

  void update(const sva::PTransformd & surface, const sva::ForceVecd & force, const mc_rtc::gui::ForceConfig & c);

protected:
  void handleEndRequest(const InteractiveMarkerFeedbackConstPtr & feedback) override;

protected:
  mc_rtc::gui::ForceConfig config_;
  sva::ForceVecd force_;
};

} // namespace mc_rtc_rviz
