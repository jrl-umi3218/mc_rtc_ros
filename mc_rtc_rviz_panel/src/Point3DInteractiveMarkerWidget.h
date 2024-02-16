#pragma once
#include "TransformInteractiveMarkerWidget.h"

namespace mc_rtc_rviz
{

struct Point3DInteractiveMarkerWidget : public TransformInteractiveMarkerWidget
{
  Point3DInteractiveMarkerWidget(const ClientWidgetParam & params,
                                 const WidgetId & requestId,
                                 std::shared_ptr<InteractiveMarkerServer> & server,
                                 const mc_rtc::gui::PointConfig & config,
                                 bool control_position,
                                 ClientWidget * label);
};

} // namespace mc_rtc_rviz
