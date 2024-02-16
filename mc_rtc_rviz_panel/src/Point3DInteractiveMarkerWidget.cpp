#include "Point3DInteractiveMarkerWidget.h"

namespace mc_rtc_rviz
{

Point3DInteractiveMarkerWidget::Point3DInteractiveMarkerWidget(const ClientWidgetParam & params,
                                                               const WidgetId & requestId,
                                                               std::shared_ptr<InteractiveMarkerServer> & server,
                                                               const mc_rtc::gui::PointConfig & config,
                                                               bool control_position,
                                                               ClientWidget * label)
: TransformInteractiveMarkerWidget(params,
                                   requestId,
                                   server,
                                   make3DMarker(id2name(params.id),
                                                {getPointMarker(Eigen::Vector3d::Zero(), config.color, config.scale)},
                                                control_position),
                                   false,
                                   control_position,
                                   label)

{
}

} // namespace mc_rtc_rviz
