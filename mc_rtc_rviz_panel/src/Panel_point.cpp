/*
 * Copyright 2016-2025 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "Panel.h"
#include "PanelImpl.h"
#include "Point3DInteractiveMarkerWidget.h"

namespace mc_rtc_rviz
{

void Panel::point3d(const WidgetId & id,
                    const WidgetId & requestId,
                    bool ro,
                    const Eigen::Vector3d & pos,
                    const mc_rtc::gui::PointConfig & config)
{
  Q_EMIT signal_point3d(id, requestId, ro, pos, config);
}

void Panel::got_point3d(const WidgetId & id,
                        const WidgetId & requestId,
                        bool ro,
                        const Eigen::Vector3d & pos,
                        const mc_rtc::gui::PointConfig & config)
{
  auto label = latestWidget_;
  auto & w = get_widget<Point3DInteractiveMarkerWidget>(id, requestId, impl_->int_server_, config, !ro, label);
  w.update(pos);
}

} // namespace mc_rtc_rviz
