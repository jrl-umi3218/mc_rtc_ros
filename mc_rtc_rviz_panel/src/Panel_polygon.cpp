/*
 * Copyright 2016-2025 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "Panel.h"
#include "PanelImpl.h"
#include "PolygonMarkerWidget.h"

namespace mc_rtc_rviz
{

void Panel::polygon(const WidgetId & id,
                    const std::vector<std::vector<Eigen::Vector3d>> & polygons,
                    const mc_rtc::gui::Color & color)
{
  polygon(id, polygons, mc_rtc::gui::LineConfig{color});
}

void Panel::polygon(const WidgetId & id,
                    const std::vector<std::vector<Eigen::Vector3d>> & polygons,
                    const mc_rtc::gui::LineConfig & config)
{
  Q_EMIT signal_polygon(id, polygons, config);
}

void Panel::got_polygon(const WidgetId & id,
                        const std::vector<std::vector<Eigen::Vector3d>> & polygons,
                        const mc_rtc::gui::LineConfig & c)
{
  auto & w = get_widget<PolygonMarkerWidget>(id, impl_->marker_array_);
  w.update(polygons, c);
}

} // namespace mc_rtc_rviz
