/*
 * Copyright 2016-2025 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "Panel.h"
#include "PanelImpl.h"
#include "PolyhedronMarkerWidget.h"

namespace mc_rtc_rviz
{

void Panel::polyhedron(const WidgetId & id,
                       const std::vector<std::array<Eigen::Vector3d, 3>> & triangles,
                       const std::vector<std::array<mc_rtc::gui::Color, 3>> & colors,
                       const mc_rtc::gui::PolyhedronConfig & c)
{
  Q_EMIT signal_polyhedron(id, triangles, colors, c);
}

void Panel::got_polyhedron(const WidgetId & id,
                           const std::vector<std::array<Eigen::Vector3d, 3>> & triangles,
                           const std::vector<std::array<mc_rtc::gui::Color, 3>> & colors,
                           const mc_rtc::gui::PolyhedronConfig & c)
{
  auto & w = get_widget<PolyhedronMarkerWidget>(id, impl_->marker_array_, c);
  w.update(triangles, colors);
}

} // namespace mc_rtc_rviz
