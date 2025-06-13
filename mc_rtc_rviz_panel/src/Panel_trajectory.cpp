/*
 * Copyright 2016-2025 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "DisplayTrajectoryWidget.h"
#include "Panel.h"
#include "PanelImpl.h"

namespace mc_rtc_rviz
{

void Panel::trajectory(const WidgetId & id,
                       const std::vector<Eigen::Vector3d> & points,
                       const mc_rtc::gui::LineConfig & config)
{
  Q_EMIT signal_trajectory(id, points, config);
}

void Panel::trajectory(const WidgetId & id,
                       const std::vector<sva::PTransformd> & points,
                       const mc_rtc::gui::LineConfig & config)
{
  Q_EMIT signal_trajectory(id, points, config);
}

void Panel::trajectory(const WidgetId & id, const Eigen::Vector3d & point, const mc_rtc::gui::LineConfig & config)
{
  Q_EMIT signal_trajectory(id, point, config);
}

void Panel::trajectory(const WidgetId & id, const sva::PTransformd & point, const mc_rtc::gui::LineConfig & config)
{
  Q_EMIT signal_trajectory(id, point, config);
}

void Panel::got_trajectory(const WidgetId & id,
                           const std::vector<Eigen::Vector3d> & points,
                           const mc_rtc::gui::LineConfig & config)
{
  auto & w = get_widget<DisplayTrajectoryWidget>(id, impl_->marker_array_);
  w.update(points, config);
}

void Panel::got_trajectory(const WidgetId & id,
                           const std::vector<sva::PTransformd> & points,
                           const mc_rtc::gui::LineConfig & config)
{
  auto & w = get_widget<DisplayTrajectoryWidget>(id, impl_->marker_array_);
  w.update(points, config);
}

void Panel::got_trajectory(const WidgetId & id, const Eigen::Vector3d & point, const mc_rtc::gui::LineConfig & config)
{
  auto & w = get_widget<DisplayTrajectoryWidget>(id, impl_->marker_array_);
  w.update(point, config);
}

void Panel::got_trajectory(const WidgetId & id, const sva::PTransformd & point, const mc_rtc::gui::LineConfig & config)
{
  auto & w = get_widget<DisplayTrajectoryWidget>(id, impl_->marker_array_);
  w.update(point, config);
}

} // namespace mc_rtc_rviz
