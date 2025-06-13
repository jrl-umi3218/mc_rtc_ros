/*
 * Copyright 2016-2025 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "ForceInteractiveMarkerWidget.h"
#include "Panel.h"
#include "PanelImpl.h"

namespace mc_rtc_rviz
{

void Panel::force(const WidgetId & id,
                  const WidgetId & requestId,
                  const sva::ForceVecd & force_,
                  const sva::PTransformd & start,
                  const mc_rtc::gui::ForceConfig & forceConfig,
                  bool ro)
{
  Q_EMIT signal_force(id, requestId, force_, start, forceConfig, ro);
}

void Panel::arrow(const WidgetId & id,
                  const WidgetId & requestId,
                  const Eigen::Vector3d & start,
                  const Eigen::Vector3d & end,
                  const mc_rtc::gui::ArrowConfig & config,
                  bool ro)
{
  Q_EMIT signal_arrow(id, requestId, start, end, config, ro);
}

void Panel::got_force(const WidgetId & id,
                      const WidgetId & requestId,
                      const sva::ForceVecd & forcep,
                      const sva::PTransformd & surface,
                      const mc_rtc::gui::ForceConfig & forceConfig,
                      bool ro)
{
  auto label = latestWidget_;
  auto & w = get_widget<ForceInteractiveMarkerWidget>(id, requestId, impl_->int_server_, impl_->marker_array_, surface,
                                                      forcep, forceConfig, ro, label);
  w.update(surface, forcep, forceConfig);
}

void Panel::got_arrow(const WidgetId & id,
                      const WidgetId & requestId,
                      const Eigen::Vector3d & start,
                      const Eigen::Vector3d & end,
                      const mc_rtc::gui::ArrowConfig & config,
                      bool ro)
{
  auto label = latestWidget_;
  auto & w = get_widget<ArrowInteractiveMarkerWidget>(id, requestId, impl_->int_server_, impl_->marker_array_, start,
                                                      end, config, !ro, !ro, label);
  w.update(start, end, config);
}

} // namespace mc_rtc_rviz
