/*
 * Copyright 2016-2025 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "Panel.h"
#include "PanelImpl.h"
#include "TransformInteractiveMarkerWidget.h"
#include "XYThetaInteractiveMarkerWidget.h"

namespace mc_rtc_rviz
{

void Panel::rotation(const WidgetId & id, const WidgetId & requestId, bool ro, const sva::PTransformd & pos)
{
  Q_EMIT signal_rotation(id, requestId, ro, pos);
}

void Panel::transform(const WidgetId & id, const WidgetId & requestId, bool ro, const sva::PTransformd & pos)
{
  Q_EMIT signal_transform(id, requestId, ro, pos);
}

void Panel::xytheta(const WidgetId & id,
                    const WidgetId & requestId,
                    bool ro,
                    const Eigen::Vector3d & vec,
                    double altitude)
{
  Q_EMIT signal_xytheta(id, requestId, ro, vec, altitude);
}

void Panel::got_rotation(const WidgetId & id, const WidgetId & requestId, bool ro, const sva::PTransformd & pos)
{
  auto label = latestWidget_;
  auto & w = get_widget<TransformInteractiveMarkerWidget>(id, requestId, impl_->int_server_, !ro, false, label);
  w.update(pos);
}

void Panel::got_transform(const WidgetId & id, const WidgetId & requestId, bool ro, const sva::PTransformd & pos)
{
  auto label = latestWidget_;
  auto & w = get_widget<TransformInteractiveMarkerWidget>(id, requestId, impl_->int_server_, !ro, !ro, label);
  w.update(pos);
}

void Panel::got_xytheta(const WidgetId & id,
                        const WidgetId & requestId,
                        bool ro,
                        const Eigen::Vector3d & vec,
                        double altitude)
{
  auto label = latestWidget_;
  auto & w = get_widget<XYThetaInteractiveMarkerWidget>(id, requestId, impl_->int_server_, sva::PTransformd::Identity(),
                                                        !ro, !ro, label);
  w.update(vec, altitude);
}

} // namespace mc_rtc_rviz
