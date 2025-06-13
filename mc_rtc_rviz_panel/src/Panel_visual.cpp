/*
 * Copyright 2016-2025 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "Panel.h"
#include "PanelImpl.h"
#include "VisualWidget.h"

namespace mc_rtc_rviz
{

void Panel::visual(const WidgetId & id, const rbd::parsers::Visual & visual, const sva::PTransformd & pos)
{
  Q_EMIT signal_visual(id, visual, pos);
}

void Panel::got_visual(const WidgetId & id, const rbd::parsers::Visual & visual, const sva::PTransformd & pose)
{
  auto & w = get_widget<VisualWidget>(id, impl_->marker_array_);
  w.update(visual, pose);
}

} // namespace mc_rtc_rviz
