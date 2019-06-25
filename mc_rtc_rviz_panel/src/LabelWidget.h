/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ClientWidget.h"

namespace mc_rtc_rviz
{
struct LabelWidget : public ClientWidget
{
  LabelWidget(const ClientWidgetParam & param);

  void update(const std::string & in);

protected:
  QHBoxLayout * layout_;
  QLabel * label_;
};
} // namespace mc_rtc_rviz
