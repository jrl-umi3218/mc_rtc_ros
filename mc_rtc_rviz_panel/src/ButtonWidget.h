/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ClientWidget.h"

namespace mc_rtc_rviz
{
struct ButtonWidget : public ClientWidget
{
  Q_OBJECT
public:
  ButtonWidget(const ClientWidgetParam & param);
private slots:
  void button_released();
};
} // namespace mc_rtc_rviz
