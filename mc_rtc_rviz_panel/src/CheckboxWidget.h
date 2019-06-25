/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ClientWidget.h"

namespace mc_rtc_rviz
{
struct CheckboxWidget : public ClientWidget
{
  Q_OBJECT
public:
  CheckboxWidget(const ClientWidgetParam & param);

  void update(bool b);

private:
  QCheckBox * cbox_;
private slots:
  void toggled(bool);
};
} // namespace mc_rtc_rviz
