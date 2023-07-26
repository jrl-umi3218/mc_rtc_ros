/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ClientWidget.h"
#include "FormElementContainer.h"

namespace mc_rtc_rviz
{
struct FormElement;
struct FormElementHeader;

struct FormWidget;

struct FormWidget : public ClientWidget
{
  Q_OBJECT
public:
  FormWidget(const ClientWidgetParam & param);

  void update();

  inline FormElementContainer * container() noexcept { return container_; }

private:
  FormElementContainer * container_;
private slots:
  void released();
};

} // namespace mc_rtc_rviz
