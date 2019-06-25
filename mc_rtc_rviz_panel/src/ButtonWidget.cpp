/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "ButtonWidget.h"

namespace mc_rtc_rviz
{

ButtonWidget::ButtonWidget(const ClientWidgetParam & param) : ClientWidget(param)
{
  auto layout = new QHBoxLayout(this);
  auto button = new QPushButton(name().c_str(), this);
  layout->addWidget(button);
  connect(button, SIGNAL(released()), this, SLOT(button_released()));
}

void ButtonWidget::button_released()
{
  client().send_request(id());
}

} // namespace mc_rtc_rviz
