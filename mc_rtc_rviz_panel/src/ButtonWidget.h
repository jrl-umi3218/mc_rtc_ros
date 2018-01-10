#pragma once

#include "BaseWidget.h"

struct ButtonWidget : public BaseWidget
{
  ButtonWidget(const std::string & name,
               request_t request);

  virtual ~ButtonWidget() = default;

  void update(const mc_rtc::Configuration & data) override final {}

  QPushButton * button_;
};
