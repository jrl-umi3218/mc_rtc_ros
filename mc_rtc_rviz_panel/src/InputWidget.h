#pragma once

#include "BaseWidget.h"

#include "PointInputDialog.h"

struct InputWidget : public BaseWidget
{
  InputWidget(const std::string & name,
              const mc_rtc::Configuration & data,
              request_t request);

  virtual ~InputWidget() = default;

  void update(const mc_rtc::Configuration & data) override;

  PointInputDialog * input = nullptr;
};
