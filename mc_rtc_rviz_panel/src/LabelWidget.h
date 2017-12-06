#pragma once

#include "BaseWidget.h"

struct LabelWidget : public BaseWidget
{
  LabelWidget(const std::string & name,
              const mc_rtc::Configuration & data);

  virtual ~LabelWidget() = default;

  void update(const mc_rtc::Configuration & data) override final;

  QLabel * nameLabel;
  QLabel * dataLabel;
  bool is_vector;
  // FIXME Need a tooltip thingy for vectors
};
