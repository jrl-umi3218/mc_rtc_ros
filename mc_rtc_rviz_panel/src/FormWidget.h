
#pragma once

#include "BaseWidget.h"

#include <QComboBox>
#include <QScrollArea>
#include <QStackedWidget>

struct FormWidget : public BaseWidget
{
  FormWidget(QWidget * parent,
             const mc_rtc::Configuration & data,
             request_t request);

  virtual ~FormWidget() = default;

  void update(const mc_rtc::Configuration & data) override final {}
private:
  request_t request_;
  using element_callback_t = std::function<void(mc_rtc::Configuration&)>;
  std::vector<element_callback_t> element_callbacks_;
  QFormLayout * layout_;
  QPushButton * confirm_button_ = new QPushButton("Confirm");
};
