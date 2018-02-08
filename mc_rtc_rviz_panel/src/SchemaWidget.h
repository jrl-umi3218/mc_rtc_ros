#pragma once

#include "BaseWidget.h"

#include <QComboBox>
#include <QScrollArea>
#include <QStackedWidget>

struct SchemaWidget : public BaseWidget
{
  SchemaWidget(QWidget * parent,
               const mc_rtc::Configuration & data,
               const mc_rtc::Configuration & ctl_data,
               request_t request);

  virtual ~SchemaWidget() = default;

  void update(const mc_rtc::Configuration & data) override final {}
private:
  request_t request_;
  QComboBox * combo_;
  QScrollArea * scroll_;
  QStackedWidget * stack_;
  QWidget * current_form_ = nullptr;
  QPushButton * confirm_button_ = new QPushButton("Confirm");

  void comboCurrentIndexChanged(int);
  void confirmPushed();
};
