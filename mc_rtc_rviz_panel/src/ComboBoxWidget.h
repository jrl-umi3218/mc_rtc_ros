#pragma once

#include "BaseWidget.h"

#include <QLabel>
#include <QComboBox>

struct ComboBoxWidget : public BaseWidget
{
  ComboBoxWidget(const std::string & name,
                 const mc_rtc::Configuration & data,
                 request_t request);

  virtual ~ComboBoxWidget() = default;

  void update(const mc_rtc::Configuration & data) override;
private:
  QLabel * label_ = nullptr;
  QComboBox * combo_ = nullptr;
  std::vector<std::string> values_;
  request_t request_;
private:
  void updateComboBox();
};
