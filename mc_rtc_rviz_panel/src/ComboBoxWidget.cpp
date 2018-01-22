#include "ComboBoxWidget.h"

#include <iostream>

ComboBoxWidget::ComboBoxWidget(const std::string & name,
                               const mc_rtc::Configuration & data,
                               request_t request)
: request_(request)
{
  label_ = new QLabel(name.c_str());
  layout->addWidget(label_);
  values_ = data("GUI")("values");
  combo_ = new QComboBox();
  layout->addWidget(combo_);
  updateComboBox();
  void (QComboBox::*sig)(int) = &QComboBox::currentIndexChanged;
  connect(combo_, sig,
          this, [this](int index)
          {
            if(index != 0)
            {
              mc_rtc::Configuration data;
              data.add("data", combo_->currentText().toStdString());
              request_(data("data"));
            }
          });
}

void ComboBoxWidget::update(const mc_rtc::Configuration & data)
{
  auto values = data("GUI", mc_rtc::Configuration{})("values", std::vector<std::string>{});
  if(values.size() != values_.size())
  {
    values_ = values;
    updateComboBox();
  }
  else
  {
    for(const auto & v : values)
    {
      if(std::find(values_.begin(), values_.end(), v) == values_.end())
      {
        values_ = values;
        updateComboBox();
        return;
      }
    }
  }
}

void ComboBoxWidget::updateComboBox()
{
  combo_->clear();
  combo_->addItem("Select an item among the list...");
  for(const auto & v : values_)
  {
    combo_->addItem(v.c_str());
  }
}
