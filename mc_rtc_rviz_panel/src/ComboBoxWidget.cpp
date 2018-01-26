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
            if( (has_default_item_ && index != 0) || !has_default_item_ )
            {
              mc_rtc::Configuration data;
              data.add("data", combo_->currentText().toStdString());
              request_(data("data"));
              if(has_default_item_)
              {
                combo_->removeItem(0);
                has_default_item_ = false;
              }
            }
          });
}

void ComboBoxWidget::update(const mc_rtc::Configuration & data)
{
  std::string selected = data;
  int idx = combo_->findText(selected.c_str());
  if(idx != -1)
  {
    combo_->blockSignals(true);
    combo_->setCurrentIndex(idx);
    if(has_default_item_)
    {
      combo_->removeItem(0);
      has_default_item_ = false;
    }
    combo_->blockSignals(false);
  }
  else
  {
    if(!has_default_item_)
    {
      updateComboBox();
    }
  }
}

void ComboBoxWidget::updateComboBox()
{
  combo_->clear();
  has_default_item_ = true;
  combo_->addItem("Select an item among the list...");
  for(const auto & v : values_)
  {
    combo_->addItem(v.c_str());
  }
}
