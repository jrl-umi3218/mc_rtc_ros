/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "ComboInputWidget.h"

namespace
{

std::vector<std::string> ref2values(const mc_rtc::Configuration & data, const std::vector<std::string> & ref)
{
  auto d = data;
  for(const auto & k : ref)
  {
    d = d(k, mc_rtc::Configuration{});
  }
  return d.size() ? d : std::vector<std::string>{};
}

} // namespace

namespace mc_rtc_rviz
{

ComboInputWidget::ComboInputWidget(const ClientWidgetParam & param,
                                   const mc_rtc::Configuration & data,
                                   const std::vector<std::string> & ref)
: ComboInputWidget(param, {})
{
  update(data, ref);
}

ComboInputWidget::ComboInputWidget(const ClientWidgetParam & param, const std::vector<std::string> & values)
: ClientWidget(param), values_()
{
  auto layout = new QFormLayout(this);
  combo_ = new QComboBox(this);
  layout->addRow(name().c_str(), combo_);
  update(values);
  connect(combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(currentIndexChanged(int)));
}

void ComboInputWidget::currentIndexChanged(int idx)
{
  if(idx == -1) return;
  client().send_request(id(), combo_->currentText().toStdString());
}

void ComboInputWidget::update(const std::string & data, const std::vector<std::string> & values)
{
  auto blocked = combo_->signalsBlocked();
  combo_->blockSignals(true);
  update(values);
  auto idx = combo_->findText(data.c_str());
  if(idx != -1)
  {
    combo_->setCurrentIndex(idx);
  }
  combo_->blockSignals(blocked);
}

void ComboInputWidget::update(const std::string & in,
                              const mc_rtc::Configuration & data,
                              const std::vector<std::string> & values)
{
  auto blocked = combo_->signalsBlocked();
  combo_->blockSignals(true);
  update(data, values);
  auto idx = combo_->findText(in.c_str());
  if(idx != -1)
  {
    combo_->setCurrentIndex(idx);
  }
  combo_->blockSignals(blocked);
}

void ComboInputWidget::update(const std::vector<std::string> & values)
{
  if(values != values_)
  {
    combo_->clear();
    for(const auto & v : values)
    {
      combo_->addItem(v.c_str());
    }
    values_ = values;
  }
}

void ComboInputWidget::update(const mc_rtc::Configuration & data, const std::vector<std::string> & values)
{
  update(ref2values(data, values));
}

} // namespace mc_rtc_rviz
