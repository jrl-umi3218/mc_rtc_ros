/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "NumberSliderWidget.h"

namespace mc_rtc_rviz
{

NumberSliderWidget::NumberSliderWidget(const ClientWidgetParam & param, double min, double max)
: ClientWidget(param), min_(min), max_(max)
{
  auto layout = new QGridLayout(this);
  if(!secret())
  {
    layout->addWidget(new QLabel(param.id.name.c_str()), 0, 0, 2, 1);
  }
  valueLabel_ = new QLabel(this);
  layout->addWidget(valueLabel_, 0, 1, Qt::AlignCenter);
  slider_ = new QSlider(Qt::Horizontal, this);
  slider_->setRange(0, 100);
  layout->addWidget(slider_, 1, 1);
  connect(slider_, SIGNAL(sliderMoved(int)), this, SLOT(sliderMoved(int)));
  connect(slider_, SIGNAL(sliderPressed()), this, SLOT(sliderPressed()));
  connect(slider_, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
}

void NumberSliderWidget::update(double data, double min, double max)
{
  if(locked_)
  {
    return;
  }
  value_ = data;
  min_ = min;
  max_ = max;
  int slide_value = std::floor(100 * (value_ - min_) / (max_ - min_));
  slider_->setValue(slide_value);
  valueLabel_->setText(QString::number(value_));
}

void NumberSliderWidget::sliderMoved(int value)
{
  double v = min_ + value * (max_ - min_) / 100;
  valueLabel_->setText(QString::number(v));
  client().send_request(id(), v);
}

void NumberSliderWidget::sliderPressed()
{
  locked_ = true;
}

void NumberSliderWidget::sliderReleased()
{
  locked_ = false;
}

} // namespace mc_rtc_rviz
