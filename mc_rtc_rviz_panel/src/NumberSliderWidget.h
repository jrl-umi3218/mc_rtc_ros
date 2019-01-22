#pragma once

#include "ClientWidget.h"

namespace mc_rtc_rviz
{

struct NumberSliderWidget : public ClientWidget
{
  Q_OBJECT
public:
  NumberSliderWidget(const ClientWidgetParam & param, double min, double max);

  void update(double data);
private:
  QLabel * valueLabel_;
  QSlider * slider_;
  bool locked_ = false;
  double value_;
  double min_;
  double max_;
private slots:
  void sliderPressed();
  void sliderMoved(int);
  void sliderReleased();
};

}
