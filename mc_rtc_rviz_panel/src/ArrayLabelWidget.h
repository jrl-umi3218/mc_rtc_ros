/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ArrayInputWidget.h"

namespace mc_rtc_rviz
{
struct ArrayLabelWidget : public ArrayInputWidget
{
  ArrayLabelWidget(const ClientWidgetParam & param, const std::vector<std::string> & labels);

  void update(const Eigen::VectorXd & data);

protected:
  QLabel * normLabel_ = nullptr;
};

} // namespace mc_rtc_rviz
