/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ClientWidget.h"

namespace mc_rtc_rviz
{

struct SchemaWidget : public ClientWidget
{
  Q_OBJECT
public:
  SchemaWidget(const ClientWidgetParam & params, const std::string & schema, const mc_rtc::Configuration & data);

private:
  QStackedWidget * stack_;
private slots:
  void currentIndexChanged(int idx);
};

} // namespace mc_rtc_rviz
