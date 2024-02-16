/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ClientWidget.h"
#include "utils.h"

namespace mc_rtc_rviz
{

struct VisualWidget : public ClientWidget
{
  Q_OBJECT
public:
  VisualWidget(const ClientWidgetParam & params, MarkerArray & markers);

  ~VisualWidget() override;

  void update(const rbd::parsers::Visual & visual, const sva::PTransformd & pose);

private:
  MarkerArray & markers_;
  Marker marker_;
  bool visible_;
  bool was_visible_;
  QPushButton * button_;
private slots:
  void toggled(bool);
};

} // namespace mc_rtc_rviz
