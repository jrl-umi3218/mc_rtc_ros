/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ClientWidget.h"
#include "utils.h"

#include <visualization_msgs/MarkerArray.h>

namespace mc_rtc_rviz
{

struct InteractiveMarkerWidget : public ClientWidget
{
  Q_OBJECT

public:
  InteractiveMarkerWidget(const ClientWidgetParam & params,
                          const WidgetId & requestId,
                          std::shared_ptr<interactive_markers::InteractiveMarkerServer> & server,
                          const vm::InteractiveMarker & marker,
                          ClientWidget * label);

protected:
  virtual void handleRequest(const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback) = 0;

protected:
  WidgetId request_id_;
  SharedMarker marker_;
  QPushButton * button_;
  QVBoxLayout * layout_;

private slots:
  void toggled(bool);
};

} // namespace mc_rtc_rviz
