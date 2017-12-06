#pragma once

#include "BaseWidget.h"
#include "PointInputDialog.h"

#include <interactive_markers/interactive_marker_server.h>

struct Point3DWidget : public BaseWidget
{
  Point3DWidget(const std::string & name,
                const mc_rtc::Configuration & data,
                request_t request,
                interactive_markers::InteractiveMarkerServer & int_server_);

  virtual ~Point3DWidget() = default;

  void update(const mc_rtc::Configuration & data) override final;

  PointInputDialog * input = nullptr;
  interactive_markers::InteractiveMarkerServer & server;
  std::string marker_name_;
};
