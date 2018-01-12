#pragma once

#include "BaseWidget.h"
#include "PointInputDialog.h"

#include <interactive_markers/interactive_marker_server.h>

struct TransformWidget : public BaseWidget
{
  TransformWidget(const std::string & name,
                  const mc_rtc::Configuration & data,
                  request_t request,
                  std::shared_ptr<interactive_markers::InteractiveMarkerServer> nt_server_);

  virtual ~TransformWidget();

  void update(const mc_rtc::Configuration & data) override final;

  PointInputDialog * input = nullptr;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  std::string marker_name_;
  visualization_msgs::InteractiveMarker marker_;
  interactive_markers::InteractiveMarkerServer::FeedbackCallback marker_cb_;
  QPushButton * visible = nullptr;
  void onVisibleToggle(bool);
};
