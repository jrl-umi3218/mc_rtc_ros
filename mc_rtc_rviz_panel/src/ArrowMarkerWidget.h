#pragma once

#include <mc_rtc/ros.h>
#include "ClientWidget.h"
#include "utils.h"
#include <mc_rtc/GUITypes.h>
#include <visualization_msgs/MarkerArray.h>

namespace mc_rtc_rviz
{

struct ArrowMarkerWidget : public ClientWidget
{
  Q_OBJECT
public:
  ArrowMarkerWidget(const ClientWidgetParam & params,
                    visualization_msgs::MarkerArray & markers);

  void update(const Eigen::Vector3d & start, const Eigen::Vector3d & end, const mc_rtc::gui::ArrowConfig & c);
private:
  visualization_msgs::MarkerArray & markers_;
  QPushButton * button_;
  bool visible_ = true;
  bool was_visible_ = true;
private slots:
  void toggled(bool);
};

}
