#pragma once

#include "ClientWidget.h"
#include "utils.h"

namespace mc_rtc_rviz
{

struct InteractiveMarkerWidget : public ClientWidget
{
  Q_OBJECT
public:
  InteractiveMarkerWidget(const ClientWidgetParam & params,
                          interactive_markers::InteractiveMarkerServer & server,
                          const WidgetId & requestId,
                          const sva::PTransformd & pos,
                          bool control_orientation,
                          bool control_position);

  void operator()(const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback);

  void update(const Eigen::Vector3d & t) { marker_.update(t); }
  void update(const sva::PTransformd & pos) { marker_.update(pos); }
private:
  SharedMarker marker_;
  WidgetId request_id_;
  bool control_orientation_;
  bool control_position_;
  QPushButton * button_;
private slots:
  void toggled(bool);
};

}
