#pragma once
#include "InteractiveMarkerWidget.h"

namespace mc_rtc_rviz
{

struct TransformInteractiveMarkerWidget : public InteractiveMarkerWidget
{
  Q_OBJECT
public:
  TransformInteractiveMarkerWidget(const ClientWidgetParam & params,
                                   const WidgetId & requestId,
                                   std::shared_ptr<interactive_markers::InteractiveMarkerServer> & server,
                                   bool control_orientation,
                                   bool control_position,
                                   ClientWidget * label);

  TransformInteractiveMarkerWidget(const ClientWidgetParam & params,
                                   const WidgetId & requestId,
                                   std::shared_ptr<interactive_markers::InteractiveMarkerServer> & server,
                                   const vm::InteractiveMarker & marker,
                                   bool control_orientation,
                                   bool control_position,
                                   ClientWidget * label);

  void handleRequest(const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback) override;

  void update(const Eigen::Vector3d & t)
  {
    marker_.update(t);
  }
  void update(const sva::PTransformd & pos)
  {
    marker_.update(pos);
  }

protected:
  bool control_orientation_;
  bool control_position_;
};

} // namespace mc_rtc_rviz
