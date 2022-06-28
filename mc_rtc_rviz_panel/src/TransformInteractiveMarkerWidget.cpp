#include "TransformInteractiveMarkerWidget.h"

namespace mc_rtc_rviz
{

TransformInteractiveMarkerWidget::TransformInteractiveMarkerWidget(
    const ClientWidgetParam & params,
    const WidgetId & requestId,
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> & server,
    bool control_orientation,
    bool control_position,
    ClientWidget * label)
: InteractiveMarkerWidget(
    params,
    requestId,
    server,
    make6DMarker(id2name(params.id), makeAxisMarker(0.15 * 0.9), control_position, control_orientation),
    label),
  control_orientation_(control_orientation), control_position_(control_position)
{
}

TransformInteractiveMarkerWidget::TransformInteractiveMarkerWidget(
    const ClientWidgetParam & params,
    const WidgetId & requestId,
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> & server,
    const vm::InteractiveMarker & marker,
    bool control_orientation,
    bool control_position,
    ClientWidget * label)
: InteractiveMarkerWidget(params, requestId, server, marker, label), control_orientation_(control_orientation),
  control_position_(control_position)
{
}

void TransformInteractiveMarkerWidget::handleRequest(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
{
  if(!control_position_ && !control_orientation_)
  {
    return;
  }
  if(control_position_ && !control_orientation_)
  {
    Eigen::Vector3d v{feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z};
    client().send_request(request_id_, v);
  }
  else if(!control_position_ && control_orientation_)
  {
    auto q = Eigen::Quaterniond{feedback->pose.orientation.w, feedback->pose.orientation.x,
                                feedback->pose.orientation.y, feedback->pose.orientation.z}
                 .inverse();
    client().send_request(request_id_, q);
  }
  else
  {
    Eigen::Vector3d v{feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z};
    auto q = Eigen::Quaterniond{feedback->pose.orientation.w, feedback->pose.orientation.x,
                                feedback->pose.orientation.y, feedback->pose.orientation.z}
                 .inverse();
    client().send_request(request_id_, sva::PTransformd{q, v});
  }
}

} // namespace mc_rtc_rviz
