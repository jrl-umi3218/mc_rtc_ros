#include "ForceInteractiveMarkerWidget.h"

namespace mc_rtc_rviz
{

ForceInteractiveMarkerWidget::ForceInteractiveMarkerWidget(
    const ClientWidgetParam & params,
    const WidgetId & requestId,
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> & server,
    const sva::PTransformd & surface,
    const sva::ForceVecd & force,
    const mc_rtc::gui::ForceConfig & config,
    bool ro,
    ClientWidget * label)
: ArrowInteractiveMarkerWidget(params,
                               requestId,
                               server,
                               surface.translation(),
                               surface.translation(),
                               config,
                               false,
                               !ro,
                               label),
  config_(config), force_(force)
{
}

void ForceInteractiveMarkerWidget::update(const sva::PTransformd & surface,
                                          const sva::ForceVecd & force,
                                          const mc_rtc::gui::ForceConfig & c)
{
  config_ = c;
  force_ = force;
  const auto & start = surface.translation();
  sva::PTransformd X_surface_end = Eigen::Vector3d{config_.force_scale * force.force()};
  Eigen::Vector3d end = (X_surface_end * surface).translation();
  ArrowInteractiveMarkerWidget::update(start, end, c);
}

void ForceInteractiveMarkerWidget::handleEndRequest(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
{
  Eigen::Vector3d end =
      Eigen::Vector3d{feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z};
  const Eigen::Vector3d & start = arrow_points_.head<3>();

  client().send_request(request_id_, sva::ForceVecd(force_.couple(), 1. / config_.force_scale * (end - start)));
}

} // namespace mc_rtc_rviz
