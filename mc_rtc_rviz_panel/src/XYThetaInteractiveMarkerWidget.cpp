#include "XYThetaInteractiveMarkerWidget.h"

#include <mc_rbdyn/rpy_utils.h>

namespace mc_rtc_rviz
{

XYThetaInteractiveMarkerWidget::XYThetaInteractiveMarkerWidget(
    const ClientWidgetParam & params,
    const WidgetId & requestId,
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> & server,
    const sva::PTransformd & /*pos*/,
    bool control_orientation,
    bool control_position,
    ClientWidget * label)
: InteractiveMarkerWidget(params, requestId, server, makeXYThetaMarker(id2name(requestId)), label)
{
  if(control_position || control_orientation)
  {
    coupled_marker_ = marker_.marker();
    decoupled_marker_ = make6DMarker(id2name(requestId), makeAxisMarker(0.15 * 0.9), control_position,
                                     control_orientation, true, true, false, false, false, true);
    coupled_checkbox_ = new QCheckBox("Coupled position/orientation");
    coupled_checkbox_->setChecked(false);
    layout_->addWidget(coupled_checkbox_);
    connect(coupled_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(control_state_changed(int)));
    control_state_changed(/* anything = */ 42); // start with desired coupled/decoupled marker
  }
  else
  { // readonly
    marker_.marker(makeXYThetaMarker(id2name(requestId), true));
    marker_.applyChanges();
  }
}

void XYThetaInteractiveMarkerWidget::update(const Eigen::Vector3d & vec, double altitude)
{
  sva::PTransformd X(sva::RotZ(vec.z()), {vec.x(), vec.y(), altitude});
  marker_.update(X);
}

void XYThetaInteractiveMarkerWidget::handleRequest(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
{
  auto q = Eigen::Quaterniond{feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y,
                              feedback->pose.orientation.z}
               .inverse();
  Eigen::Matrix3d R(q);
  Eigen::VectorXd v(4);
  v << feedback->pose.position.x, feedback->pose.position.y, mc_rbdyn::rpyFromMat(R).z(), feedback->pose.position.z;
  client().send_request(request_id_, v);
}

void XYThetaInteractiveMarkerWidget::control_state_changed(int)
{
  if(coupled_checkbox_->isChecked())
  {
    marker_.marker(coupled_marker_);
  }
  else
  {
    marker_.marker(decoupled_marker_);
  }
  marker_.applyChanges();
}

} // namespace mc_rtc_rviz
