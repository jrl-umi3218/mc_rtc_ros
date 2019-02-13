#include "InteractiveMarkerWidget.h"

#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/rpy_utils.h>

namespace mc_rtc_rviz
{

InteractiveMarkerWidget::InteractiveMarkerWidget(const ClientWidgetParam & params,
                                                 const WidgetId & requestId,
                                                 interactive_markers::InteractiveMarkerServer & server,
                                                 const vm::InteractiveMarker & marker,
                                                 ClientWidget * label)
: ClientWidget(params), request_id_(requestId),
  marker_(server,
          id2name(requestId),
          marker,
          [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback) { handleRequest(feedback); })
{
  layout_ = new QVBoxLayout(this);
  button_ = label->showHideButton();
  if(!button_)
  {
    button_ = new QPushButton("Hide");
    layout_->addWidget(button_);
  }
  button_->setCheckable(true);
  button_->setChecked(!visible());
  if(!visible())
  {
    toggled(!visible());
  }
  connect(button_, SIGNAL(toggled(bool)), this, SLOT(toggled(bool)));
}

void InteractiveMarkerWidget::toggled(bool hide)
{
  marker_.toggle();
  button_->setText(hide ? "Show" : "Hide");
  visible(!hide);
}

TransformInteractiveMarkerWidget::TransformInteractiveMarkerWidget(const ClientWidgetParam & params,
                                                                   const WidgetId & requestId,
                                                                   interactive_markers::InteractiveMarkerServer & server,
                                                                   const sva::PTransformd & /*pos*/,
                                                                   bool control_orientation,
                                                                   bool control_position,
                                                                   ClientWidget * label)
: InteractiveMarkerWidget(
      params,
      requestId,
      server,
      make6DMarker(id2name(requestId), control_position, control_orientation, makeAxisMarker(0.15 * 0.9)),
      label),
  control_orientation_(control_orientation), control_position_(control_position)
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

XYThetaInteractiveMarkerWidget::XYThetaInteractiveMarkerWidget(const ClientWidgetParam & params,
                                                               const WidgetId & requestId,
                                                               interactive_markers::InteractiveMarkerServer & server,
                                                               const sva::PTransformd & /*pos*/,
                                                               bool control_orientation,
                                                               bool control_position,
                                                               ClientWidget * label)
: InteractiveMarkerWidget(params, requestId, server, makeXYThetaMarker(id2name(requestId)), label)
{
  coupled_marker_ = marker_.marker();
  decoupled_marker_ = make6DMarker(id2name(requestId), control_position, control_orientation,
                                   makeAxisMarker(0.15 * 0.9), true, true, false, false, false, true);
  if(control_position || control_orientation)
  {
    coupled_checkbox_ = new QCheckBox("Coupled position/orientation");
    coupled_checkbox_->setChecked(false);
    layout_->addWidget(coupled_checkbox_);
    connect(coupled_checkbox_, SIGNAL(stateChanged(int)), this, SLOT(control_state_changed(int)));
  }
  control_state_changed(/* anything = */ 42); // start with desired coupled/decoupled marker
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
