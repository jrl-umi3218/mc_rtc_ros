/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "InteractiveMarkerWidget.h"

#include <mc_rbdyn/configuration_io.h>
#include <mc_rbdyn/rpy_utils.h>

namespace mc_rtc_rviz
{

InteractiveMarkerWidget::InteractiveMarkerWidget(const ClientWidgetParam & params,
                                                 const WidgetId & requestId,
                                                 std::shared_ptr<interactive_markers::InteractiveMarkerServer> & server,
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

ArrowInteractiveMarkerWidget::ArrowInteractiveMarkerWidget(
    const ClientWidgetParam & params,
    const WidgetId & requestId,
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> & server,
    visualization_msgs::MarkerArray & markers,
    const mc_rtc::gui::ArrowConfig & config,
    bool ro,
    ClientWidget * label)
: ClientWidget(params), request_id_(requestId), markers_(markers),
  start_marker_(
      server,
      id2name(requestId),
      make3DMarker(id2name(params.id) + "_start",
                   {getPointMarker(Eigen::Vector3d::Zero(), config.color, config.start_point_scale)},
                   ro),
      [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback) { handleStartRequest(feedback); }),
  end_marker_(
      server,
      id2name(requestId),
      make3DMarker(id2name(params.id) + "_end",
                   {getPointMarker(Eigen::Vector3d::Zero(), config.color, config.end_point_scale)},
                   ro),
      [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback) { handleEndRequest(feedback); })
{
}

void ArrowInteractiveMarkerWidget::update(const Eigen::Vector3d & start,
                                          const sva::ForceVecd & force,
                                          const mc_rtc::gui::ForceConfig & c)
{
  const auto & end = start + c.force_scale * force.force();
  ArrowInteractiveMarkerWidget::update(start, end, c);
}

void ArrowInteractiveMarkerWidget::update(const Eigen::Vector3d & start,
                                          const Eigen::Vector3d & end,
                                          const mc_rtc::gui::ArrowConfig & c)
{
  start_ = start;
  end_ = end_;
  start_marker_.update(start);
  end_marker_.update(end);

  // bool show = visible_ || was_visible_;
  visualization_msgs::Marker m;
  m.type = visualization_msgs::Marker::ARROW;
  m.action = visualization_msgs::Marker::ADD;
  // if(!visible_ && was_visible_)
  //{
  //  m.action = visualization_msgs::Marker::DELETE;
  //}
  m.lifetime = ros::Duration(1);
  m.points.push_back(rosPoint(start));
  m.points.push_back(rosPoint(end));
  m.scale.x = c.shaft_diam;
  m.scale.y = c.head_diam;
  m.scale.z = c.head_len;
  m.color.a = c.color.a;
  m.color.r = c.color.r;
  m.color.g = c.color.g;
  m.color.b = c.color.b;
  m.header.stamp = ros::Time::now();
  m.header.frame_id = "robot_map";
  m.ns = id2name(id());
  // if(show)
  //{
  markers_.markers.push_back(m);
  //}

  // was_visible_ = visible_;
}

void ArrowInteractiveMarkerWidget::handleStartRequest(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
{
  start_ = Eigen::Vector3d{feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z};
  mc_rtc::Configuration data;
  data.add("start", start_);
  data.add("end", end_);
  client().send_request(request_id_, data);
}

void ArrowInteractiveMarkerWidget::handleEndRequest(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
{
  end_ = Eigen::Vector3d{feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z};
  mc_rtc::Configuration data;
  data.add("start", start_);
  data.add("end", end_);
  client().send_request(request_id_, data);
}

Point3DInteractiveMarkerWidget::Point3DInteractiveMarkerWidget(
    const ClientWidgetParam & params,
    const WidgetId & requestId,
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> & server,
    const mc_rtc::gui::PointConfig & config,
    bool control_position,
    ClientWidget * label)
: TransformInteractiveMarkerWidget(params,
                                   requestId,
                                   server,
                                   make3DMarker(id2name(params.id),
                                                {getPointMarker(Eigen::Vector3d::Zero(), config.color, config.scale)},
                                                control_position),
                                   false,
                                   control_position,
                                   label)

{
}

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
  coupled_marker_ = marker_.marker();
  decoupled_marker_ = make6DMarker(id2name(requestId), makeAxisMarker(0.15 * 0.9), control_position,
                                   control_orientation, true, true, false, false, false, true);
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
