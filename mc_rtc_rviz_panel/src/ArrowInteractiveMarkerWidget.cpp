#include "ArrowInteractiveMarkerWidget.h"

namespace mc_rtc_rviz
{

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

} // namespace mc_rtc_rviz
