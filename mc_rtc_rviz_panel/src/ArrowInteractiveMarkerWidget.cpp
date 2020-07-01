#include "ArrowInteractiveMarkerWidget.h"

namespace mc_rtc_rviz
{

ArrowInteractiveMarkerWidget::ArrowInteractiveMarkerWidget(
    const ClientWidgetParam & params,
    const WidgetId & requestId,
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> & server,
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & end,
    const mc_rtc::gui::ArrowConfig & config,
    bool control_start,
    bool control_end,
    ClientWidget * label)
: ClientWidget(params), request_id_(requestId),
  start_marker_(
      server,
      id2name(requestId),
      vm::InteractiveMarker{},
      [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback) { handleStartRequest(feedback); }),
  end_marker_(
      server,
      id2name(requestId),
      vm::InteractiveMarker{},
      [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback) { handleEndRequest(feedback); }),
  arrow_marker_(server,
                id2name(requestId),
                makeInteractiveMarker(id2name(requestId) + "_arrow", makeArrowMarker(start, end, config)),
                [](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &) {})

{
  if(config.start_point_scale > 0)
  {
    start_marker_.marker(make3DMarker(id2name(params.id) + "_start",
                                      {getPointMarker(Eigen::Vector3d::Zero(), config.color, config.start_point_scale)},
                                      control_start));
  }
  else
  {
    start_marker_.marker(make3DMarker(id2name(params.id) + "_start", {}, control_start));
  }

  if(config.end_point_scale > 0)
  {
    end_marker_.marker(make3DMarker(id2name(params.id) + "_end",
                                    {getPointMarker(Eigen::Vector3d::Zero(), config.color, config.end_point_scale)},
                                    control_end));
  }
  else
  {
    end_marker_.marker(make3DMarker(id2name(params.id) + "_end", {}, control_end));
  }
  start_marker_.applyChanges();
  end_marker_.applyChanges();

  arrow_points_ << start, end;
  button_ = label->showHideButton();
  button_->setCheckable(true);
  button_->setChecked(!visible());
  if(!visible())
  {
    toggled(!visible());
  }
  connect(button_, SIGNAL(toggled(bool)), this, SLOT(toggled(bool)));
}

void ArrowInteractiveMarkerWidget::update(const Eigen::Vector3d & start,
                                          const Eigen::Vector3d & end,
                                          const mc_rtc::gui::ArrowConfig & c)
{
  if(!is_in_range(start))
  {
    mc_rtc::log::error("Could not display arrow {}: invalid value in start coordinates ({})", id2name(id()),
                       start.transpose());
    return;
  }
  if(!is_in_range(end))
  {
    mc_rtc::log::error("Could not display arrow {}: invalid value in end coordinates ({})", id2name(id()),
                       end.transpose());
    return;
  }

  arrow_points_.head<3>() = start;
  arrow_points_.tail<3>() = end;
  if(visible())
  {
    start_marker_.update(start);
    end_marker_.update(end);
    arrow_marker_.marker(makeInteractiveMarker(id2name(request_id_) + "_arrow", makeArrowMarker(start, end, c)));
    arrow_marker_.applyChanges();
  }
}

void ArrowInteractiveMarkerWidget::handleStartRequest(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
{
  arrow_points_.head<3>() =
      Eigen::Vector3d{feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z};
  client().send_request(request_id_, arrow_points_);
}

void ArrowInteractiveMarkerWidget::handleEndRequest(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
{
  arrow_points_.tail<3>() =
      Eigen::Vector3d{feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z};
  client().send_request(request_id_, arrow_points_);
}

void ArrowInteractiveMarkerWidget::toggled(bool hide)
{
  start_marker_.toggle();
  end_marker_.toggle();
  arrow_marker_.toggle();
  button_->setText(hide ? "Show" : "Hide");
  visible(!hide);
}

} // namespace mc_rtc_rviz
