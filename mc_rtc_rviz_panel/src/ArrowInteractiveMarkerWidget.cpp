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
    bool ro,
    ClientWidget * label)
: ClientWidget(params), request_id_(requestId), start_(start), end_(end),
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
      [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback) { handleEndRequest(feedback); }),
  arrow_marker_(server,
                id2name(requestId),
                makeInteractiveMarker(id2name(requestId) + "_arrow", makeArrowMarker(start, end, config)),
                [](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &) {})

{
  layout_ = new QVBoxLayout(this);
  if(!secret())
  {
    layout_->addWidget(new QLabel(id().name.c_str()));
  }
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

void ArrowInteractiveMarkerWidget::toggled(bool hide)
{
  start_marker_.toggle();
  end_marker_.toggle();
  arrow_marker_.toggle();
  button_->setText(hide ? "Show" : "Hide");
  visible(!hide);
}

} // namespace mc_rtc_rviz
