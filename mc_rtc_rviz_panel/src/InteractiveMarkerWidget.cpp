/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "InteractiveMarkerWidget.h"

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

} // namespace mc_rtc_rviz
