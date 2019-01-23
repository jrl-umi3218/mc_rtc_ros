#include "InteractiveMarkerWidget.h"

#include <mc_rbdyn/configuration_io.h>

namespace mc_rtc_rviz
{

InteractiveMarkerWidget::InteractiveMarkerWidget(const ClientWidgetParam & params,
                                                 interactive_markers::InteractiveMarkerServer & server,
                                                 const WidgetId & requestId,
                                                 const sva::PTransformd & pos,
                                                 bool control_orientation,
                                                 bool control_position)
: ClientWidget(params),
  marker_(server, id2name(requestId), control_position, control_orientation, vm::Marker::CUBE, [this](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback){ (*this)(feedback); }),
  request_id_(requestId),
  control_orientation_(control_orientation),
  control_position_(control_position)
{
  auto layout = new QVBoxLayout(this);
  button_ = new QPushButton(("Hide " + request_id_.name + " marker").c_str());
  button_->setCheckable(true);
  button_->setChecked(!visible());
  if(!visible())
  {
    marker_.toggle();
  }
  connect(button_, SIGNAL(toggled(bool)), this, SLOT(toggled(bool)));
  layout->addWidget(button_);
}

void InteractiveMarkerWidget::toggled(bool hide)
{
  marker_.toggle();
  if(hide)
  {
    button_->setText(("Show " + request_id_.name + " marker").c_str());
  }
  else
  {
    button_->setText(("Hide " + request_id_.name + " marker").c_str());
  }
  visible(!hide);
}

void InteractiveMarkerWidget::operator()(const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
{
  if(!control_position_ && !control_orientation_) { return; }
  if(control_position_ && !control_orientation_)
  {
    Eigen::Vector3d v {
      feedback->pose.position.x,
      feedback->pose.position.y,
      feedback->pose.position.z
    };
    client().send_request(request_id_, v);
  }
  else if(!control_position_ && control_orientation_)
  {
    auto q =  Eigen::Quaterniond{
            feedback->pose.orientation.w,
            feedback->pose.orientation.x,
            feedback->pose.orientation.y,
            feedback->pose.orientation.z
          }.inverse();
    client().send_request(request_id_, q);
  }
  else
  {
    Eigen::Vector3d v {
      feedback->pose.position.x,
      feedback->pose.position.y,
      feedback->pose.position.z
    };
    auto q =  Eigen::Quaterniond{
            feedback->pose.orientation.w,
            feedback->pose.orientation.x,
            feedback->pose.orientation.y,
            feedback->pose.orientation.z
          }.inverse();
    client().send_request(request_id_, sva::PTransformd{q, v});
  }
}

}
