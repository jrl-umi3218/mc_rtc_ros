#include "TransformWidget.h"

#include "utils.h"

#include <mc_rbdyn/configuration_io.h>

TransformWidget::TransformWidget(const std::string & name,
                                 const std::string & full_name,
                                 const mc_rtc::Configuration & data,
                                 request_t request,
                                 std::shared_ptr<interactive_markers::InteractiveMarkerServer> int_server_)
: BaseWidget(new QVBoxLayout()),
  server(int_server_),
  marker_name_(full_name)
{
  //input = new PointInputDialog(name, {"x", "y", "z"}, data.has("SET"), false, request);
  //layout->addWidget(input);
  marker_ = make6DMarker(marker_name_, data.has("SET"), data.has("SET"));
  marker_cb_ = [request](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
    {
      mc_rtc::Configuration config;
      sva::PTransformd target;
      target.translation().x() = feedback->pose.position.x;
      target.translation().y() = feedback->pose.position.y;
      target.translation().z() = feedback->pose.position.z;
      Eigen::Quaterniond q { feedback->pose.orientation.w,
                             feedback->pose.orientation.x,
                             feedback->pose.orientation.y,
                             feedback->pose.orientation.z };
      target.rotation() = q.inverse().toRotationMatrix();
      config.add("data", target);
      request(config("data"));
    };
  int_server_->insert(marker_, marker_cb_);
  int_server_->applyChanges();
  visible = new QPushButton("👁");
  visible->setCheckable(true);
  visible->setSizePolicy({QSizePolicy::Policy::Fixed, QSizePolicy::Policy::Preferred});
  //input->layout->addWidget(visible, 0, 1, Qt::AlignCenter);
  connect(visible, &QPushButton::toggled,
          this, &TransformWidget::onVisibleToggle);
}

TransformWidget::~TransformWidget()
{
  if(!visible->isChecked())
  {
    server->erase(marker_name_);
  }
}

void TransformWidget::update(const mc_rtc::Configuration & data)
{
  //input->update(data);
  if(!visible->isChecked())
  {
    geometry_msgs::Pose pose;
    sva::PTransformd pos = data;
    Eigen::Quaterniond ori(pos.rotation());
    ori = ori.inverse();
    pose.orientation.w = ori.w();
    pose.orientation.x = ori.x();
    pose.orientation.y = ori.y();
    pose.orientation.z = ori.z();
    pose.position.x = pos.translation().x();
    pose.position.y = pos.translation().y();
    pose.position.z = pos.translation().z();
    server->insert(marker_, marker_cb_);
    server->setPose(marker_name_, pose);
  }
}

void TransformWidget::onVisibleToggle(bool t)
{
  auto font = visible->font();
  font.setStrikeOut(t);
  visible->setFont(font);
  if(t)
  {
    server->erase(marker_name_);
  }
  else
  {
    server->insert(marker_, marker_cb_);
  }
}
