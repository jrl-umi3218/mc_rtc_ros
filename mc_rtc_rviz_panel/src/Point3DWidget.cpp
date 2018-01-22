#include "Point3DWidget.h"

#include "utils.h"

Point3DWidget::Point3DWidget(const std::string & name,
                             const std::string & full_name,
                             const mc_rtc::Configuration & data,
                             request_t request,
                             std::shared_ptr<interactive_markers::InteractiveMarkerServer> int_server_)
: BaseWidget(new QVBoxLayout()),
  server(int_server_),
  marker_name_(full_name)
{
  input = new PointInputDialog(name, {"x", "y", "z"}, data.has("SET"), false, request);
  layout->addWidget(input);
  marker_ = make6DMarker(marker_name_, data.has("SET"), false);
  marker_cb_ = [request](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
    {
      mc_rtc::Configuration config;
      auto data = config.array("data", 3);
      data.push(feedback->pose.position.x);
      data.push(feedback->pose.position.y);
      data.push(feedback->pose.position.z);
      request(data);
    };
  int_server_->insert(marker_, marker_cb_);
  int_server_->applyChanges();
  visible = new QPushButton("👁");
  visible->setCheckable(true);
  visible->setSizePolicy({QSizePolicy::Policy::Fixed, QSizePolicy::Policy::Preferred});
  input->layout->addWidget(visible, 0, 1, Qt::AlignCenter);
  connect(visible, &QPushButton::toggled,
          this, &Point3DWidget::onVisibleToggle);
}

Point3DWidget::~Point3DWidget()
{
  if(!visible->isChecked())
  {
    server->erase(marker_name_);
  }
}

void Point3DWidget::update(const mc_rtc::Configuration & data)
{
  input->update(data);
  if(!visible->isChecked())
  {
    geometry_msgs::Pose pose;
    Eigen::Vector3d v = data;
    pose.orientation.w = 1.0;
    pose.position.x = v.x();
    pose.position.y = v.y();
    pose.position.z = v.z();
    server->insert(marker_, marker_cb_);
    server->setPose(marker_name_, pose);
  }
}

void Point3DWidget::onVisibleToggle(bool t)
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
