#include "Point3DWidget.h"

#include "utils.h"

Point3DWidget::Point3DWidget(const std::string & name,
                             const mc_rtc::Configuration & data,
                             request_t request,
                             interactive_markers::InteractiveMarkerServer & int_server_)
: BaseWidget(new QVBoxLayout()),
  server(int_server_)
{
  input = new PointInputDialog(name, {"x", "y", "z"}, data.has("SET"), false, request);
  layout->addWidget(input);
  auto marker = make6DMarker(name, data.has("SET"), false);
  int_server_.insert(marker,
      [request](const visualization_msgs::InteractiveMarkerFeedbackConstPtr & feedback)
      {
        mc_rtc::Configuration config;
        auto data = config.array("data", 3);
        data.push(feedback->pose.position.x);
        data.push(feedback->pose.position.y);
        data.push(feedback->pose.position.z);
        request(data);
      }
  );
  marker_name_ = name;
}

void Point3DWidget::update(const mc_rtc::Configuration & data)
{
  input->update(data);
  geometry_msgs::Pose pose;
  Eigen::Vector3d v = data;
  pose.orientation.w = 1.0;
  pose.position.x = v.x();
  pose.position.y = v.y();
  pose.position.z = v.z();
  server.setPose(marker_name_, pose);
}
