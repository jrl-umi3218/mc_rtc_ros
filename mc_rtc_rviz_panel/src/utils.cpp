#include "utils.h"

namespace mc_rtc_rviz
{

namespace
{

vm::Marker makeVisual(int t, const vm::InteractiveMarker & marker)
{
  vm::Marker ret;
  ret.type = t;
  ret.scale.x = marker.scale*0.45;
  ret.scale.y = marker.scale*0.45;
  ret.scale.z = marker.scale*0.45;
  ret.color.r = 1.0;
  ret.color.g = 0.0;
  ret.color.b = 0.0;
  ret.color.a = 1.0;
  ret.pose.orientation.w = 1.0;
  return ret;
}

vm::InteractiveMarkerControl & makeVisualControl(int t,
    vm::InteractiveMarker & marker)
{
  vm::InteractiveMarkerControl ret;
  ret.always_visible = true;
  ret.orientation.w = 1.0;
  ret.markers.push_back(makeVisual(t, marker));
  marker.controls.push_back(ret);
  return marker.controls.back();
}

vm::InteractiveMarker make6DMarker(const std::string & name,
                                   bool control_position,
                                   bool control_orientation,
                                   int type)
{
  vm::InteractiveMarker ret;
  ret.header.frame_id = "robot_map";
  ret.scale = 0.15;
  ret.name = name;
  ret.description = "";
  makeVisualControl(type, ret);

  vm::InteractiveMarkerControl control;
  control.orientation.w = 0.707107;
  control.orientation.x = 0.707107;
  control.orientation.y = 0;
  control.orientation.z = 0;
  if(control_orientation)
  {
    control.name = "rotate_x";
    control.interaction_mode = vm::InteractiveMarkerControl::ROTATE_AXIS;
    ret.controls.push_back(control);
  }
  if(control_position)
  {
    control.name = "move_x";
    control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
    ret.controls.push_back(control);
  }

  control.orientation.w = 0.707107;
  control.orientation.x = 0;
  control.orientation.y = 0.707107;
  control.orientation.z = 0;
  if(control_orientation)
  {
    control.name = "rotate_z";
    control.interaction_mode = vm::InteractiveMarkerControl::ROTATE_AXIS;
    ret.controls.push_back(control);
  }
  if(control_position)
  {
    control.name = "move_z";
    control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
    ret.controls.push_back(control);
  }

  control.orientation.w = 0.707107;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0.707107;
  if(control_orientation)
  {
    control.name = "rotate_y";
    control.interaction_mode = vm::InteractiveMarkerControl::ROTATE_AXIS;
    ret.controls.push_back(control);
  }
  if(control_position)
  {
    control.name = "move_y";
    control.interaction_mode = vm::InteractiveMarkerControl::MOVE_AXIS;
    ret.controls.push_back(control);
  }

  return ret;
}

}

SharedMarker::SharedMarker(interactive_markers::InteractiveMarkerServer & server,
                           const std::string & name,
                           bool control_position, bool control_orientation, int type,
                           interactive_markers::InteractiveMarkerServer::FeedbackCallback callback)
: server_(server),
  marker_(make6DMarker(name, control_position, control_orientation, type)),
  callback_(callback)
{
  server_.insert(marker_, callback_);
}

SharedMarker::~SharedMarker()
{
  if(!hidden_)
  {
    server_.erase(marker_.name);
  }
}

void SharedMarker::toggle()
{
  if(hidden_)
  {
    hidden_ = false;
    server_.insert(marker_, callback_);
  }
  else
  {
    hidden_ = true;
    server_.erase(marker_.name);
  }
}

void SharedMarker::update(const Eigen::Vector3d & t)
{
  if(!hidden_)
  {
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = t.x();
    pose.position.y = t.y();
    pose.position.z = t.z();
    server_.setPose(marker_.name, pose);
  }
}

void SharedMarker::update(const sva::PTransformd & pos)
{
  if(!hidden_)
  {
    geometry_msgs::Pose pose;
    Eigen::Quaterniond q {pos.rotation().transpose()};
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.position.x = pos.translation().x();
    pose.position.y = pos.translation().y();
    pose.position.z = pos.translation().z();
    server_.setPose(marker_.name, pose);
  }
}

}
