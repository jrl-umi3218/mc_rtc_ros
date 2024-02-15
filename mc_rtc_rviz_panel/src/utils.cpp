/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "utils.h"

namespace mc_rtc_rviz
{

Point rosPoint(const Eigen::Vector3d & vec)
{
  Point p;
  p.x = vec.x();
  p.y = vec.y();
  p.z = vec.z();
  return p;
}

Marker makeVisual(int t, double scale)
{
  Marker ret;
  ret.action = Marker::ADD;
  ret.type = t;
  ret.scale.x = scale;
  ret.scale.y = scale;
  ret.scale.z = scale;
  ret.color.r = 1.0;
  ret.color.g = 0.0;
  ret.color.b = 0.0;
  ret.color.a = 1.0;
  ret.pose.orientation.w = 1.0;
  return ret;
}

std::vector<Marker> makeAxisMarker(double scale)
{
  const Eigen::Vector3d t0 = scale * Eigen::Vector3d{0., 0., 0.};
  const Eigen::Vector3d tx = scale * Eigen::Vector3d{1., 0., 0.};
  const Eigen::Vector3d ty = scale * Eigen::Vector3d{0., 1., 0.};
  const Eigen::Vector3d tz = scale * Eigen::Vector3d{0., 0., 1.};

  Marker m;
  m.type = Marker::ARROW;
  m.action = Marker::ADD;
  // Arrow shaft diameter
  m.scale.x = scale * 0.15;
  // arrow head diameter
  m.scale.y = scale * 0.15;
  // arrow head length
  m.scale.z = scale * 0.5;
  m.pose.orientation.w = 1.0;

  std::vector<Marker> ret;
  // X axis
  m.points.push_back(rosPoint(t0));
  m.points.push_back(rosPoint(tx));
  m.color.a = 1.;
  m.color.r = 1.;
  m.color.g = 0.;
  m.color.b = 0.;
  ret.push_back(m);
  // Y axis
  m.points.clear();
  m.points.push_back(rosPoint(t0));
  m.points.push_back(rosPoint(ty));
  m.color.a = 1.;
  m.color.r = 0.;
  m.color.g = 1.;
  m.color.b = 0.;
  ret.push_back(m);
  // Z axis
  m.points.clear();
  m.points.push_back(rosPoint(t0));
  m.points.push_back(rosPoint(tz));
  m.color.a = 1.;
  m.color.r = 0.;
  m.color.g = 0.;
  m.color.b = 1.;
  ret.push_back(m);
  return ret;
}

std::vector<Marker> makeArrowMarker(const Eigen::Vector3d & start,
                                    const Eigen::Vector3d & end,
                                    const mc_rtc::gui::ArrowConfig & c)
{
  std::vector<Marker> markers;
  Marker m;
  m.type = Marker::ARROW;
  m.action = Marker::ADD;
  m.points.push_back(rosPoint(start));
  m.points.push_back(rosPoint(end));
  m.scale.x = c.shaft_diam;
  m.scale.y = c.head_diam;
  m.scale.z = c.head_len;
  m.color.a = static_cast<float>(c.color.a);
  m.color.r = static_cast<float>(c.color.r);
  m.color.g = static_cast<float>(c.color.g);
  m.color.b = static_cast<float>(c.color.b);
  markers.push_back(m);
  return markers;
}

InteractiveMarkerControl & makeVisualControl(const std::vector<Marker> & visual_makers, InteractiveMarker & marker)
{
  InteractiveMarkerControl ret;
  ret.always_visible = true;
  ret.orientation.w = 1.0;
  ret.markers = visual_makers;
  marker.controls.push_back(ret);
  return marker.controls.back();
}

InteractiveMarker makeInteractiveMarker(const std::string & name, const std::vector<Marker> & visual_markers)
{
  InteractiveMarker marker;
  marker.header.frame_id = "robot_map";
  marker.name = name;
  InteractiveMarkerControl ret;
  ret.always_visible = true;
  ret.orientation.w = 1.0;
  ret.markers = visual_markers;
  marker.controls.push_back(ret);
  return marker;
}

InteractiveMarker make6DMarker(const std::string & name,
                               const std::vector<Marker> & visual_markers,
                               bool control_position,
                               bool control_orientation,
                               bool move_x,
                               bool move_y,
                               bool move_z,
                               bool rotate_x,
                               bool rotate_y,
                               bool rotate_z)
{
  InteractiveMarker ret;
  ret.header.frame_id = "robot_map";
  ret.scale = 0.15f;
  ret.name = name;
  ret.description = "";
  makeVisualControl(visual_markers, ret);

  InteractiveMarkerControl control;
  control.orientation.w = 0.707107;
  control.orientation.x = 0.707107;
  control.orientation.y = 0;
  control.orientation.z = 0;
  if(control_orientation && rotate_x)
  {
    control.name = "rotate_x";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    ret.controls.push_back(control);
  }
  if(control_position && move_x)
  {
    control.name = "move_x";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    ret.controls.push_back(control);
  }

  control.orientation.w = 0.707107;
  control.orientation.x = 0;
  control.orientation.y = 0.707107;
  control.orientation.z = 0;
  if(control_orientation && rotate_z)
  {
    control.name = "rotate_z";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    ret.controls.push_back(control);
  }
  if(control_position && move_z)
  {
    control.name = "move_z";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    ret.controls.push_back(control);
  }

  control.orientation.w = 0.707107;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0.707107;
  if(control_orientation && rotate_y)
  {
    control.name = "rotate_y";
    control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
    ret.controls.push_back(control);
  }
  if(control_position && move_y)
  {
    control.name = "move_y";
    control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
    ret.controls.push_back(control);
  }

  return ret;
}

InteractiveMarker make3DMarker(const std::string & name,
                               const std::vector<Marker> & visual_markers,
                               bool control_position,
                               bool /*move_x*/,
                               bool /*move_y*/,
                               bool /*move_z*/)
{
  return make6DMarker(name, visual_markers, control_position, false);
}

InteractiveMarker makeXYThetaMarker(const std::string & name, bool readonly)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "robot_map";
  int_marker.scale = 0.25;
  int_marker.name = name;
  int_marker.description = "";
  makeVisualControl(makeAxisMarker(0.15 * 0.9), int_marker);

  if(!readonly)
  {
    InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
    int_marker.controls.push_back(control);
  }
  return int_marker;
}

Marker getPointMarker(const Eigen::Vector3d & pos, const mc_rtc::gui::Color & color, double scale = 0.02)
{
  Marker m;
  m.type = Marker::SPHERE;
  m.action = Marker::ADD;
  m.scale.x = scale;
  m.scale.y = scale;
  m.scale.z = scale;
  m.color.a = static_cast<float>(color.a);
  m.color.r = static_cast<float>(color.r);
  m.color.g = static_cast<float>(color.g);
  m.color.b = static_cast<float>(color.b);
  m.pose.orientation.w = 1.0;
  m.pose.orientation.x = 0.0;
  m.pose.orientation.y = 0.0;
  m.pose.orientation.z = 0.0;
  m.pose.position.x = pos.x();
  m.pose.position.y = pos.y();
  m.pose.position.z = pos.z();
  return m;
}

SharedMarker::SharedMarker(std::shared_ptr<InteractiveMarkerServer> server,
                           const std::string & /*name*/,
                           const InteractiveMarker & marker,
                           InteractiveMarkerServer::FeedbackCallback callback)
: server_(server), marker_(marker), callback_(callback)
{
  server_->insert(marker_, callback_);
}

SharedMarker::~SharedMarker()
{
  if(!hidden_) { server_->erase(marker_.name); }
}

void SharedMarker::toggle()
{
  if(hidden_)
  {
    hidden_ = false;
    server_->insert(marker_, callback_);
  }
  else
  {
    hidden_ = true;
    server_->erase(marker_.name);
  }
}

void SharedMarker::update(const Eigen::Vector3d & t)
{
  if(!hidden_)
  {
    Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = t.x();
    pose.position.y = t.y();
    pose.position.z = t.z();
    server_->setPose(marker_.name, pose);
  }
}

void SharedMarker::update(const sva::PTransformd & pos)
{
  if(!hidden_)
  {
    Pose pose;
    Eigen::Quaterniond q{pos.rotation().transpose()};
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.position.x = pos.translation().x();
    pose.position.y = pos.translation().y();
    pose.position.z = pos.translation().z();
    server_->setPose(marker_.name, pose);
  }
}

} // namespace mc_rtc_rviz
