/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rtc/gui/types.h>
#include <mc_rtc_ros/ros.h>

#include <SpaceVecAlg/SpaceVecAlg>

#ifdef MC_RTC_ROS_IS_ROS2
#  include <std_msgs/msg/color_rgba.hpp>
#  include <visualization_msgs/msg/interactive_marker.hpp>
#  include <visualization_msgs/msg/interactive_marker_control.hpp>
#  include <visualization_msgs/msg/marker_array.hpp>
#  include <interactive_markers/interactive_marker_server.hpp>
#else
#  include <std_msgs/ColorRGBA.h>
#  include <visualization_msgs/InteractiveMarker.h>
#  include <visualization_msgs/InteractiveMarkerControl.h>
#  include <visualization_msgs/MarkerArray.h>
#  include <interactive_markers/interactive_marker_server.h>
#endif

namespace mc_rtc_rviz
{

#ifdef MC_RTC_ROS_IS_ROS2
using ColorRGBA = std_msgs::msg::ColorRGBA;
using InteractiveMarker = visualization_msgs::msg::InteractiveMarker;
using InteractiveMarkerServer = interactive_markers::InteractiveMarkerServer;
using InteractiveMarkerControl = visualization_msgs::msg::InteractiveMarkerControl;
using InteractiveMarkerFeedbackConstPtr = visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;
using Point = geometry_msgs::msg::Point;
using Pose = geometry_msgs::msg::Pose;
#else
using ColorRGBA = std_msgs::ColorRGBA;
using InteractiveMarker = visualization_msgs::InteractiveMarker;
using InteractiveMarkerServer = interactive_markers::InteractiveMarkerServer;
using InteractiveMarkerControl = visualization_msgs::InteractiveMarkerControl;
using InteractiveMarkerFeedbackConstPtr = visualization_msgs::InteractiveMarkerFeedbackConstPtr;
using Marker = visualization_msgs::Marker;
using MarkerArray = visualization_msgs::MarkerArray;
using Point = geometry_msgs::Point;
using Pose = geometry_msgs::Pose;
#endif

// Check if at least one values in x.array() is nan
template<typename Derived>
inline bool is_nan(const Eigen::MatrixBase<Derived> & x)
{
  return x.array().isNaN().any();
}

template<typename Derived>
inline bool is_in_range(const Eigen::MatrixBase<Derived> & x, double min = -10e10, double max = 10e10)
{
  return (x.array() > min).all() && (x.array() < max).all();
}

Marker getPointMarker(const Eigen::Vector3d & pos, const mc_rtc::gui::Color & color, double scale);
Point rosPoint(const Eigen::Vector3d & vec);
Marker makeVisual(int t, double baseScale);
InteractiveMarker makeInteractiveMarker(const std::string & name, const std::vector<Marker> & visual_markers);
std::vector<Marker> makeAxisMarker(double baseScale);
InteractiveMarker make6DMarker(const std::string & name,
                               const std::vector<Marker> & visual_markers,
                               bool control_position,
                               bool control_orientation,
                               bool move_x = true,
                               bool move_y = true,
                               bool move_z = true,
                               bool rotate_x = true,
                               bool rotate_y = true,
                               bool rotate_z = true);
InteractiveMarker make3DMarker(const std::string & name,
                               const std::vector<Marker> & visual_markers,
                               bool control_position,
                               bool move_x = true,
                               bool move_y = true,
                               bool move_z = true);
InteractiveMarker makeXYThetaMarker(const std::string & name, bool readonly = false);
Marker makeArrowMarker(const Eigen::Vector3d & start, const Eigen::Vector3d & end, const mc_rtc::gui::ArrowConfig & c);

struct SharedMarker
{
  SharedMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
               const std::string & name,
               const InteractiveMarker & marker,
               interactive_markers::InteractiveMarkerServer::FeedbackCallback callback);

  ~SharedMarker();

  void toggle();

  void update(const Eigen::Vector3d & t);

  void update(const sva::PTransformd & pos);

  void marker(const InteractiveMarker & marker)
  {
    server_->erase(marker_.name);
    marker_ = marker;
    server_->insert(marker_, callback_);
  }

  InteractiveMarker & marker() { return marker_; }

  void applyChanges() { server_->applyChanges(); }

private:
  std::shared_ptr<InteractiveMarkerServer> server_;
  InteractiveMarker marker_;
  InteractiveMarkerServer::FeedbackCallback callback_;
  bool hidden_ = false;
};

} // namespace mc_rtc_rviz
