#pragma once

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>

#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_rtc/GUITypes.h>

namespace vm = visualization_msgs;

namespace mc_rtc_rviz
{

visualization_msgs::Marker getPointMarker(const std::string & ns, const Eigen::Vector3d & pos, const mc_rtc::gui::Color& color, double scale);
geometry_msgs::Point rosPoint(const Eigen::Vector3d& vec);
vm::Marker makeVisual(int t, double baseScale);
std::vector<vm::Marker> makeAxisMarker(double baseScale);
vm::InteractiveMarker make6DMarker(const std::string & name,
                                   bool control_position,
                                   bool control_orientation,
                                   const std::vector<vm::Marker>& visual_markers);

struct SharedMarker
{
  SharedMarker(interactive_markers::InteractiveMarkerServer & server,
               const std::string & name,
               const vm::InteractiveMarker &marker,
               interactive_markers::InteractiveMarkerServer::FeedbackCallback callback);

  ~SharedMarker();

  void toggle();

  void update(const Eigen::Vector3d & t);

  void update(const sva::PTransformd & pos);
private:
  interactive_markers::InteractiveMarkerServer & server_;
  vm::InteractiveMarker marker_;
  interactive_markers::InteractiveMarkerServer::FeedbackCallback callback_;
  bool hidden_ = false;
};

}
