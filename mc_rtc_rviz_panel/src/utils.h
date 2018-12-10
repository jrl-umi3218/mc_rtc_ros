#pragma once

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>

#include <SpaceVecAlg/SpaceVecAlg>

namespace vm = visualization_msgs;

namespace mc_rtc_rviz
{

struct SharedMarker
{
  SharedMarker(interactive_markers::InteractiveMarkerServer & server,
               const std::string & name,
               bool control_position, bool control_orientation, int type,
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
