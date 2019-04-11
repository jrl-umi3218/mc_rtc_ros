#pragma once

#include <mc_rtc/GUITypes.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>

#include <interactive_markers/interactive_marker_server.h>

namespace vm = visualization_msgs;

namespace mc_rtc_rviz
{

visualization_msgs::Marker getPointMarker(const std::string & ns,
                                          const Eigen::Vector3d & pos,
                                          const mc_rtc::gui::Color & color,
                                          double scale);
geometry_msgs::Point rosPoint(const Eigen::Vector3d & vec);
vm::Marker makeVisual(int t, double baseScale);
std::vector<vm::Marker> makeAxisMarker(double baseScale);
vm::InteractiveMarker make6DMarker(const std::string & name,
                                   bool control_position,
                                   bool control_orientation,
                                   const std::vector<vm::Marker> & visual_markers,
                                   bool move_x = true,
                                   bool move_y = true,
                                   bool move_z = true,
                                   bool rotate_x = true,
                                   bool rotate_y = true,
                                   bool rotate_z = true);
vm::InteractiveMarker makeXYThetaMarker(const std::string & name);

struct SharedMarker
{
  SharedMarker(std::shared_ptr<interactive_markers::InteractiveMarkerServer> server,
               const std::string & name,
               const vm::InteractiveMarker & marker,
               interactive_markers::InteractiveMarkerServer::FeedbackCallback callback);

  ~SharedMarker();

  void toggle();

  void update(const Eigen::Vector3d & t);

  void update(const sva::PTransformd & pos);

  void marker(const vm::InteractiveMarker & marker)
  {
    server_->erase(marker_.name);
    marker_ = marker;
    server_->insert(marker_, callback_);
  }

  vm::InteractiveMarker & marker()
  {
    return marker_;
  }

  void applyChanges()
  {
    // XXX without erasing and re-inserting the marker, applyChanges does
    // nothing when the marker controls have been modified.
    server_->erase(marker_.name);
    server_->insert(marker_, callback_);
    server_->applyChanges();
  }

private:
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  vm::InteractiveMarker marker_;
  interactive_markers::InteractiveMarkerServer::FeedbackCallback callback_;
  bool hidden_ = false;
};

} // namespace mc_rtc_rviz
