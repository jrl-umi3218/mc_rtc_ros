#pragma once

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>

#include <SpaceVecAlg/SpaceVecAlg>

namespace vm = visualization_msgs;

namespace mc_rtc_rviz
{

vm::Marker makeVisual(int t, const vm::InteractiveMarker & marker);

vm::InteractiveMarkerControl & makeVisualControl(int t,
                                                 vm::InteractiveMarker & marker);

vm::InteractiveMarker make6DMarker(const std::string & name,
                                   bool control_position,
                                   bool control_orientation,
                                   int type);

struct SharedMarker
{
  SharedMarker(interactive_markers::InteractiveMarkerServer & server,
               const visualization_msgs::InteractiveMarker & marker,
               interactive_markers::InteractiveMarkerServer::FeedbackCallback callback)
  : server_(server),
    marker_(marker),
    callback_(callback)
  {
    server_.insert(marker, callback);
  }
  ~SharedMarker()
  {
    if(!hidden_)
    {
      server_.erase(marker_.name);
    }
  }

  void toggle()
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

  void update(const Eigen::Vector3d & t)
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

  void update(const sva::PTransformd & pos)
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
private:
  interactive_markers::InteractiveMarkerServer & server_;
  vm::InteractiveMarker marker_;
  interactive_markers::InteractiveMarkerServer::FeedbackCallback callback_;
  bool hidden_ = false;
};

}
