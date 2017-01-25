#include <iostream>
#include <sch/S_Polyhedron/S_Polyhedron.h>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/SCHAddon.h>

#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Point.h"

namespace
{
  template<typename T>
  void getParam(ros::NodeHandle & n, const std::string & param, T & out)
  {
    std::string true_param;
    n.searchParam(param, true_param);
    n.getParam(true_param, out);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mc_convex_visualization");

  ros::NodeHandle n;
  std::vector<std::string> robot_params = {"HRP2DRC"};
  std::string tf_prefix = "";
  std::vector<std::string> filtered_convexes = {};
  getParam(n, "mc_convex_visualization/robot_params", robot_params);
  getParam(n, "mc_convex_visualization/tf_prefix", tf_prefix);
  getParam(n, "mc_convex_visualization/filtered_convexes", filtered_convexes);

  ros::Publisher sch_pub = n.advertise<visualization_msgs::MarkerArray>("sch_marker", 1000);

  std::shared_ptr<mc_rbdyn::RobotModule> robot;
  /* SHAME */
  if(robot_params.size() == 1)
  {
    robot = mc_rbdyn::RobotLoader::get_robot_module(robot_params[0]);
  }
  else if(robot_params.size() == 2)
  {
    robot = mc_rbdyn::RobotLoader::get_robot_module(robot_params[0], robot_params[1]);
  }
  else if(robot_params.size() == 3)
  {
    robot = mc_rbdyn::RobotLoader::get_robot_module(robot_params[0], robot_params[1], robot_params[2]);
  }
  else
  {
    ROS_ERROR_STREAM("Invalid robot_params size passed to mc_convex_visualization: " << robot_params.size());
  }
  auto cols = robot->convexHull();

  visualization_msgs::MarkerArray markers;
  unsigned id = 0;
  for(const auto & col : cols)
  {
    if(std::find(filtered_convexes.begin(), filtered_convexes.end(), col.first) != filtered_convexes.end())
    {
      continue;
    }
    const std::string & frame = col.second.first;
    sch::S_Polyhedron * poly = sch::mc_rbdyn::Polyhedron(col.second.second);
    sva::PTransformd colTrans = sva::PTransformd::Identity();
    if(robot->collisionTransforms().count(frame))
    {
      colTrans = robot->collisionTransforms().at(frame);
    }
    auto & pa = *(poly->getPolyhedronAlgorithm());
    visualization_msgs::Marker marker;
    marker.header.frame_id = tf_prefix + frame;
    marker.header.stamp = ros::Time();
    marker.ns = col.first;
    marker.id = ++id;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(0.5);
    const auto triangles = pa.triangles_;
    const auto vertexes = pa.vertexes_;
    for(unsigned int i=0; i < triangles.size(); i++)
    {
      const auto a = vertexes[triangles[i].a]->getCordinates();
      const auto b = vertexes[triangles[i].b]->getCordinates();
      const auto c = vertexes[triangles[i].c]->getCordinates();
      sva::PTransformd va(Eigen::Vector3d(a[0], a[1], a[2]));
      sva::PTransformd vb(Eigen::Vector3d(b[0], b[1], b[2]));
      sva::PTransformd vc(Eigen::Vector3d(c[0], c[1], c[2]));
      va = va*colTrans;
      vb = vb*colTrans;
      vc = vc*colTrans;
      geometry_msgs::Point p;
      p.x = va.translation().x();
      p.y = va.translation().y();
      p.z = va.translation().z();
      marker.points.push_back(p);
      p.x = vb.translation().x();
      p.y = vb.translation().y();
      p.z = vb.translation().z();
      marker.points.push_back(p);
      p.x = vc.translation().x();
      p.y = vc.translation().y();
      p.z = vc.translation().z();
      marker.points.push_back(p);
    }
    markers.markers.push_back(marker);
  }

  ros::Rate rate(10);
  while(ros::ok())
  {
    sch_pub.publish(markers);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
