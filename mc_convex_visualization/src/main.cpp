/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/SCHAddon.h>

#include <mc_rtc/ros.h>

#include <sch/S_Polyhedron/S_Polyhedron.h>

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>

namespace
{
template<typename T>
void getParam(ros::NodeHandle & n, const std::string & param, T & out)
{
  std::string true_param;
  n.searchParam(param, true_param);
  n.getParam(true_param, out);
}

std::vector<std::string> robotParam(ros::NodeHandle & n)
{
  std::string robot_str = "";
  getParam(n, "robot", robot_str);
  if(robot_str.size() == 0)
  {
    return {};
  }
  if(robot_str[0] == '[')
  {
    std::vector<std::string> ret = {""};
    for(size_t i = 1; i < robot_str.size(); ++i)
    {
      const auto & c = robot_str[i];
      if(c == ',')
      {
        ret.push_back("");
      }
      else if(c != ']' && c != ' ')
      {
        ret.back() += c;
      }
    }
    return ret;
  }
  return {robot_str};
}

visualization_msgs::MarkerArray convexMarkers(const std::string & tf_prefix,
                                              const mc_rbdyn::RobotModulePtr & robotModule,
                                              const std::vector<std::string> & filtered_convexes)
{
  visualization_msgs::MarkerArray markers;
  unsigned id = 0;
  for(const auto & col : robotModule->convexHull())
  {
    if(std::find(filtered_convexes.begin(), filtered_convexes.end(), col.first) != filtered_convexes.end())
    {
      continue;
    }
    const std::string & frame = col.second.first;
    sch::S_Polyhedron * poly = sch::mc_rbdyn::Polyhedron(col.second.second);
    sva::PTransformd colTrans = sva::PTransformd::Identity();
    if(robotModule->collisionTransforms().count(frame))
    {
      colTrans = robotModule->collisionTransforms().at(frame);
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
    for(unsigned int i = 0; i < triangles.size(); i++)
    {
      const auto a = vertexes[triangles[i].a]->getCoordinates();
      const auto b = vertexes[triangles[i].b]->getCoordinates();
      const auto c = vertexes[triangles[i].c]->getCoordinates();
      sva::PTransformd va(Eigen::Vector3d(a[0], a[1], a[2]));
      sva::PTransformd vb(Eigen::Vector3d(b[0], b[1], b[2]));
      sva::PTransformd vc(Eigen::Vector3d(c[0], c[1], c[2]));
      va = va * colTrans;
      vb = vb * colTrans;
      vc = vc * colTrans;
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
  return markers;
}

mc_rbdyn::RobotModulePtr rmFromParam(const std::vector<std::string> & robot_params)
{
  if(robot_params.size() == 1)
  {
    return mc_rbdyn::RobotLoader::get_robot_module(robot_params[0]);
  }
  else if(robot_params.size() == 2)
  {
    return mc_rbdyn::RobotLoader::get_robot_module(robot_params[0], robot_params[1]);
  }
  else if(robot_params.size() == 3)
  {
    return mc_rbdyn::RobotLoader::get_robot_module(robot_params[0], robot_params[1], robot_params[2]);
  }
  else if(robot_params.size() > 3)
  {
    ROS_ERROR_STREAM("Invalid robot_params size passed to mc_surfaces_visualization: " << robot_params.size());
  }
  return nullptr;
}

} // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "mc_convex_visualization");

  ros::NodeHandle n;
  ros::NodeHandle n_private("~");

  std::vector<std::string> robot_module = {};
  getParam(n, "robot_module", robot_module);
  std::vector<std::string> robot_param = robotParam(n_private);
  bool robot_set = robot_param.size() != 0;
  std::vector<std::string> robot_params = robot_set ? robot_param : robot_module;

  std::vector<std::string> filtered_convexes = {};
  getParam(n, "filtered_convexes", filtered_convexes);

  std::string tf_prefix = "";
  getParam(n, "tf_prefix", tf_prefix);
  if(tf_prefix.size() && tf_prefix[tf_prefix.size() - 1] != '/')
  {
    tf_prefix += '/';
  }

  bool publish = false;
  getParam(n_private, "publish", publish);

  ros::Publisher sch_pub = n.advertise<visualization_msgs::MarkerArray>("sch_marker", 1000);

  std::shared_ptr<mc_rbdyn::RobotModule> robotModule = nullptr;
  std::unique_ptr<mc_rtc::RobotPublisher> robot_pub;
  std::unique_ptr<mc_rbdyn::Robots> robots;
  visualization_msgs::MarkerArray markers;
  auto init = [&]() {
    robotModule = rmFromParam(robot_params);
    if(!robotModule)
    {
      return;
    }
    robots.reset(new mc_rbdyn::Robots());
    robots->load(*robotModule);
    markers = convexMarkers(tf_prefix, robotModule, filtered_convexes);
    if(publish)
    {
      robot_pub.reset(new mc_rtc::RobotPublisher(tf_prefix, 50, 0.01));
      robot_pub->init(robots->robot());
    }
  };

  init();

  ros::Rate rate(10);
  while(ros::ok())
  {
    sch_pub.publish(markers);
    if(robot_pub)
    {
      robot_pub->update(0.01, robots->robot(), {});
    }
    getParam(n, "robot_module", robot_module);
    if(!robot_set && robot_module.size() && robot_module != robot_params)
    {
      robot_params = robot_module;
      init();
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
