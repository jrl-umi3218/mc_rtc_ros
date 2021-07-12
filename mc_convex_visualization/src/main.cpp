/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/SCHAddon.h>

#include <mc_rtc/ros.h>

#include <sch/S_Object/S_Box.h>
#include <sch/S_Object/S_Cylinder.h>
#include <sch/S_Object/S_Sphere.h>
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

visualization_msgs::Marker initMarker(const std::string & frame_id, const std::string & name, size_t id, int32_t t)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = name;
  marker.id = id;
  marker.type = t;
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
  return marker;
}

void setMarkerPose(visualization_msgs::Marker & marker, const sva::PTransformd & pt)
{
  auto q = Eigen::Quaterniond(pt.rotation().transpose());
  const auto & t = pt.translation();
  marker.pose.orientation.w = q.w();
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.position.x = t.x();
  marker.pose.position.y = t.y();
  marker.pose.position.z = t.z();
}

visualization_msgs::Marker fromPolyhedron(const std::string & frame_id,
                                          const std::string & name,
                                          size_t id,
                                          sch::S_Polyhedron & poly,
                                          const sva::PTransformd & colTrans)
{
  auto marker = initMarker(frame_id, name, id, visualization_msgs::Marker::TRIANGLE_LIST);
  auto & pa = *poly.getPolyhedronAlgorithm();
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
    const auto normal = triangles[i].normal;
    auto cross = (a - b) ^ (a - c);
    auto dot = normal * (cross / cross.norm());
    bool reversePointOrder = dot < 0;
    std::vector<sva::PTransformd> vertexOrder = {va, vb, vc};
    if(reversePointOrder)
    {
      vertexOrder = {vc, vb, va};
    }
    for(size_t i = 0; i < vertexOrder.size(); i++)
    {
      vertexOrder[i] = vertexOrder[i] * colTrans;
      geometry_msgs::Point p;
      p.x = vertexOrder[i].translation().x();
      p.y = vertexOrder[i].translation().y();
      p.z = vertexOrder[i].translation().z();
      marker.points.push_back(p);
    }
  }
  return marker;
}

visualization_msgs::Marker fromBox(const std::string & frame_id,
                                   const std::string & name,
                                   size_t id,
                                   sch::S_Box & box,
                                   const sva::PTransformd & colTrans)
{
  auto marker = initMarker(frame_id, name, id, visualization_msgs::Marker::CUBE);
  auto & scale = marker.scale;
  box.getBoxParameters(scale.x, scale.y, scale.z);
  setMarkerPose(marker, colTrans);
  return marker;
}

visualization_msgs::Marker fromCylinder(const std::string & frame_id,
                                        const std::string & name,
                                        size_t id,
                                        sch::S_Cylinder & cylinder,
                                        const sva::PTransformd & colTrans)
{
  auto marker = initMarker(frame_id, name, id, visualization_msgs::Marker::CYLINDER);
  marker.scale.x = 2 * cylinder.getRadius();
  marker.scale.y = 2 * cylinder.getRadius();
  marker.scale.z = (cylinder.getP2() - cylinder.getP1()).norm();
  auto midP = cylinder.getP1() + (cylinder.getP2() - cylinder.getP1()) * marker.scale.z / 2;
  Eigen::Vector3d midPV3{midP.m_x, midP.m_y, midP.m_z};
  sva::PTransformd cylinderCenter{Eigen::Matrix3d::Identity(), midPV3};
  setMarkerPose(marker, colTrans * cylinderCenter);
  return marker;
}

visualization_msgs::Marker fromSphere(const std::string & frame_id,
                                      const std::string & name,
                                      size_t id,
                                      sch::S_Sphere & sphere,
                                      const sva::PTransformd & colTrans)
{
  auto marker = initMarker(frame_id, name, id, visualization_msgs::Marker::SPHERE);
  marker.scale.x = 2 * sphere.getRadius();
  marker.scale.y = 2 * sphere.getRadius();
  marker.scale.z = 2 * sphere.getRadius();
  setMarkerPose(marker, colTrans);
  return marker;
}

visualization_msgs::MarkerArray convexMarkers(const std::string & tf_prefix,
                                              const mc_rbdyn::Robot & robot,
                                              const std::vector<std::string> & filtered_convexes)
{
  visualization_msgs::MarkerArray markers;
  unsigned id = 0;
  for(const auto & col : robot.convexes())
  {
    if(std::find(filtered_convexes.begin(), filtered_convexes.end(), col.first) != filtered_convexes.end())
    {
      continue;
    }
    const auto & frame = col.second.first;
    sch::S_Object * object = col.second.second.get();
    const auto & colTrans = robot.collisionTransform(col.first);
    if(sch::S_Polyhedron * poly = dynamic_cast<sch::S_Polyhedron *>(object))
    {
      markers.markers.push_back(fromPolyhedron(tf_prefix + frame, col.first, ++id, *poly, colTrans));
    }
    else if(sch::S_Box * box = dynamic_cast<sch::S_Box *>(object))
    {
      markers.markers.push_back(fromBox(tf_prefix + frame, col.first, ++id, *box, colTrans));
    }
    else if(sch::S_Cylinder * cylinder = dynamic_cast<sch::S_Cylinder *>(object))
    {
      markers.markers.push_back(fromCylinder(tf_prefix + frame, col.first, ++id, *cylinder, colTrans));
    }
    else if(sch::S_Sphere * sphere = dynamic_cast<sch::S_Sphere *>(object))
    {
      markers.markers.push_back(fromSphere(tf_prefix + frame, col.first, ++id, *sphere, colTrans));
    }
    else
    {
      mc_rtc::log::warning("Cannot display {} collision object", col.first);
    }
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
    markers = convexMarkers(tf_prefix, robots->robot(), filtered_convexes);
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
      robot_pub->update(0.01, robots->robot());
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
