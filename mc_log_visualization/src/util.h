#pragma once

#include <geometry_msgs/PolygonStamped.h>
#include <geos/geom/CoordinateSequence.h>
#include <geos/geom/LinearRing.h>
#include <geos/geom/Polygon.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Core>
#include <chrono>
#include <iostream>

#define MEASURE_TIME(expr)                                                             \
  {                                                                                    \
    auto start = std::chrono::system_clock::now();                                     \
    expr auto end = std::chrono::system_clock::now();                                  \
    std::chrono::duration<double> dt = end - start;                                    \
    std::cout << "Time to call " #expr << std::endl << dt.count() << "s" << std::endl; \
  }

struct CoMPublisher
{
  CoMPublisher(ros::NodeHandle & nh, const std::string & name = "com_marker") : nh(nh)
  {
    com_pub = nh.advertise<visualization_msgs::Marker>(name, 1);
    msg.ns = "robot";
    msg.id = static_cast<int>(std::hash<std::string>()("com_marker"));
    msg.type = visualization_msgs::Marker::SPHERE;
    msg.action = 0;
    msg.pose.orientation.w = 1.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.scale.x = 0.05;
    msg.scale.y = 0.05;
    msg.scale.z = 0.05;
    msg.color.r = 1.0;
    msg.color.g = 0.0;
    msg.color.b = 0.0;
    msg.color.a = 1.0;
    msg.lifetime = ros::Duration(1);
    msg2 = msg;
    msg2.id = static_cast<int>(std::hash<std::string>()("com_marker2"));
    msg2.color.r = 0.0;
    msg2.color.g = 1.0;
    msg2.color.b = 0.0;
  }

  void publish_com(const Eigen::Vector3d & com)
  {
    update_marker(com);
    com_pub.publish(msg);
    com_pub.publish(msg2);
  }

private:
  void update_marker(const Eigen::Vector3d & com)
  {
    msg.header.seq = seq++;
    msg.header.frame_id = "robot_map";
    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = com.x();
    msg.pose.position.y = com.y();
    msg.pose.position.z = 0; // com.z();
    msg2.header = msg.header;
    msg2.pose.position = msg.pose.position;
    msg2.pose.position.z = com.z();
  }

private:
  ros::NodeHandle & nh;
  ros::Publisher com_pub;
  visualization_msgs::Marker msg;
  visualization_msgs::Marker msg2;
  unsigned int seq = 0;
};

struct PolygonPublisher
{
  PolygonPublisher(ros::NodeHandle & nh) : nh(nh)
  {
    poly_pub = nh.advertise<geometry_msgs::PolygonStamped>("stability_polygon", 10);
  }

  void publish_poly(const std::shared_ptr<geos::geom::Geometry> & geom)
  {
    std::vector<Eigen::Vector3d> poly;
    geos::geom::Polygon * polyIn = dynamic_cast<geos::geom::Polygon *>(geom.get());
    if(polyIn)
    {
      const geos::geom::CoordinateSequence * seq = polyIn->getExteriorRing()->getCoordinates();
      for(size_t i = 0; i < seq->size(); ++i)
      {
        const geos::geom::Coordinate & p = seq->getAt(i);
        poly.emplace_back(p.x, p.y, 0);
      }
    }
    geometry_msgs::PolygonStamped msg;
    msg.header.seq = seq++;
    msg.header.frame_id = "robot_map";
    msg.header.stamp = ros::Time::now();
    msg.polygon.points.reserve(poly.size());
    for(const auto & p : poly)
    {
      geometry_msgs::Point32 pt;
      pt.x = static_cast<float>(p.x());
      pt.y = static_cast<float>(p.y());
      pt.z = static_cast<float>(p.z());
      msg.polygon.points.push_back(pt);
    }
    poly_pub.publish(msg);
  }

private:
  ros::NodeHandle & nh;
  ros::Publisher poly_pub;
  unsigned int seq = 0;
};
