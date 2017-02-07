#include "ContactForcePublisher.h"

namespace
{
  geometry_msgs::Point pointFromVector(const Eigen::Vector3d & p)
  {
    geometry_msgs::Point ret;
    ret.x = p.x();
    ret.y = p.y();
    ret.z = p.z();
    return ret;
  }

  visualization_msgs::Marker makeForceMarker(
    const std::string & ns,
    Eigen::Vector3d start, Eigen::Vector3d end,
    const std::string & bodyName,
    const ros::Time & time, unsigned int id,
    const std::string & tf_prefix)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = tf_prefix + "/" + bodyName;
    marker.header.stamp = time;
    marker.ns = ns;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = id;
    marker.lifetime = ros::Duration(0.5);
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.points.emplace_back(pointFromVector(start));
    marker.points.emplace_back(pointFromVector(end));
    return marker;
  }

  visualization_msgs::Marker makeForceNormMarker(
    const std::string & ns,
    double norm,
    const std::string & bodyName,
    const ros::Time & time, unsigned int id,
    const std::string & tf_prefix)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = tf_prefix + "/" + bodyName;
    marker.header.stamp = time;
    marker.ns = ns;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.id = id;
    marker.lifetime = ros::Duration(0.5);
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    std::stringstream ss;
    ss << std::setprecision(3) << norm;
    marker.text = ss.str();
    return marker;
  }
}

namespace mc_rtc_ros
{

ContactForcePublisher::ContactForcePublisher(ros::NodeHandle & nh,
                                             mc_control::MCGlobalController & gc)
: nh(nh), gc(gc)
{
  rate = floor(1/this->gc.configuration().publish_timestep);
  update_th = std::thread([this]()
                          {
    ros::Rate rt(2*rate);
    while(running)
    {
      if(update_ready)
      {
        for(const auto & pub : force_markers_publisher)
        {
          const auto & pub_key = pub.first;
          auto & publisher = force_markers_publisher[pub_key];
          auto & array = force_markers[pub_key];
          publisher.publish(array);
          auto & norm_publisher = force_norm_markers_publisher[pub_key];
          auto & norm_array = force_norm_markers[pub_key];
          norm_publisher.publish(norm_array);
        }
        update_ready = false;
      }
      rt.sleep();
    }
                          });
}

void ContactForcePublisher::stop()
{
  running = false;
}

void ContactForcePublisher::update()
{
  if(iter++ % rate != 0)
  {
    return;
  }
  auto compute_mass = [](const mc_rbdyn::Robot & r)
  {
    double mass = 0;
    for(const auto & b : r.mb().bodies())
    {
      mass += b.inertia().mass();
    }
    return mass == 0 ? 1 : mass;
  };
  auto get_tf_prefix = [](const mc_rbdyn::Robots & robots, unsigned int idx)
  {
    if(robots.robotIndex() == idx) { return std::string("control"); }
    std::stringstream ss;
    ss << "control/env_" << idx;
    return ss.str();
  };
  auto get_pub_key = [this,&get_tf_prefix](const mc_rbdyn::Robots robots, unsigned int idx)
  {
    std::stringstream ss;
    ss << get_tf_prefix(robots, idx) << "/contact_force";
    return ss.str();
  };
  const auto & robots = gc.controller().robots();
  const auto & res = gc.controller().send(0);
  for(size_t i = 0; i < robots.robots().size(); ++i)
  {
    auto pub_key = get_pub_key(robots, i);
    if(force_markers_publisher.count(pub_key) == 0)
    {
      force_markers_publisher[pub_key] = nh.advertise<visualization_msgs::MarkerArray>(pub_key, 100);
      std::string pub_norm = pub_key + "_norm";
      force_norm_markers_publisher[pub_key] = nh.advertise<visualization_msgs::MarkerArray>(pub_norm, 100);
    }
    force_markers[pub_key].markers.resize(0);
    force_norm_markers[pub_key].markers.resize(0);
  }
  auto time = ros::Time::now();
  unsigned int id = 0;
  for(size_t ci = 0; ci < res.contacts.size(); ++ci)
  {
    const auto & c = res.contacts[ci];
    const auto & r1 = robots.robots()[c.r1_index];
    const auto & r2 = robots.robots()[c.r2_index];
    auto r1_pub_key = get_pub_key(robots, c.r1_index);
    auto r2_pub_key = get_pub_key(robots, c.r2_index);
    mc_rbdyn::Contact contact(robots, c.r1_index, c.r2_index,
                              c.r1_surface, c.r2_surface);
    auto qp_contact = gc.controller().solver().contactById(contact.contactId(robots));
    if(qp_contact.first != -1)
    {
      const auto & qp_c = qp_contact.second;
      const auto & r1Points = qp_c.r1Points;
      const auto & r2Points = qp_c.r2Points;
      Eigen::Vector3d r1F = Eigen::Vector3d::Zero();
      Eigen::Vector3d r2F = Eigen::Vector3d::Zero();
      auto pos = res.contacts_lambda_begin[ci];
      std::string r1_tf_prefix = get_tf_prefix(robots, c.r1_index);
      std::string r2_tf_prefix = get_tf_prefix(robots, c.r2_index);
      for(unsigned int i = 0; i < r1Points.size(); ++i)
      {
        auto lambda = res.lambdaVec.segment(pos, qp_c.nrLambda(i));
        auto r1Dir = qp_c.force(lambda, i, qp_c.r1Cones);
        auto r2Dir = qp_c.force(lambda, i, qp_c.r2Cones);
        pos += qp_c.nrLambda(i);
        r1F += r1Dir;
        r2F += r2Dir;
        auto r1Start = r1Points[i];
        auto r1End = r1Start + r1Dir * (1/(9.81*compute_mass(r1)));
        auto r2Start = r2Points[i];
        auto r2End = r2Start + r2Dir * (1/(9.81*compute_mass(r2)));
        force_markers[r1_pub_key].markers.emplace_back(
                                    makeForceMarker(r1_pub_key,
                                                    r1Start, r1End,
                                                    c.r1_body,
                                                    time, ++id,
                                                    r1_tf_prefix));
        force_markers[r2_pub_key].markers.emplace_back(
                                    makeForceMarker(r2_pub_key,
                                                    r2Start, r2End,
                                                    c.r2_body,
                                                    time, ++id,
                                                    r2_tf_prefix));
      }
      force_norm_markers[r1_pub_key].markers.emplace_back(
                                  makeForceNormMarker(r1_pub_key,
                                                      r1F.norm(),
                                                      c.r1_body,
                                                      time, ++id,
                                                      r1_tf_prefix));
      force_norm_markers[r2_pub_key].markers.emplace_back(
                                  makeForceNormMarker(r2_pub_key,
                                                      r2F.norm(),
                                                      c.r2_body,
                                                      time, ++id,
                                                      r2_tf_prefix));
    }
  }
  update_ready = true;
}

}
