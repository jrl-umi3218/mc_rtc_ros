#include <mc_rbdyn/CylindricalSurface.h>
#include <mc_rbdyn/GripperSurface.h>
#include <mc_rbdyn/PlanarSurface.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <tf/transform_broadcaster.h>

namespace
{
template<typename T>
void getParam(ros::NodeHandle & n, const std::string & param, T & out)
{
  std::string true_param;
  n.searchParam(param, true_param);
  n.getParam(true_param, out);
}

tf::StampedTransform tfFromSurface(const std::string & prefix, const mc_rbdyn::Surface & surf)
{
  tf::StampedTransform tf;

  const sva::PTransformd & pt = surf.X_b_s();
  tf.setOrigin(tf::Vector3(pt.translation().x(), pt.translation().y(), pt.translation().z()));

  Eigen::Quaterniond ori(pt.rotation().transpose());
  tf.setRotation(tf::Quaternion(ori.x(), ori.y(), ori.z(), ori.w()));

  tf.frame_id_ = prefix + surf.bodyName();
  tf.child_frame_id_ = prefix + "surfaces/" + surf.name();
  tf.stamp_ = ros::Time::now();

  return tf;
}
} // namespace

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "surfaces_visualization");

  ros::NodeHandle n;

  std::vector<std::string> robot_params = {"env", mc_rtc::MC_ENV_DESCRIPTION_PATH, "door"};
  std::string tf_prefix = "";
  getParam(n, "robot_module", robot_params);
  getParam(n, "tf_prefix", tf_prefix);
  if(tf_prefix.size() && tf_prefix[tf_prefix.size() - 1] != '/')
  {
    tf_prefix += '/';
  }

  ros::Publisher surface_pub = n.advertise<visualization_msgs::MarkerArray>("surfaces", 1000);

  std::shared_ptr<mc_rbdyn::RobotModule> robotModule;

  /* SHAME */
  if(robot_params.size() == 1)
  {
    robotModule = mc_rbdyn::RobotLoader::get_robot_module(robot_params[0]);
  }
  else if(robot_params.size() == 2)
  {
    robotModule = mc_rbdyn::RobotLoader::get_robot_module(robot_params[0], robot_params[1]);
  }
  else if(robot_params.size() == 3)
  {
    robotModule = mc_rbdyn::RobotLoader::get_robot_module(robot_params[0], robot_params[1], robot_params[2]);
  }
  else
  {
    ROS_ERROR_STREAM("Invalid robot_params size passed to mc_surfaces_visualization: " << robot_params.size());
  }

  auto robot = mc_rbdyn::loadRobot(*robotModule);

  tf::TransformBroadcaster br;
  std::vector<tf::StampedTransform> transforms;

  visualization_msgs::MarkerArray markers;
  unsigned id = 0;
  for(const auto & pair : robot->robots()[0].surfaces())
  {

    std::shared_ptr<mc_rbdyn::Surface> surf = pair.second;
    const std::string & frame = surf->name();

    tf::StampedTransform stf = tfFromSurface(tf_prefix, *surf);
    transforms.push_back(stf);

    visualization_msgs::Marker marker;
    marker.header.frame_id = stf.child_frame_id_;
    marker.header.stamp = stf.stamp_;
    marker.ns = frame;
    marker.id = ++id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(0.5);

    if(surf->type() == "cylindrical")
    {
      marker.type = visualization_msgs::Marker::CYLINDER;
      mc_rbdyn::CylindricalSurface * cs = dynamic_cast<mc_rbdyn::CylindricalSurface *>(surf.get());
      marker.scale.x = 2 * cs->radius();
      marker.scale.y = 2 * cs->radius();
      marker.scale.z = cs->width();
      Eigen::Quaterniond new_ori = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2., Eigen::Vector3d::UnitY()));
      marker.pose.orientation.x = new_ori.x();
      marker.pose.orientation.y = new_ori.y();
      marker.pose.orientation.z = new_ori.z();
      marker.pose.orientation.w = new_ori.w();

      markers.markers.push_back(marker);
    }
    else if(surf->type() == "planar")
    {
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      const std::vector<std::pair<double, double>> & points =
          dynamic_cast<mc_rbdyn::PlanarSurface *>(surf.get())->planarPoints();
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      geometry_msgs::Point p;
      for(const auto & point : points)
      {
        p.x = point.first;
        p.y = point.second;
        p.z = 0.0;
        marker.points.push_back(p);
      }
      marker.points.push_back(marker.points[0]);

      marker.scale.x = 0.01;

      markers.markers.push_back(marker);

      auto normal = marker;
      normal.id = ++id;
      normal.ns += "_normal";
      normal.type = visualization_msgs::Marker::ARROW;
      normal.scale.x = 0.01;
      normal.scale.y = 0.02;
      normal.scale.z = 0.1;
      normal.color.g = 0.0;
      normal.color.b = 1.0;
      normal.points.clear();
      p.x = 0;
      p.y = 0;
      p.z = 0;
      normal.points.push_back(p);
      p.x = 0;
      p.y = 0;
      p.z = 0.2;
      normal.points.push_back(p);
      markers.markers.push_back(normal);
    }
    else if(surf->type() == "gripper")
    {
      marker.type = visualization_msgs::Marker::ARROW;
      marker.scale.x = 0.005;
      marker.scale.y = 0.01;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      const std::vector<sva::PTransformd> pFO =
          dynamic_cast<mc_rbdyn::GripperSurface *>(surf.get())->pointsFromOrigin();
      for(const auto & p : pFO)
      {
        auto m = marker;
        m.id = ++id;
        auto p1 = p.translation();
        auto p2 = p1 + p.rotation().col(2) * 0.05;
        geometry_msgs::Point gp;
        gp.x = p1.x();
        gp.y = p1.y();
        gp.z = p1.z();
        m.points.push_back(gp);
        gp.x = p2.x();
        gp.y = p2.y();
        gp.z = p2.z();
        m.points.push_back(gp);
        markers.markers.push_back(m);
      }
    }
  }

  ros::Rate rate(10);
  while(ros::ok())
  {
    auto now = ros::Time::now();
    for(auto & m : markers.markers)
    {
      m.header.stamp = now;
    }
    surface_pub.publish(markers);
    for(auto & stf : transforms)
    {
      stf.stamp_ = now;
    }
    br.sendTransform(transforms);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
