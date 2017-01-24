#include <iostream>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/SCHAddon.h>
#include <mc_rbdyn/CylindricalSurface.h>
#include <mc_rbdyn/PlanarSurface.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "surfaces_visualization");

  po::options_description desc("Allowed options");
  desc.add_options()("help", "produce help message")
                    ("pkgpth", po::value<std::string>(), "package path for the environment")
                    ("name", po::value<std::string>(), "robot name");
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  ros::NodeHandle n;
  std::vector<std::string> robot_params = {"env", "/home/herve/ros/catkin_workspace/src/multi_contact/mc_int_obj_description", "door"};

  if(vm.count("pkgpth"))
  {
    robot_params[1] = vm["pkgpth"].as<std::string>();
  }

  if(vm.count("name"))
  {
    robot_params[2] = vm["name"].as<std::string>();
  }

  std::string tf_prefix = "";

  ros::Publisher cylinder_pub = n.advertise<visualization_msgs::MarkerArray>("surfaces", 1000);

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
    ROS_ERROR_STREAM("Invalid robot_params size passed to sch_visualization: " << robot_params.size());
  }

  auto robot = mc_rbdyn::loadRobot(*robotModule, robotModule->rsdf_dir);

  tf::TransformBroadcaster br;
  std::vector<tf::StampedTransform> transforms;

  visualization_msgs::MarkerArray markers;
  unsigned id = 0;
  for(const auto & pair : robot->robots()[0].surfaces())
  {

    const auto surf = pair.second;
    const std::string & frame = surf->name();

    tf::Transform tf;
    std::string surf_name = tf_prefix + "surfaces/" + frame;

    const sva::PTransformd& pt = surf->X_b_s();
    tf.setOrigin(tf::Vector3(pt.translation().x(),
                             pt.translation().y(),
                             pt.translation().z()));

    Eigen::Quaterniond ori(pt.rotation().transpose());
    tf.setRotation(tf::Quaternion(ori.x(), ori.y(), ori.z(), ori.w()));

    tf::StampedTransform stf(tf, ros::Time::now(), tf_prefix + surf->bodyName(), surf_name);

    transforms.push_back(stf);

    visualization_msgs::Marker marker;
    marker.header.frame_id = surf_name;
    marker.header.stamp = ros::Time();
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
      mc_rbdyn::CylindricalSurface* cs = dynamic_cast<mc_rbdyn::CylindricalSurface*>(surf.get());
      marker.scale.x = 2*cs->radius();
      marker.scale.y = 2*cs->radius();
      marker.scale.z = cs->width();
      Eigen::Quaterniond new_ori = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI/2., Eigen::Vector3d::UnitY()));
      marker.pose.orientation.x = new_ori.x();
      marker.pose.orientation.y = new_ori.y();
      marker.pose.orientation.z = new_ori.z();
      marker.pose.orientation.w = new_ori.w();
    }
    if(surf->type() == "planar")
    {
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      const std::vector<std::pair<double, double>>& points =
        dynamic_cast<mc_rbdyn::PlanarSurface*>(surf.get())->planarPoints();
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      geometry_msgs::Point p;
      for(const auto& point : points)
      {
        p.x = point.first;
        p.y = point.second;
        p.z = 0.0;
        marker.points.push_back(p);
      }
      p.x = points[0].first;
      p.y = points[0].second;
      p.z = 0.0;
      marker.points.push_back(p);

      marker.scale.x = 0.01;
    }
    markers.markers.push_back(marker);
  }

  ros::Rate rate(10);
  while(ros::ok())
  {
    cylinder_pub.publish(markers);
    for(auto& stf : transforms)
    {
      stf.stamp_ = ros::Time::now();
      br.sendTransform(stf);
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
