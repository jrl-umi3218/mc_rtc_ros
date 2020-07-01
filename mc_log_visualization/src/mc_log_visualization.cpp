/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "LogPublisher.h"

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
} // namespace

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "log_visualizer", ros::init_options::NoSigintHandler);
  auto nh = mc_rtc::ROSBridge::get_node_handle();
  if(!nh)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Failed to initialized node handle");
  }

  ros::NodeHandle nh_private("~");

  std::string log = "";
  getParam(nh_private, "log", log);

  mc_rbdyn::RobotModulePtr mod;
  {
    std::vector<std::string> robot_params = robotParam(nh_private);
    if(robot_params.size() == 1)
    {
      mod = mc_rbdyn::RobotLoader::get_robot_module(robot_params[0]);
    }
    else if(robot_params.size() == 2)
    {
      mod = mc_rbdyn::RobotLoader::get_robot_module(robot_params[0], robot_params[1]);
    }
    else if(robot_params.size() == 3)
    {
      mod = mc_rbdyn::RobotLoader::get_robot_module(robot_params[0], robot_params[1], robot_params[2]);
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("log_visualization cannot handle the robot_params it was given");
    }
  }
  double dt = 0.005;
  getParam(nh_private, "dt", dt);

  mc_rtc::log::info("Replaying log: {}", log);
  LogPublisher appli(*nh, log, mod, dt);
  appli.run();

  return 0;
}
