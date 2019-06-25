/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "LogPublisher.h"

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "log_visualizer", ros::init_options::NoSigintHandler);
  auto nh = mc_rtc::ROSBridge::get_node_handle();
  if(!nh)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Failed to initialized node handle")
  }
  std::string log = "";
  {
    std::string param;
    nh->searchParam("log", param);
    nh->getParam(param, log);
  }
  mc_rbdyn::RobotModulePtr mod;
  {
    std::vector<std::string> robot_params;
    std::string param;
    nh->searchParam("robot_module", param);
    nh->getParam(param, robot_params);
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
      LOG_ERROR_AND_THROW(std::runtime_error, "log_visualization cannot handle the robot_params it was given")
    }
  }
  double dt = 0.005;
  {
    std::string param = "";
    nh->searchParam("dt", param);
    nh->getParam(param, dt);
  }

  LOG_INFO("Replaying log: " << log)
  LogPublisher appli(*nh, log, mod, dt);
  appli.run();

  return 0;
}
