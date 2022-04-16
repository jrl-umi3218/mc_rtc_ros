/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "LogRobot.h"

#include <mc_rbdyn/rpy_utils.h>

#include <fstream>

LogRobot::LogRobot(const LogRobot::Configuration & config)
: config_(config), publisher_(config.id + "/", 1 / config.dt, config.dt), robots_(mc_rbdyn::loadRobot(*config.rm))
{
}

void LogRobot::update(const mc_rtc::log::FlatLog & log, size_t i)
{
  q = log.get(config_.configuration, i, q);
  encoders = log.get(config_.encoders, i, encoders);
  if(config_.base.size())
  {
    base = log.get(config_.base, i, base);
  }
  if(config_.base_translation.size())
  {
    base.translation() = log.get(config_.base_translation, i, base.translation());
  }
  if(config_.base_rotation.size())
  {
    if(config_.base_rotation_is_imu)
    {
      if(i == 0)
      {
        X_imu0_0 = {log.get<Eigen::Quaterniond>(config_.base_rotation, i, {1, 0, 0, 0})};
        X_imu0_0 = X_imu0_0.inv();
      }
      sva::PTransformd X_0_imu = {log.get<Eigen::Quaterniond>(config_.base_rotation, i, {1, 0, 0, 0})};
      base.rotation() = (X_0_imu * X_imu0_0).rotation();
    }
    else
    {
      base.rotation() = Eigen::Matrix3d(log.get(config_.base_rotation, i, Eigen::Quaterniond(base.rotation())));
    }
  }
  auto & robot = robots_->robot();
  for(size_t refIdx = 0; refIdx < robot.refJointOrder().size(); ++refIdx)
  {
    const auto & jN = robot.refJointOrder()[refIdx];
    if(robot.hasJoint(jN))
    {
      auto jIndex = robot.jointIndexByName(jN);
      if(robot.mbc().q[jIndex].size() == 1)
      {
        robot.mbc().q[jIndex][0] = q[i];
      }
    }
  }
  robot.posW(base);
  publisher_.update(config_.dt, robot);
}

std::vector<std::string> LogRobot::surfaces() const
{
  std::vector<std::string> ret;
  for(const auto & p : robot().surfaces())
  {
    ret.push_back(p.first);
  }
  return ret;
}

std::vector<std::string> LogRobot::bodies() const
{
  std::vector<std::string> ret;
  for(const auto & b : robot().mb().bodies())
  {
    ret.push_back(b.name());
  }
  return ret;
}
