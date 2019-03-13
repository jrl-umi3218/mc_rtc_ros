#include "LogRobot.h"

#include <mc_rbdyn/rpy_utils.h>

#include <fstream>

LogRobot::LogRobot(const LogRobot::Configuration & config)
: config_(config), publisher_(config.id + "/", 1 / config.dt, config.dt)
{
  robots_ = mc_rbdyn::loadRobot(*config.rm);
  std::string urdfPath = config.rm->urdf_path;
  std::ifstream ifs(urdfPath);
  std::stringstream urdf;
  if(ifs.is_open())
  {
    urdf << ifs.rdbuf();
  }
  if(config_.configuration != config_.encoders)
  {
    for(auto & g : config.rm->grippers())
    {
      auto g_ptr = std::make_shared<mc_control::Gripper>(
          robots_->robot(), g.joints, urdf.str(), std::vector<double>(g.joints.size(), 0), config.dt, g.reverse_limits);
      for(auto & j : g_ptr->names)
      {
        gripperJoints_.push_back(j);
      }
    }
  }
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
  for(size_t i = 0; i < robot.refJointOrder().size(); ++i)
  {
    const auto & jN = robot.refJointOrder()[i];
    if(robot.hasJoint(jN))
    {
      auto jIndex = robot.jointIndexByName(jN);
      if(robot.mbc().q[jIndex].size() == 1)
      {
        if(std::find(gripperJoints_.begin(), gripperJoints_.end(), jN) != gripperJoints_.end())
        {
          robot.mbc().q[jIndex][0] = encoders[i];
        }
        else
        {
          robot.mbc().q[jIndex][0] = q[i];
        }
      }
    }
  }
  // base.rotation() = base.rotation().transpose();
  robot.posW(base);
  publisher_.update(config_.dt, robot, {});
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
