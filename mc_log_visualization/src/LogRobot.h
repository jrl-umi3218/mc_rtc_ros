#pragma once

#include <mc_control/generic_gripper.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rtc/log/FlatLog.h>
#include <mc_rtc/ros.h>

/** Publish a robot state based on log data*/
struct LogRobot
{
  struct Configuration
  {
    /** RobotModule used for robots initialization */
    mc_rbdyn::RobotModulePtr rm;
    /** Timestep of the log */
    double dt;
    /** Identifier and TF prefix */
    std::string id;
    /** Configuration entry in the log */
    std::string configuration;
    /** Encoders entry in the log */
    std::string encoders;
    /** PTransfomd entry used for floating-base estimation */
    std::string base;
    /** If not empty, use this as floating-base rotation */
    std::string base_rotation;
    /** If not empty, use this as floating-base translation */
    std::string base_translation;
    /** If true, interpret base_rotation as an IMU reading */
    bool base_rotation_is_imu = false;
  };

  /** Create a LogRobot with the provided configuration */
  LogRobot(const Configuration & config);

  void update(const mc_rtc::log::FlatLog & log, size_t i);

private:
  Configuration config_;
  mc_rtc::RobotPublisher publisher_;
  std::shared_ptr<mc_rbdyn::Robots> robots_;
  std::vector<std::string> gripperJoints_;
  sva::PTransformd base;
  std::vector<double> q;
  std::vector<double> encoders;
  // Inverse of initial IMU orientation
  sva::PTransformd X_imu0_0;
};
