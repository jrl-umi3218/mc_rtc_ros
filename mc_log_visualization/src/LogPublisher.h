#pragma once

#include "LogRobot.h"

#include <mc_control/ControllerServer.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/GUIState.h>
#include <mc_rtc/log/FlatLog.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>

#include <ros/ros.h>

#include <fstream>
#include <memory>
#include <thread>

/** Publish an mc_rtc binary log using the GUI facilities */
struct LogPublisher
{
public:
  /** Constructor */
  LogPublisher(ros::NodeHandle & nh, const std::string & logfile, mc_rbdyn::RobotModulePtr mod, double dt);

  /** Start the publication thread */
  void pubThread();

  /** Add a button to the GUI to remove a log entry that has been added to the GUI
   *
   * \param section Section of the configuration where the data is stored
   *
   * \param entry Entry in the log
   */
  void addRemoveExtraDataButton(const std::string & section, const std::string & entry);

  /** Add a 3D point from a Vector3d log entry to the GUI */
  void addVector3dAsPoint3D(const std::string & entry);

  /** Add a force recording from the log to the GUI */
  void addForce(const std::string & entry, const std::string & surface);

  /** Get the filesystem path to the configuration */
  bfs::path configPath() const;

  /** Load the configuration and restore GUI elements */
  void loadConfig();

  /** Save the configuration to disk */
  void saveConfig() const;

  /** Update the GUI */
  void rebuildGUI();

  /** Select a time range for the publication */
  bool selectTime(const std::string & t);

  /** Run the main application */
  void run();

private:
  /** Used to store/load the extra data provided by the log */
  mc_rtc::Configuration config;
  /* ROS */
  ros::NodeHandle & nh;
  std::shared_ptr<mc_rbdyn::RobotModule> mod;

  bool running = true;
  std::thread pub_th;

  /** Log data */
  mc_rtc::log::FlatLog log;

  /** Time parameters */
  double dt;
  double rate;
  ros::Rate rt;

  /** Play/pause playback */
  bool paused = false;
  /** Play in reverse */
  bool reversed = false;
  /** Playback speed numerator */
  unsigned int playback_num = 1;
  /** Playback speed denominator */
  unsigned int playback_den = 1;
  /* min/max/current playback index/time */
  double min_t = 0;
  size_t min_i = 0;
  double max_t = 0;
  size_t max_i = 0;
  double cur_t = 0;
  size_t cur_i = 0;
  /* Current time data */
  std::string t_data;
  /* Current extra entry menu currently selected */
  std::string extra_selected = "";

  std::unique_ptr<LogRobot> robot;
  std::unique_ptr<LogRobot> real_robot;

  /* UI related */
  mc_rtc::gui::StateBuilder gui;
  std::unique_ptr<mc_control::ControllerServer> server;
  const std::vector<std::string> extraDataCategory = {"Log visualizer - Extra data"};
  const std::vector<std::string> extraDataRemoveCategory = {"Log visualizer - Extra data", "Remove"};
};
