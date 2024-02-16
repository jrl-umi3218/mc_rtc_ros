/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "Panel.h"

#include <QApplication>
#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
#include <csignal>

#ifdef MC_RTC_ROS_IS_ROS2
#  include <rclcpp/rclcpp.hpp>
#else
#  include <ros/ros.h>
#endif

QApplication * app_ = nullptr;

void signal_handler(int)
{
  if(app_) { app_->exit(); }
}

int main(int argc, char * argv[])
{
#ifdef MC_RTC_ROS_IS_ROS2
  rclcpp::init(argc, argv);
#else
  ros::init(argc, argv, "mc_rtc_gui");
#endif
  QApplication app(argc, argv);
  app.setWindowIcon(QIcon(":/icons/gui.ico"));
  app_ = &app;
  QMainWindow window;
  window.setWindowTitle("mc_rtc GUI");
  window.resize(QDesktopWidget().availableGeometry(&window).size() * 0.7);
  window.setCentralWidget(new mc_rtc_rviz::Panel(&window));
  window.show();
  std::signal(SIGINT, &signal_handler);
  return app.exec();
}
