#pragma once

#ifndef Q_MOC_RUN

#include <memory>

#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <rviz/panel.h>
#include <tf/tfMessage.h>
#include <tf/transform_listener.h>

#include <QComboBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QObject>
#include <QSpinBox>
#include <QTabWidget>

#include <mc_control/ControllerClient.h>

#endif

namespace mc_rtc_rviz
{

class Panel: public QWidget, public mc_control::ControllerClient
{
Q_OBJECT
public:
  Panel(QWidget* parent = 0);

  const mc_rtc::Configuration & data() const { return data_; }
private:
  void handle_gui_state(const char * data, size_t size) override;
  /** ROS stuff */
  ros::NodeHandle nh_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> int_server_;

  /** GUI stuff */
  QLayout * mainLayout;
  std::map<std::string, QWidget*> widgets_;
  std::map<std::string, std::function<void(QWidget*)>> remove_w_;
  void handle_category(const std::string & category,
                       const std::string & name,
                       const std::map<std::string, mc_rtc::Configuration> & items,
                       const mc_rtc::Configuration & data,
                       QWidget * parent,
                       int level,
                       std::vector<std::string> & seen);
  /** Static data available for the GUI elements */
  mc_rtc::Configuration data_;
private slots:
  void handle_gui_state_impl(const char * data);
signals:
  void gotData(const char * data);
};

class MyPanel : public rviz::Panel
{
Q_OBJECT
public:
  MyPanel(QWidget * parent = 0);

  mc_rtc_rviz::Panel * panel;
};

}
