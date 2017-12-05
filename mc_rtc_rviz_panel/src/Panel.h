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

class Panel: public rviz::Panel, public mc_control::ControllerClient
{
Q_OBJECT
public:
  Panel(QWidget* parent = 0);
private:
  void handle_gui_state(const char * data, size_t size);
  QTabWidget tabW_;
  std::map<std::string, QWidget*> tabs_;
  std::map<std::string, int> tabs_index_;
private slots:
  void handle_gui_state_impl(const char * data);
signals:
  void gotData(const char * data);
};

}
