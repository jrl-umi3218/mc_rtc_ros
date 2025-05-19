/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifdef MC_RTC_ROS_IS_ROS2
#  include <rviz_common/panel.hpp>
using PanelBase = rviz_common::Panel;
#else
#  include <rviz/panel.h>
#  include <rviz/render_panel.h>
#  include <rviz/view_manager.h>
#  include <rviz/visualization_manager.h>
using PanelBase = rviz::Panel;
#endif

#include "Panel.h"

namespace mc_rtc_rviz
{

class MyPanel : public PanelBase
{
  Q_OBJECT
public:
  MyPanel(QWidget * parent = 0);

  void onInitialize() override;

#ifdef MC_RTC_ROS_IS_ROS2
  virtual ~MyPanel();
#else
  virtual ~MyPanel() override;
#endif

  mc_rtc_rviz::Panel * panel;
};

} // namespace mc_rtc_rviz
