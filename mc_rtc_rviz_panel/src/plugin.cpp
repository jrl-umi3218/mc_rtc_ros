/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "plugin.h"

namespace mc_rtc_rviz
{

MyPanel::MyPanel(QWidget * parent) : PanelBase(parent)
{
  auto layout = new QVBoxLayout();
  panel = new mc_rtc_rviz::Panel(parent);
  layout->QLayout::addWidget(panel);
  setLayout(layout);
}

void MyPanel::onInitialize()
{
  rviz_common::Panel::onInitialize(); 
  rviz_common::DisplayContext* ctx = getDisplayContext();
  if (!ctx)
  {
    qWarning("DisplayContext not available in onInitialize()");
    return;
  }

  panel->setDisplayContext(ctx);
  panel->setDisplayGroup(ctx->getRootDisplayGroup());
}

MyPanel::~MyPanel() {}

} // namespace mc_rtc_rviz

#ifdef MC_RTC_ROS_IS_ROS2
#  include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mc_rtc_rviz::MyPanel, rviz_common::Panel)
#else
#  include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mc_rtc_rviz::MyPanel, rviz::Panel)
#endif
