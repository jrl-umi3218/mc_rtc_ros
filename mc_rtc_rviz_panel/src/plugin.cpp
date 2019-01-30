#include "plugin.h"

namespace mc_rtc_rviz
{

MyPanel::MyPanel(QWidget * parent) : rviz::Panel(parent)
{
  auto layout = new QVBoxLayout();
  panel = new mc_rtc_rviz::Panel(parent);
  layout->QLayout::addWidget(panel);
  setLayout(layout);
}

MyPanel::~MyPanel() {}

} // namespace mc_rtc_rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mc_rtc_rviz::MyPanel, rviz::Panel)
