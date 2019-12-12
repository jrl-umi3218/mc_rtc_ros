/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <rviz/panel.h>

#include "Panel.h"

namespace mc_rtc_rviz
{

class MyPanel : public rviz::Panel
{
  Q_OBJECT
public:
  MyPanel(QWidget * parent = 0);

  virtual ~MyPanel() override;

  mc_rtc_rviz::Panel * panel;
};

} // namespace mc_rtc_rviz
