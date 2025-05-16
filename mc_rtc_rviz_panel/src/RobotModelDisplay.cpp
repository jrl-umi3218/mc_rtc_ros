/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "RobotModelDisplay.h"

namespace mc_rtc_rviz
{

RobotModelDisplay::RobotModelDisplay(const ClientWidgetParam & param,
                                     DisplayContext * display_context,
                                     DisplayGroup * display_group)
: ClientWidget(param)
{
  layout_ = new QHBoxLayout(this);
  if(!secret()) { layout_->addWidget(new QLabel(name().c_str(), this)); }
  label_ = new QLabel("", this);
  label_->setWordWrap(true);
  layout_->addWidget(label_);

  setDisplayContext(display_context);
  setDisplayGroup(display_group);

  if(!displayGroup() || !displayContext())
  {
    qWarning("Display group or context not set before init");
    return;
  }

  robot_model_display = displayGroup()->createDisplay(display_class_id.c_str());

  robot_model_display->initialize(displayContext());
  displayGroup()->addChild(robot_model_display);
  robot_model_display->setEnabled(true);
}

void RobotModelDisplay::update(const std::vector<std::string> & in)
{
  std::string params = "[ " + in[0] + "]";
  label_->setText(params.c_str());

  robot_model_display->setName(in[0].c_str());
  robot_model_display->subProp("Description Topic")->setValue(("/" + in[1] + "/robot_description").c_str());
  robot_model_display->subProp("TF Prefix")->setValue(in[1].c_str());
}

} // namespace mc_rtc_rviz
