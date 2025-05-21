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

RobotModelDisplay::~RobotModelDisplay()
{
  displayGroup()->takeDisplay(robot_model_display);
  displayGroup()->reset();
  delete robot_model_display;
}

void RobotModelDisplay::update(const std::string & robot_name)
{
  std::string params = "[ " + robot_name + "]";
  label_->setText(params.c_str());

  robot_model_display->setName(robot_name.c_str());

  robot_model_display->subProp(description_prop.c_str())
      ->setValue(("/control/" + name() + "/robot_description").c_str());
  robot_model_display->subProp("TF Prefix")->setValue(("control/" + name()).c_str());
}

} // namespace mc_rtc_rviz
