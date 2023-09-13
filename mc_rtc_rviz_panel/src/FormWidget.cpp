/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "FormWidget.h"

namespace mc_rtc_rviz
{

FormWidget::FormWidget(const ClientWidgetParam & param) : ClientWidget(param)
{
  container_ = new FormElementContainer(this);
  auto layout = new QVBoxLayout(this);
  layout->addWidget(container_);
  auto button = new QPushButton(name().c_str());
  connect(button, SIGNAL(released()), this, SLOT(released()));
  layout->addWidget(button);
}

void FormWidget::update()
{
  container_->update();
}

void FormWidget::released()
{
  std::string msg;
  bool ok = container_->ready(msg);
  if(ok)
  {
    mc_rtc::Configuration out;
    container_->collect(out);
    client().send_request(id(), out);
  }
  else
  {
    msg = msg.substr(0, msg.size() - 1); // remove last \n
    QMessageBox::critical(this, (name() + " filling incomplete").c_str(), msg.c_str());
  }
}

} // namespace mc_rtc_rviz
