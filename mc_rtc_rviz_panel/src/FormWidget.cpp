/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "FormWidget.h"

namespace mc_rtc_rviz
{

FormWidget::FormWidget(const ClientWidgetParam & param) : ClientWidget(param)
{
  layout_ = new QFormLayout(this);
  auto button = new QPushButton(name().c_str());
  connect(button, SIGNAL(released()), this, SLOT(released()));
  layout_->addRow(button);
}

void FormWidget::update()
{
  idx_ = 0;
  if(changed_)
  {
    while(layout_->rowCount() != 1)
    {
      layout_->removeRow(0);
    }
    for(auto el : elements_)
    {
      add_element_to_layout(el);
    }
    changed_ = false;
  }
}

void FormWidget::released()
{
  mc_rtc::Configuration out;
  std::string msg;
  bool ok = true;
  for(auto & el : elements_)
  {
    bool ret = el->fill(out, msg);
    if(!ret)
    {
      msg += '\n';
    }
    ok = ret && ok;
  }
  if(ok)
  {
    client().send_request(id(), out);
  }
  else
  {
    msg = msg.substr(0, msg.size() - 1); // remove last \n
    QMessageBox::critical(this, (name() + " filling incomplete").c_str(), msg.c_str());
  }
}

void FormWidget::add_element(FormElement * element)
{
  elements_.push_back(element);
  add_element_to_layout(element);
}

void FormWidget::add_element_to_layout(FormElement * element)
{
  for(const auto & el : elements_)
  {
    element->update_dependencies(el);
    el->update_dependencies(element);
  }
  if(element->hidden())
  {
    return;
  }
  auto label_text = element->name();
  if(element->required())
  {
    label_text += "*";
  }
  if(!element->spanning())
  {
    layout_->insertRow(layout_->rowCount() - 1, label_text.c_str(), element);
  }
  else
  {
    if(element->show_name())
    {
      layout_->insertRow(layout_->rowCount() - 1, new QLabel(label_text.c_str(), this));
    }
    layout_->insertRow(layout_->rowCount() - 1, element);
  }
}

} // namespace mc_rtc_rviz
