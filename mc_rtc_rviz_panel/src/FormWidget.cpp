/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "FormWidget.h"

namespace mc_rtc_rviz
{

FormWidget::FormWidget(const ClientWidgetParam & param) : ClientWidget(param)
{
  vlayout_ = new QVBoxLayout(this);
  make_form_layout();
  auto button = new QPushButton(name().c_str());
  connect(button, SIGNAL(released()), this, SLOT(released()));
  vlayout_->addWidget(form_);
  vlayout_->addWidget(button);
}

void FormWidget::make_form_layout()
{
  form_ = new QWidget();
  auto formLayout = new QVBoxLayout(form_);
  auto requiredWidget = new QWidget(form_);
  requiredLayout_ = new QFormLayout(requiredWidget);
  formLayout->addWidget(requiredWidget);
  optionalToolBox_ = new QToolBox(form_);
  formLayout->addWidget(optionalToolBox_);
  auto optionalWidget = new QWidget(form_);
  optionalLayout_ = new QFormLayout(optionalWidget);
  optionalToolBox_->addItem(optionalWidget, "Optional fields");
  vlayout_->insertWidget(0, form_);
  for(auto el : elements_)
  {
    el->setParent(form_);
    add_element_to_layout(el);
  }
}

void FormWidget::update()
{
  idx_ = 0;
  if(changed_)
  {
    auto item = vlayout_->takeAt(0);
    make_form_layout();
    changed_ = false;
    item->widget()->deleteLater();
  }
}

void FormWidget::released()
{
  mc_rtc::Configuration out;
  std::string msg;
  bool ok = true;
  for(auto & el : elements_)
  {
    bool ret = el->can_fill(msg);
    if(!ret)
    {
      msg += '\n';
    }
    else if(el->ready())
    {
      out.add(el->name(), el->serialize());
    }
    ok = ret && ok;
  }
  if(ok)
  {
    client().send_request(id(), out);
    for(auto & el : elements_)
    {
      el->reset();
    }
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
    element->hide();
    return;
  }
  auto label_text = element->name();
  auto is_robot_or_required = [&label_text, element]() {
    return element->required() || label_text == "robot" || label_text == "r1" || label_text == "r2";
  };
  QFormLayout * layout_ = is_robot_or_required() ? requiredLayout_ : optionalLayout_;
  if(element->required())
  {
    label_text += "*";
  }
  if(!element->spanning())
  {
    layout_->addRow(label_text.c_str(), element);
  }
  else
  {
    if(element->show_name())
    {
      layout_->addRow(new QLabel(label_text.c_str(), form_));
    }
    layout_->addRow(element);
  }
  if(optionalLayout_->rowCount() == 0)
  {
    optionalToolBox_->hide();
  }
  else
  {
    optionalToolBox_->show();
  }
}

} // namespace mc_rtc_rviz
