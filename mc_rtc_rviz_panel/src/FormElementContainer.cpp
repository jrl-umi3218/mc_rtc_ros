/*
 * Copyright 2016-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "FormElementContainer.h"

namespace mc_rtc_rviz
{

struct FormElementHeader : public QWidget
{
  FormElementHeader(const char * name, FormElement * element, QWidget * parent) : QWidget(parent), element_(element)
  {
    layout_ = new QHBoxLayout(this);
    label_ = new QLabel(name);
    layout_->addWidget(label_);
    button_ = new QPushButton("Reset");
    layout_->addWidget(button_);
    button_->hide();
    connect(button_, SIGNAL(clicked(bool)), element, SLOT(unlocked()));
  }

  void update()
  {
    if(element_->locked()) { button_->show(); }
    else { button_->hide(); }
  }

private:
  FormElement * element_;
  QHBoxLayout * layout_;
  QLabel * label_;
  QPushButton * button_;
};

FormElementContainer::FormElementContainer(QWidget * parent)
{
  vlayout_ = new QVBoxLayout(this);
  make_form_layout();
  vlayout_->addWidget(form_);
}

void FormElementContainer::make_form_layout()
{
  for(auto & el : elements_header_)
  {
    if(el) { el->deleteLater(); }
  }
  elements_header_.clear();
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

void FormElementContainer::update()
{
  idx_ = 0;
  if(changed_)
  {
    auto item = vlayout_->takeAt(0);
    make_form_layout();
    changed_ = false;
    item->widget()->deleteLater();
  }
  else
  {
    for(auto & h : elements_header_)
    {
      if(h) { h->update(); }
    }
  }
}

bool FormElementContainer::ready(std::string & msg)
{
  bool ok = true;
  for(auto & el : elements_)
  {
    bool ret = el->can_fill(msg);
    if(!ret) { msg += '\n'; }
    ok = ret && ok;
  }
  return ok;
}

void FormElementContainer::collect(mc_rtc::Configuration & out)
{
  for(auto & el : elements_)
  {
    if(el->ready()) { out.add(el->name(), el->serialize()); }
    for(auto & el : elements_)
    {
      el->unlock();
      el->reset();
    }
  }
}

void FormElementContainer::add_element(FormElement * elementIn)
{
  elements_.push_back(elementIn);
  add_element_to_layout(elementIn);
}

void FormElementContainer::add_element_to_layout(FormElement * elementIn)
{
  for(const auto & el : elements_)
  {
    elementIn->update_dependencies(el);
    el->update_dependencies(elementIn);
  }
  if(elementIn->hidden())
  {
    elements_header_.push_back(nullptr);
    elementIn->hide();
    return;
  }
  auto label_text = elementIn->name();
  auto is_robot_or_required = [&label_text, elementIn]()
  { return elementIn->required() || label_text == "robot" || label_text == "r1" || label_text == "r2"; };
  QFormLayout * layout_ = is_robot_or_required() ? requiredLayout_ : optionalLayout_;
  if(elementIn->required()) { label_text += "*"; }
  auto header = new FormElementHeader(label_text.c_str(), elementIn, form_);
  elements_header_.push_back(header);
  if(!elementIn->spanning()) { layout_->addRow(header, elementIn); }
  else
  {
    if(elementIn->show_name()) { layout_->addRow(header); }
    layout_->addRow(elementIn);
  }
  if(optionalLayout_->rowCount() == 0) { optionalToolBox_->hide(); }
  else { optionalToolBox_->show(); }
}

} // namespace mc_rtc_rviz
