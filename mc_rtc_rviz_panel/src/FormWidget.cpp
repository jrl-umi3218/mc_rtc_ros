/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "FormWidget.h"

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
  else
  {
    for(auto & h : elements_header_)
    {
      if(h) { h->update(); }
    }
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
    if(!ret) { msg += '\n'; }
    else if(el->ready()) { out.add(el->name(), el->serialize()); }
    ok = ret && ok;
  }
  if(ok)
  {
    client().send_request(id(), out);
    for(auto & el : elements_)
    {
      el->unlock();
      el->reset();
    }
  }
  else
  {
    msg = msg.substr(0, msg.size() - 1); // remove last \n
    QMessageBox::critical(this, (name() + " filling incomplete").c_str(), msg.c_str());
  }
}

void FormWidget::add_element(FormElement * elementIn)
{
  elements_.push_back(elementIn);
  add_element_to_layout(elementIn);
}

void FormWidget::add_element_to_layout(FormElement * elementIn)
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
