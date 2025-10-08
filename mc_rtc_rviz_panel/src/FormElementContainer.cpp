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
    else
    {
      button_->hide();
    }
  }

  FormElement * element_;
  QHBoxLayout * layout_;
  QLabel * label_;
  QPushButton * button_;
};

FormElementContainer::FormElementContainer(QWidget * parent, FormElementContainer * parentForm, bool array_like)
: QWidget(parent), parentForm_(parentForm), array_like_(array_like)
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
  optionalToolBox_->hide();
  vlayout_->insertWidget(0, form_);
  for(auto el : elements_)
  {
    el->setParent(form_);
    add_element_to_layout(el);
  }
}

void FormElementContainer::remove_element(FormElement * element)
{
  auto it = std::find(elements_.begin(), elements_.end(), element);
  if(it == elements_.end()) { return; }
  (*it)->deleteLater();
  size_t idx = std::distance(elements_.begin(), it);
  elements_.erase(it);
  for(; idx < elements_.size(); ++idx) { elements_[idx]->name(std::to_string(idx)); }
  changed_ = true;
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

bool FormElementContainer::ready(std::string & msg) const
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

bool FormElementContainer::ready() const
{
  std::string msg;
  return ready(msg);
}

bool FormElementContainer::locked() const
{
  return std::any_of(elements_.begin(), elements_.end(), [](auto * elem) { return elem->locked(); });
}

void FormElementContainer::unlock()
{
  for(auto & el : elements_) { el->unlock(); }
}

void FormElementContainer::reset()
{
  for(auto & el : elements_) { el->reset(); }
}

void FormElementContainer::collect(mc_rtc::Configuration & out)
{
  for(auto & el : elements_)
  {
    if(el->ready()) { out.add(el->name(), el->serialize()); }
    el->unlock();
    el->reset();
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
  if(array_like_)
  {
    auto remove_button = new QPushButton("-");
    header->layout_->addWidget(remove_button);
    connect(remove_button, &QPushButton::released, this,
            [this, elementIn]()
            {
              remove_element(elementIn);
              update();
            });
  }
  elements_header_.push_back(header);
  if(!elementIn->spanning()) { layout_->addRow(header, elementIn); }
  else
  {
    if(elementIn->show_name()) { layout_->addRow(header); }
    layout_->addRow(elementIn);
  }
  if(optionalLayout_->rowCount() == 0) { optionalToolBox_->hide(); }
  else
  {
    optionalToolBox_->show();
  }
}

void FormElementContainer::copy(const FormElementContainer & other)
{
  for(const auto & e : other.elements_) { add_element(e->clone(this, e->name())); }
  update();
}

} // namespace mc_rtc_rviz
