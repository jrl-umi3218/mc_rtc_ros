/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ClientWidget.h"

namespace mc_rtc_rviz
{
struct FormElement;

struct FormWidget;

template<typename T, typename... Args>
T * make_new(FormWidget * self, const std::string & name, Args &&... args)
{
  return new T(self, name, std::forward<Args>(args)...);
}

struct FormWidget : public ClientWidget
{
  Q_OBJECT
public:
  FormWidget(const ClientWidgetParam & param);

  void update();

  template<typename T, typename... Args>
  void element(const std::string & name, Args &&... args);

  void add_element(FormElement * element);

private:
  void add_element_to_layout(FormElement * element);

  QVBoxLayout * vlayout_;
  QWidget * form_;
  QFormLayout * layout_;
  bool changed_ = true;
  size_t idx_;
  std::vector<FormElement *> elements_;
private slots:
  void released();
};

} // namespace mc_rtc_rviz

#include "FormElement.h"

namespace mc_rtc_rviz
{

template<typename T, typename... Args>
void FormWidget::element(const std::string & name, Args &&... args)
{
  if(elements_.size() <= idx_)
  {
    changed_ = true;
  }
  else
  {
    auto & el = elements_[idx_];
    if(el->type() != typeid(T).hash_code() || el->name() != name
       || static_cast<T *>(el)->changed(std::forward<Args>(args)...))
    {
      changed_ = true;
      elements_.resize(idx_);
    }
    else
    {
      idx_ += 1;
      return;
    }
  }
  elements_.push_back(new T(form_, name, std::forward<Args>(args)...));
  elements_.back()->type(typeid(T).hash_code());
  idx_ += 1;
}

} // namespace mc_rtc_rviz
