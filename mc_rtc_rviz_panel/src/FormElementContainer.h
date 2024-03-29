/*
 * Copyright 2016-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "FormElement.h"

namespace mc_rtc_rviz
{

struct FormElementHeader;

struct FormElementContainer : public QWidget
{
  FormElementContainer(QWidget * parent, FormElementContainer * parentForm = nullptr, bool array_like = false);

  template<typename T, typename... Args>
  T * element(const std::string & name, Args &&... args);

  void add_element(FormElement * element);

  void update();

  bool ready(std::string & msg) const;

  bool ready() const;

  bool locked() const;

  void unlock();

  void reset();

  void collect(mc_rtc::Configuration & out);

  void remove_element(FormElement * element);

  inline FormElementContainer * parentForm() noexcept { return parentForm_; }

  inline bool empty() const noexcept { return elements_.empty(); }

  inline const FormElement * element() const noexcept
  {
    assert(!empty());
    return elements_[0];
  }

  inline const std::vector<FormElement *> & elements() const noexcept { return elements_; }

  void copy(const FormElementContainer & container);

private:
  void add_element_to_layout(FormElement * element);

  void make_form_layout();

  bool array_like_ = false;
  QVBoxLayout * vlayout_;
  QWidget * form_;
  QFormLayout * requiredLayout_;
  QToolBox * optionalToolBox_;
  QFormLayout * optionalLayout_;
  bool changed_ = true;
  size_t idx_;
  std::vector<FormElementHeader *> elements_header_;
  std::vector<FormElement *> elements_;

  FormElementContainer * parentForm_;
};

template<typename T, typename... Args>
T * FormElementContainer::element(const std::string & name, Args &&... args)
{
  if(elements_.size() <= idx_) { changed_ = true; }
  else
  {
    auto & el = elements_[idx_];
    if(el->type() != typeid(T).hash_code() || el->name() != name)
    {
      changed_ = true;
      elements_.resize(idx_);
    }
    else
    {
      if(!el->locked()) { static_cast<T *>(el)->changed(std::forward<Args>(args)...); }
      idx_ += 1;
      return dynamic_cast<T *>(el);
    }
  }
  elements_.push_back(new T(form_, name, std::forward<Args>(args)...));
  elements_.back()->type(typeid(T).hash_code());
  idx_ += 1;
  return dynamic_cast<T *>(elements_.back());
}

} // namespace mc_rtc_rviz
