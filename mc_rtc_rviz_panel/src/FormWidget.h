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

  template<typename T, typename... Args>
  void element(const std::string & name, Args &&... args);

  void add_element(FormElement * element);

private:
  QFormLayout * layout_;
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
  for(const auto & el : elements_)
  {
    if(el->name() == name)
    {
      return;
    }
  }
  add_element(new T(this, name, std::forward<Args>(args)...));
}
} // namespace mc_rtc_rviz
