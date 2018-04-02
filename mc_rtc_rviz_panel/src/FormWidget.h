#pragma once

#include "ClientWidget.h"

namespace mc_rtc_rviz
{
  struct FormElement;

  struct FormWidget : public ClientWidget
  {
    FormWidget(const ClientWidgetParam & param);

    template<typename T, typename ... Args>
    void element(const std::string & name, Args && ... args)
    {
      element(name, [&]() { return new T(this, name, std::forward<Args>(args)...); });
    }

    void element(const std::string & name, std::function<FormElement*()> make_fn);

    void add_element(FormElement * element);
  private:
    QFormLayout * layout_;
    std::vector<FormElement*> elements_;
  };
}

