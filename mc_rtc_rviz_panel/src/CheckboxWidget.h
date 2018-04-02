#pragma once

#include "ClientWidget.h"

namespace mc_rtc_rviz
{
  struct CheckboxWidget : public ClientWidget
  {
    CheckboxWidget(const ClientWidgetParam & param);

    void update(bool b);
  private:
    QCheckBox * cbox_;
  };
}
