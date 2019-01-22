#pragma once

#include "ClientWidget.h"

namespace mc_rtc_rviz
{
  struct LabelWidget : public ClientWidget
  {
    LabelWidget(const ClientWidgetParam & param);

    void update(const std::string & in);
  protected:
    QLabel * label_;
  };
}
