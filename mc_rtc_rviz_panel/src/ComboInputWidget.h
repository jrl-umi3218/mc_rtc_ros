#pragma once

#include "ClientWidget.h"

namespace mc_rtc_rviz
{

struct ComboInputWidget : public ClientWidget
{
  Q_OBJECT
public:
  ComboInputWidget(const ClientWidgetParam & param, const std::vector<std::string> & values);

  ComboInputWidget(const ClientWidgetParam & param, const mc_rtc::Configuration & data, const std::vector<std::string> & ref);

  void update(const std::string & data);
private:
  QComboBox * combo_;
private slots:
  void currentIndexChanged(int);
};

}
