#pragma once

#include "ClientWidget.h"

namespace mc_rtc_rviz
{

struct ComboInputWidget : public ClientWidget
{
  Q_OBJECT
public:
  ComboInputWidget(const ClientWidgetParam & param, const std::vector<std::string> & values);

  ComboInputWidget(const ClientWidgetParam & param,
                   const mc_rtc::Configuration & data,
                   const std::vector<std::string> & ref);

  void update(const std::string & data, const std::vector<std::string> & values);

  void update(const std::string & in, const mc_rtc::Configuration & data, const std::vector<std::string> & ref);

private:
  QComboBox * combo_;
  std::vector<std::string> values_;
  void update(const std::vector<std::string> & values);
  void update(const mc_rtc::Configuration & data, const std::vector<std::string> & values);
private slots:
  void currentIndexChanged(int);
};

} // namespace mc_rtc_rviz
