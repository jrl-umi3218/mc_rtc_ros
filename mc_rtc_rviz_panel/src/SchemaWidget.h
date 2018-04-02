#pragma once

#include "ClientWidget.h"

namespace mc_rtc_rviz
{

struct SchemaWidget : public ClientWidget
{
  SchemaWidget(const ClientWidgetParam & params, const std::string & schema, const mc_rtc::Configuration & data);
private:
};

}
