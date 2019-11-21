/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "ClientWidget.h"

#include "Panel.h"

namespace mc_rtc_rviz
{

std::string id2name(const WidgetId & id)
{
  std::string ret;
  for(auto & c : id.category)
  {
    ret += c + "/";
  }
  ret += id.name;
  return ret;
}

ClientWidget::ClientWidget(const ClientWidgetParam & params)
: QWidget(params.parent), client_(params.client), id_(params.id)
{
}

void ClientWidget::addWidget(ClientWidget *)
{
  throw(std::runtime_error("This is only implemented for container widgets"));
}

void ClientWidget::removeWidget(ClientWidget *)
{
  throw(std::runtime_error("This is only implemented for container widgets"));
}

bool ClientWidget::visible()
{
  return static_cast<Panel &>(client_).visible(id_);
}

void ClientWidget::visible(bool visibility)
{
  static_cast<Panel &>(client_).visible(id_, visibility);
}

} // namespace mc_rtc_rviz
