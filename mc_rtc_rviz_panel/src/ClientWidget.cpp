#include "ClientWidget.h"

namespace mc_rtc_rviz
{

std::string id2name(const WidgetId& id)
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

bool ClientWidget::seen()
{
  if(seen_)
  {
    seen_ = false;
    return true;
  }
  return false;
}

void ClientWidget::addWidget(ClientWidget * w)
{
  throw(std::runtime_error("This is only implemented for container widgets"));
}

void ClientWidget::removeWidget(ClientWidget * w)
{
  throw(std::runtime_error("This is only implemented for container widgets"));
}

}
