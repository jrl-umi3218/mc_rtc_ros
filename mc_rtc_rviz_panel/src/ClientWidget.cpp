#include "ClientWidget.h"

namespace mc_rtc_rviz
{

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
