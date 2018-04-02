#include "ButtonWidget.h"

namespace mc_rtc_rviz
{

ButtonWidget::ButtonWidget(const ClientWidgetParam & param)
: ClientWidget(param)
{
  auto layout = new QHBoxLayout(this);
  auto button = new QPushButton(name().c_str(), this);
  layout->addWidget(button);
  connect(button, &QPushButton::released,
          this, [this]()
          {
            this->client().send_request(id());
          });
}

}
