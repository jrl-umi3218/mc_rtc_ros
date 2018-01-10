#include "ButtonWidget.h"

ButtonWidget::ButtonWidget(const std::string & name,
                           request_t request)
: button_(new QPushButton(name.c_str()))
{
  layout->addWidget(button_);
  connect(button_, &QPushButton::released,
          this, [request](){
            request({});
          });
}
