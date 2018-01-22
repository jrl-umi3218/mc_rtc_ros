#include "InputWidget.h"

InputWidget::InputWidget(const std::string & name,
                         const mc_rtc::Configuration & data,
                         request_t request)
: BaseWidget(new QVBoxLayout())
{
  input = new PointInputDialog(name,
      data("GUI", mc_rtc::Configuration{})("labels", std::vector<std::string>{}),
      data.has("SET"),
      false,
      request);
  layout->addWidget(input);
}

void InputWidget::update(const mc_rtc::Configuration & data)
{
  input->update(data);
}
