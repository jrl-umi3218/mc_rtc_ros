#include "FormWidget.h"

#include <QFormLayout>

#include <iostream>

FormWidget::FormWidget(QWidget * parent, const mc_rtc::Configuration & data,
    request_t request)
: BaseWidget(new QFormLayout(), parent),
  request_(request),
  layout_(static_cast<QFormLayout*>(layout))
{
  auto elements = data("GUI")("elements", std::map<std::string, mc_rtc::Configuration>{});
  for(const auto & el : elements)
  {
    layout_->addRow(el.first.c_str(), new QLineEdit());
  }
  layout_->addRow(confirm_button_);
  setLayout(layout);
}

