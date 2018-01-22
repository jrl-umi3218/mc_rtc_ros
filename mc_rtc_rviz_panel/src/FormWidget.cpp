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
    auto w = new QLineEdit();
    auto name = el.first;
    layout_->addRow(el.first.c_str(), w);
    element_callbacks_.push_back(
      [name,w](mc_rtc::Configuration & out)
      {
      bool ok = false;
        unsigned int uivalue = w->text().toUInt(&ok);
        if(ok)
        {
          out.add(name, uivalue);
        }
        else
        {
          double dvalue = w->text().toDouble(&ok);
          if(ok)
          {
            out.add(name, dvalue);
          }
          else
          {
            out.add(name, w->text().toStdString());
          }
        }
      }
    );
  }
  layout_->addRow(confirm_button_);
  confirm_button_->connect(confirm_button_, &QPushButton::released,
                           this, [this]()
                           {
                           mc_rtc::Configuration req;
                           for(const auto & el : element_callbacks_)
                           {
                            el(req);
                           }
                           request_(req);
                           });
  setLayout(layout);
}

