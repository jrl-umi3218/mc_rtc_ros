#include "ComboInputWidget.h"

namespace
{

std::vector<std::string> ref2values(const mc_rtc::Configuration & data,
                                    const std::vector<std::string> & ref)
{
  auto d = data;
  for(const auto & k : ref)
  {
    d = d(k, mc_rtc::Configuration{});
  }
  return d.size() ? d : std::vector<std::string>{};
}

}

namespace mc_rtc_rviz
{

ComboInputWidget::ComboInputWidget(const ClientWidgetParam & param,
                                   const mc_rtc::Configuration & data,
                                   const std::vector<std::string> & ref)
: ComboInputWidget(param, ref2values(data, ref))
{
}

ComboInputWidget::ComboInputWidget(const ClientWidgetParam & param,
                                   const std::vector<std::string> & values)
: ClientWidget(param)
{
  auto layout = new QVBoxLayout(this);
  combo_ = new QComboBox(this);
  layout->addWidget(combo_);
  for(const auto & v : values)
  {
    combo_->addItem(v.c_str());
  }
  connect(combo_, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
          this, [this](int idx)
          {
            if(idx == -1) return;
            client().send_request(id(), combo_->currentText().toStdString());
          });
}

void ComboInputWidget::update(const std::string & data)
{
  auto blocked = combo_->signalsBlocked();
  combo_->blockSignals(true);
  combo_->setCurrentText(data.c_str());
  combo_->blockSignals(blocked);
}

}
