#include "CheckboxWidget.h"

namespace mc_rtc_rviz
{

CheckboxWidget::CheckboxWidget(const ClientWidgetParam & param)
: ClientWidget(param)
{
  auto layout = new QHBoxLayout(this);
  cbox_ = new QCheckBox(name().c_str(), this);
  layout->addWidget(cbox_);
  connect(cbox_, &QCheckBox::toggled,
          this, [this](bool s)
          {
            this->client().send_request(id());
          });
}

void CheckboxWidget::update(bool b)
{
  if(b != cbox_->isChecked())
  {
    auto blocked = cbox_->signalsBlocked();
    cbox_->blockSignals(true);
    cbox_->setChecked(true);
    cbox_->blockSignals(blocked);
  }
}


}
