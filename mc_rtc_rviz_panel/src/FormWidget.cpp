#include "FormWidget.h"

namespace mc_rtc_rviz
{

FormWidget::FormWidget(const ClientWidgetParam & param) : ClientWidget(param)
{
  layout_ = new QFormLayout(this);
  auto button = new QPushButton(name().c_str());
  connect(button, SIGNAL(released()), this, SLOT(released()));
  layout_->addRow(button);
}

void FormWidget::released()
{
  mc_rtc::Configuration out;
  std::string msg;
  bool ok = true;
  for(auto el : elements_)
  {
    bool ret = el->fill(out, msg);
    if(!ret)
    {
      msg += '\n';
    }
    ok = ret && ok;
  }
  if(ok)
  {
    client().send_request(id(), out);
  }
  else
  {
    msg = msg.substr(0, msg.size() - 1); // remove last \n
    QMessageBox::critical(this, (name() + " filling incomplete").c_str(), msg.c_str());
  }
}

void FormWidget::add_element(FormElement * element)
{
  element->update_dependencies(elements_);
  for(auto el : elements_)
  {
    el->update_dependencies(element);
  }
  elements_.push_back(element);
  if(element->isHidden())
  {
    return;
  }
  auto label_text = element->name();
  if(element->required())
  {
    label_text += "*";
  }
  if(!element->spanning())
  {
    layout_->insertRow(layout_->rowCount() - 1, label_text.c_str(), element);
  }
  else
  {
    if(element->show_name())
    {
      layout_->insertRow(layout_->rowCount() - 1, new QLabel(label_text.c_str(), this));
    }
    layout_->insertRow(layout_->rowCount() - 1, element);
  }
}

} // namespace mc_rtc_rviz
