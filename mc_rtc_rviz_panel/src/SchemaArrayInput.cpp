#include "SchemaArrayInput.h"

namespace mc_rtc_rviz
{

namespace form
{

SchemaArrayInput::SchemaArrayInput(QWidget * parent,
                                   const std::string & name,
                                   bool required,
                                   Schema schema,
                                   const mc_rtc::Configuration & data,
                                   bool fixed_size,
                                   int min_size,
                                   int max_size)
: FormElement(parent, name, required), schema_(schema), data_(data), fixed_size_(fixed_size), min_size_(min_size),
  max_size_(max_size)
{
  spanning_ = true;
  auto mainLayout = new QVBoxLayout(this);
  auto w = new QWidget(this);
  mainLayout->QLayout::addWidget(w);
  layout_ = new QGridLayout(w);
  add_button_ = new QPushButton("+");
  connect(add_button_, SIGNAL(released()), this, SLOT(plusReleased()));
  mainLayout->addWidget(add_button_);
  if(fixed_size_)
  {
    add_button_->hide();
  }
  for(int i = 0; i < min_size_; ++i)
  {
    addItem();
  }
}

mc_rtc::Configuration SchemaArrayInput::serialize() const
{
  mc_rtc::Configuration out;
  out = out.array("DATA", items_.size());
  for(auto & f : items_)
  {
    out.push(f->serialize());
  }
  return out;
}

void SchemaArrayInput::addItem()
{
  FormElement * form;
  if(schema_.is_object())
  {
    form = new form::Form(this, std::to_string(items_.size()), false, schema_.create_form(this, data_), !fixed_size_,
                          false);
    connect(form, SIGNAL(toggled(bool)), this, SLOT(formToggled(bool)));
  }
  else
  {
    form = schema_.create_form(this, data_).at(0);
  }
  layout_->addWidget(form);
  if(!schema_.is_object())
  {
    auto minus = new QPushButton("-");
    layout_->addWidget(minus, layout_->rowCount() - 1, 1);
    connect(minus, SIGNAL(released()), this, SLOT(minusReleased()));
  }
  items_.push_back(form);
  if(items_.size() >= static_cast<size_t>(max_size_))
  {
    add_button_->hide();
  }
  ready_ = true;
}

void SchemaArrayInput::plusReleased()
{
  addItem();
}

void SchemaArrayInput::formToggled(bool)
{
  auto sender = dynamic_cast<form::Form *>(this->sender());
  if(!sender)
  {
    qDebug() << "SchemaArrayInput::formToggled unexpected event sender";
    return;
  }
  if(items_.size() == static_cast<size_t>(min_size_))
  {
    sender->rejectUncheck();
    return;
  }
  removeItem(sender);
}

void SchemaArrayInput::minusReleased()
{
  auto sender = dynamic_cast<FormElement *>(this->sender());
  if(!sender)
  {
    qDebug() << "SchemaArrayInput::minusReleased unexpected event sender";
    return;
  }
  if(items_.size() == static_cast<size_t>(min_size_))
  {
    return;
  }
  removeItem(sender);
}

void SchemaArrayInput::removeItem(FormElement * itm)
{
  auto it = std::find(items_.begin(), items_.end(), itm);
  items_.erase(it);
  itm->deleteLater();
  if(items_.size() < static_cast<size_t>(max_size_))
  {
    add_button_->show();
  }
  if(items_.size() == 0)
  {
    ready_ = false;
  }
}

} // namespace form

} // namespace mc_rtc_rviz
