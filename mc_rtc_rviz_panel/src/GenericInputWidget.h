#pragma once

#include "ClientWidget.h"

namespace mc_rtc_rviz
{

template<typename T>
struct GenericInputWidget : public ClientWidget
{
  GenericInputWidget(const ClientWidgetParam & param);

  void update(const T & data);
private:
  void set_validator();

  void to_edit(const T & data);

  T from_edit();

  QPushButton * lock_button_;
  QLineEdit * edit_;
};

using StringInputWidget = GenericInputWidget<std::string>;
using IntegerInputWidget = GenericInputWidget<int>;
using NumberInputWidget = GenericInputWidget<double>;

template<typename T>
GenericInputWidget<T>::GenericInputWidget(const ClientWidgetParam & param)
: ClientWidget(param)
{
  setLayout(new QHBoxLayout());
  layout()->addWidget(new QLabel(name().c_str()));
  lock_button_ = new QPushButton("🔒");
  lock_button_->setCheckable(true);
  connect(lock_button_, &QPushButton::toggled,
    this, [this](bool unlocked)
    {
      if(unlocked)
      {
        lock_button_->setText("🔓");;
        edit_->setReadOnly(false);
      }
      else
      {
        client().send_request(id(), from_edit());
        lock_button_->setText("🔒");
        edit_->setReadOnly(true);
      }
    }
  );

  layout()->addWidget(lock_button_);
  edit_ = new QLineEdit(this);
  edit_->setReadOnly(true);
  set_validator();
  connect(edit_, &QLineEdit::returnPressed,
          this, [this]() { if(lock_button_->isChecked()) { lock_button_->toggle(); } });
  layout()->addWidget(edit_);
}

template<typename T>
void GenericInputWidget<T>::update(const T & data)
{
  if(!lock_button_->isChecked())
  {
    to_edit(data);
  }
}

template<typename T>
void GenericInputWidget<T>::to_edit(const T & data)
{
  edit_->setText(QVariant(data).toString());
}

template<>
void GenericInputWidget<std::string>::to_edit(const std::string & data);

template<typename T>
void GenericInputWidget<T>::set_validator() {}

template<>
void GenericInputWidget<double>::set_validator();
template<>
void GenericInputWidget<int>::set_validator();

template<>
std::string GenericInputWidget<std::string>::from_edit();
template<>
int GenericInputWidget<int>::from_edit();
template<>
double GenericInputWidget<double>::from_edit();


}
