#pragma once

#include "ClientWidget.h"

namespace mc_rtc_rviz
{

struct CommonInputWidget : public ClientWidget
{
  Q_OBJECT
public:
  CommonInputWidget(const ClientWidgetParam & param);
protected slots:
  virtual void toggled(bool) = 0;
  virtual void returnPressed() = 0;
};

template<typename T>
struct GenericInputWidget : public CommonInputWidget
{
  GenericInputWidget(const ClientWidgetParam & param);

  void update(const T & data);
private:
  void set_validator();

  void to_edit(const T & data);

  T from_edit();

  QPushButton * lock_button_;
  QLineEdit * edit_;
protected: // Override slots
  void toggled(bool unlocked) override;
  void returnPressed() override;
};

using StringInputWidget = GenericInputWidget<std::string>;
using IntegerInputWidget = GenericInputWidget<int>;
using NumberInputWidget = GenericInputWidget<double>;

template<typename T>
GenericInputWidget<T>::GenericInputWidget(const ClientWidgetParam & param)
: CommonInputWidget(param)
{
  setLayout(new QHBoxLayout());
  layout()->addWidget(new QLabel(name().c_str()));
  lock_button_ = new QPushButton("EDIT");
  lock_button_->setCheckable(true);
  connect(lock_button_, SIGNAL(toggled(bool)), this, SLOT(toggled(bool)));
  layout()->addWidget(lock_button_);
  edit_ = new QLineEdit(this);
  edit_->setReadOnly(true);
  set_validator();
  connect(edit_, SIGNAL(returnPressed()), this, SLOT(returnPressed()));
  layout()->addWidget(edit_);
}

template<typename T>
void GenericInputWidget<T>::toggled(bool unlocked)
{
  if(unlocked)
  {
    lock_button_->setText("SEND");
    edit_->setReadOnly(false);
  }
  else
  {
    client().send_request(id(), from_edit());
    lock_button_->setText("EDIT");
    edit_->setReadOnly(true);
  }
}

template<typename T>
void GenericInputWidget<T>::returnPressed()
{
  if(lock_button_->isChecked()) { lock_button_->toggle(); }
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
