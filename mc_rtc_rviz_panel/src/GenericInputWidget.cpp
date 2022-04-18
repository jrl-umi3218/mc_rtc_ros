/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "GenericInputWidget.h"

namespace mc_rtc_rviz
{

CommonInputWidget::CommonInputWidget(const ClientWidgetParam & param) : ClientWidget(param) {}

template<>
void GenericInputWidget<double>::set_validator()
{
  auto validator = new QDoubleValidator(this);
  validator->setLocale(QLocale::C);
  edit_->setValidator(validator);
}

template<>
void GenericInputWidget<int>::set_validator()
{
  auto validator = new QIntValidator(this);
  validator->setLocale(QLocale::C);
  edit_->setValidator(validator);
}

template<>
void GenericInputWidget<std::string>::to_edit(const std::string & dataIn)
{
  edit_->setText(dataIn.c_str());
}

template<>
std::string GenericInputWidget<std::string>::from_edit()
{
  return edit_->text().toStdString();
}

template<>
int GenericInputWidget<int>::from_edit()
{
  return edit_->text().toInt();
}

template<>
double GenericInputWidget<double>::from_edit()
{
  return edit_->text().toDouble();
}

} // namespace mc_rtc_rviz
