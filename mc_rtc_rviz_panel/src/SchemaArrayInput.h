/*
 * Copyright 2016-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "Schema.h"

namespace mc_rtc_rviz
{

namespace form
{

/** A specialized form element to create an array of schema */
struct SchemaArrayInput : public FormElement
{
  Q_OBJECT
public:
  SchemaArrayInput(QWidget * parent,
                   const std::string & name,
                   bool required,
                   Schema schema,
                   const mc_rtc::Configuration & data,
                   bool fixed_size,
                   int min_size = 0,
                   int max_size = 256);

  mc_rtc::Configuration serialize() const override;

  void reset() override;

  inline FormElement * clone(QWidget *, const std::string &) const override
  {
    mc_rtc::log::error_and_throw("Not implement for this type");
  }

  inline void fill(const mc_rtc::Configuration &) override
  {
    mc_rtc::log::error_and_throw("Not implement for this type");
  }

private:
  Schema schema_;
  mc_rtc::Configuration data_;
  bool fixed_size_;
  int min_size_;
  int max_size_;
  QGridLayout * layout_ = nullptr;
  QPushButton * add_button_ = nullptr;

  void addItem();
  std::vector<FormElement *> items_;

  void removeItem(FormElement * itm);
protected slots:
  void plusReleased();
  void minusReleased();
  void formToggled(bool);
};

} // namespace form

} // namespace mc_rtc_rviz
