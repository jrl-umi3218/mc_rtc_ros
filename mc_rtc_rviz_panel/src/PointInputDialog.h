#pragma once

#include <QGridLayout>

#include <QPushButton>
#include <QLabel>
#include <QLineEdit>

#include <mc_rtc/Configuration.h>

#include "BaseWidget.h"

/** A generic editable vector dialog */
struct PointInputDialog : public QWidget
{
  PointInputDialog(const std::string & name,
                   const std::vector<std::string> & labels,
                   bool editable, bool stacked,
                   request_t request,
                   double min = -std::numeric_limits<double>::infinity(),
                   double max = std::numeric_limits<double>::infinity());
  void update(const mc_rtc::Configuration & data_in);
  request_t request;
  QGridLayout * layout;
  QLabel * name_label;
  std::vector<QLabel*> labels;
  std::vector<QLineEdit*> data;
  QPushButton * button;
  void toggled(bool checked);
  void returnPressed();
};
