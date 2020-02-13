/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "Panel.h"

namespace mc_rtc_rviz
{

class ConnectionDialog : public QDialog
{
  Q_OBJECT
public:
  ConnectionDialog(std::string & sub_uri, std::string & push_uri, double & timeout, QWidget * parent = nullptr);

  void accept() override;

private:
  std::string & sub_uri_;
  std::string & push_uri_;
  double & timeout_;
  QFormLayout * layout_;
  QLineEdit * subEdit_;
  QLineEdit * pushEdit_;
  QLineEdit * timeoutEdit_;
  QPushButton * confirmButton_;
  QPushButton * cancelButton_;
  QPushButton * defaultButton_;
private slots:
  void default_();
};

} // namespace mc_rtc_rviz
