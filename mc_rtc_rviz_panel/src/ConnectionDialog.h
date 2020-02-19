/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ConnectionConfiguration.h"

#include <QtWidgets>

namespace mc_rtc_rviz
{

class ConnectionDialog : public QDialog
{
  Q_OBJECT
public:
  ConnectionDialog(std::vector<ConnectionConfiguration> & configs, double & timeout, QWidget * parent = nullptr);

  void accept() override;

private:
  std::vector<ConnectionConfiguration> & configs_;
  double & timeout_;
  QFormLayout * layout_;
  QComboBox * connectionCombo_;
  QComboBox * protocolCombo_;
  QLineEdit * hostEdit_;
  QLabel * subPortLabel_;
  QLineEdit * subPortEdit_;
  QLabel * pushPortLabel_;
  QLineEdit * pushPortEdit_;
  QLineEdit * timeoutEdit_;
  QPushButton * confirmButton_;
  QPushButton * cancelButton_;

  ConnectionConfiguration::Protocol protocol();
private slots:
  void connectionChanged(int idx);
  void protocolChanged(int);
};

} // namespace mc_rtc_rviz
