/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "ConnectionDialog.h"

namespace mc_rtc_rviz
{

ConnectionDialog::ConnectionDialog(std::string & sub_uri, std::string & push_uri, double & timeout, QWidget * parent)
: QDialog(parent), sub_uri_(sub_uri), push_uri_(push_uri), timeout_(timeout)
{
  setModal(true);
  setWindowTitle("Edit connection parameters");

  layout_ = new QFormLayout(this);
  // Connection editor
  subEdit_ = new QLineEdit(sub_uri.c_str(), this);
  layout_->addRow("Subscription URI", subEdit_);
  pushEdit_ = new QLineEdit(push_uri.c_str(), this);
  layout_->addRow("Push URI", pushEdit_);
  // Timeout configuration
  timeoutEdit_ = new QLineEdit(QString::number(timeout_), this);
  timeoutEdit_->setToolTip("Connection timeout, <= 0 to disable");
  auto validator = new QDoubleValidator(this);
  validator->setLocale(QLocale::C);
  timeoutEdit_->setValidator(validator);
  auto timeoutLabel = new QLabel("Timeout");
  timeoutLabel->setToolTip(timeoutEdit_->toolTip());
  layout_->addRow(timeoutLabel, timeoutEdit_);
  // Confirmation buttons
  confirmButton_ = new QPushButton("OK", this);
  connect(confirmButton_, SIGNAL(released()), this, SLOT(accept()));
  cancelButton_ = new QPushButton("Cancel", this);
  connect(cancelButton_, SIGNAL(released()), this, SLOT(reject()));
  defaultButton_ = new QPushButton("Default", this);
  connect(defaultButton_, SIGNAL(released()), this, SLOT(default_()));
  auto hlayout = new QHBoxLayout();
  hlayout->addWidget(confirmButton_);
  hlayout->addWidget(cancelButton_);
  hlayout->addWidget(defaultButton_);
  layout_->addRow(hlayout);
}

void ConnectionDialog::accept()
{
  sub_uri_ = subEdit_->text().toStdString();
  push_uri_ = pushEdit_->text().toStdString();
  timeout_ = timeoutEdit_->text().toDouble();
  QDialog::accept();
}

void ConnectionDialog::default_()
{
  subEdit_->setText("ipc:///tmp/mc_rtc_pub.ipc");
  pushEdit_->setText("ipc:///tmp/mc_rtc_rep.ipc");
  timeoutEdit_->setText("2");
}

} // namespace mc_rtc_rviz
