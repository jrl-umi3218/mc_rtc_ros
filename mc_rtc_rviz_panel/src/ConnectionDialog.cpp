/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "ConnectionDialog.h"

namespace mc_rtc_rviz
{

ConnectionDialog::ConnectionDialog(std::vector<ConnectionConfiguration> & configs, double & timeout, QWidget * parent)
: QDialog(parent), configs_(configs), timeout_(timeout)
{
  setModal(true);
  setWindowTitle("Edit connection parameters");

  layout_ = new QFormLayout(this);
  // Connection editor
  connectionCombo_ = new QComboBox(this);
  for(const auto & c : configs)
  {
    connectionCombo_->addItem(c.toText().c_str());
  }
  connectionCombo_->addItem("New connection...");
  connect(connectionCombo_, SIGNAL(currentIndexChanged(int)), this, SLOT(connectionChanged(int)));
  layout_->addRow("Connection", connectionCombo_);
  // Elements for connection editor
  protocolCombo_ = new QComboBox(this);
  protocolCombo_->addItem("IPC");
  protocolCombo_->addItem("TCP");
  connect(protocolCombo_, SIGNAL(currentIndexChanged(int)), this, SLOT(protocolChanged(int)));
  layout_->addRow("Protocol", protocolCombo_);
  hostEdit_ = new QLineEdit(this);
  layout_->addRow("File/Host", hostEdit_);
  auto makeIntEdit = [this]() {
    auto edit = new QLineEdit(this);
    auto validator = new QIntValidator(this);
    validator->setRange(1, 65535);
    edit->setValidator(validator);
    return edit;
  };
  subPortEdit_ = makeIntEdit();
  subPortLabel_ = new QLabel("Sub port");
  layout_->addRow(subPortLabel_, subPortEdit_);
  pushPortEdit_ = makeIntEdit();
  pushPortLabel_ = new QLabel("Push port");
  layout_->addRow(pushPortLabel_, pushPortEdit_);
  connectionChanged(0);
  protocolChanged(protocolCombo_->currentIndex());
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
  auto hlayout = new QHBoxLayout();
  hlayout->addWidget(confirmButton_);
  hlayout->addWidget(cancelButton_);
  layout_->addRow(hlayout);
}

ConnectionConfiguration::Protocol ConnectionDialog::protocol()
{
  using Protocol = ConnectionConfiguration::Protocol;
  return protocolCombo_->currentText() == "IPC" ? Protocol::IPC : Protocol::TCP;
}

void ConnectionDialog::accept()
{
  auto configFromForm = [this]() {
    auto proto = protocol();
    if(proto == ConnectionConfiguration::Protocol::IPC)
    {
      return ConnectionConfiguration(hostEdit_->text().toStdString());
    }
    else
    {
      return ConnectionConfiguration(hostEdit_->text().toStdString(), subPortEdit_->text().toUInt(),
                                     pushPortEdit_->text().toUInt());
    }
  };
  if(connectionCombo_->currentIndex() < configs_.size())
  {
    configs_.erase(configs_.begin() + connectionCombo_->currentIndex());
  }
  configs_.insert(configs_.begin(), configFromForm());
  timeout_ = timeoutEdit_->text().toDouble();
  QDialog::accept();
}

void ConnectionDialog::connectionChanged(int idx)
{
  using Protocol = ConnectionConfiguration::Protocol;
  const auto & cfg = idx < configs_.size() ? configs_[idx] : ConnectionConfiguration();
  protocolCombo_->setCurrentText(cfg.protocol() == Protocol::IPC ? "IPC" : "TCP");
  hostEdit_->setText(cfg.host().c_str());
  pushPortEdit_->setText(cfg.push_suffix().c_str());
  subPortEdit_->setText(cfg.sub_suffix().c_str());
}

void ConnectionDialog::protocolChanged(int idx)
{
  if(idx == 0) // IPC
  {
    subPortLabel_->hide();
    subPortEdit_->hide();
    pushPortEdit_->hide();
    pushPortLabel_->hide();
  }
  else
  {
    subPortLabel_->show();
    subPortEdit_->show();
    if(!subPortEdit_->hasAcceptableInput())
    {
      subPortEdit_->setText("4242");
    }
    pushPortLabel_->show();
    pushPortEdit_->show();
    if(!pushPortEdit_->hasAcceptableInput())
    {
      pushPortEdit_->setText("4343");
    }
  }
}

} // namespace mc_rtc_rviz
