#pragma once

#include "Panel.h"

namespace mc_rtc_rviz
{

class ConnectionDialog : public QDialog
{
  Q_OBJECT
public:
  ConnectionDialog(std::string & sub_uri, std::string & push_uri, QWidget * parent = nullptr);

  void accept() override;
private:
  std::string & sub_uri_;
  std::string & push_uri_;
  QFormLayout * layout_;
  QLineEdit * subEdit_;
  QLineEdit * pushEdit_;
  QPushButton * confirmButton_;
  QPushButton * cancelButton_;
  QPushButton * defaultButton_;
private slots:
  void default_();
};

} // namespace mc_rtc_rviz
