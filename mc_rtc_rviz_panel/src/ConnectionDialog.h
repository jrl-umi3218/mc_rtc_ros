#pragma once

#include "Panel.h"

namespace mc_rtc_rviz
{

struct ConnectionDialog : public QDialog
{
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
};

} // namespace mc_rtc_rviz
