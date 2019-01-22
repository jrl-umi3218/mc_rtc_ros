#pragma once

#include "ClientWidget.h"

namespace mc_rtc_rviz
{

struct ArrayInputWidget : public ClientWidget
{
  Q_OBJECT
public:
  ArrayInputWidget(const ClientWidgetParam & param, const std::vector<std::string> & labels);

  void update(const Eigen::VectorXd & data);
private:
  QPushButton * lock_button_;
  QGridLayout * edits_layout_;
  int edits_row_ = 0;
  std::vector<QLineEdit*> edits_;
private slots:
  void lock_toggled(bool);
  void edit_return_pressed();
};

}
