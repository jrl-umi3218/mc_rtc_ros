#pragma once

#include "ClientWidget.h"

namespace mc_rtc_rviz
{

struct CategoryWidget : public ClientWidget
{
  Q_OBJECT
public:
  CategoryWidget(const ClientWidgetParam & param);

  void addWidget(ClientWidget * w) override;

  void removeWidget(ClientWidget * w) override;

  size_t clean() override;

  ClientWidget * widget(const std::string & name) override;
private:
  int level = 0;
  QTabWidget * tabs_ = nullptr;
  QVBoxLayout * page_layout_ = nullptr;
  QPushButton * toggle_ = nullptr;
  std::vector<ClientWidget *> widgets_;
private slots:
  void toggled(bool);
};

}
