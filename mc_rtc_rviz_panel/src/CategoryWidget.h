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
  QVBoxLayout * main_layout_ = nullptr;
  std::map<int, QHBoxLayout *> stack_layouts_;
  QTabWidget * tabs_ = nullptr;
  std::vector<ClientWidget *> widgets_;
  CategoryWidget * parent_ = nullptr;
  void updateSizeImpl(bool active);
private slots:
  void updateSize(int index);
};

} // namespace mc_rtc_rviz
