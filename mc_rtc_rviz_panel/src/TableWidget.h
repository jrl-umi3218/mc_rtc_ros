/*
 * Copyright 2016-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include "ClientWidget.h"

namespace mc_rtc_rviz
{

struct TableWidget : public ClientWidget
{
  Q_OBJECT
public:
  TableWidget(const ClientWidgetParam & params);

  void header(const std::vector<std::string> & header);

  void row(const std::vector<std::string> & data);

  void finalize();

private:
  std::vector<std::string> header_ = {};
  int rowCount_ = 0;
  QHBoxLayout * layout_ = nullptr;
  QTableWidget * table_ = nullptr;

  void updateItem(int row, int column, const std::string & text);
};

} // namespace mc_rtc_rviz
