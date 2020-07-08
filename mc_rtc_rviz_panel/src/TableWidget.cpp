/*
 * Copyright 2016-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "TableWidget.h"

namespace mc_rtc_rviz
{

TableWidget::TableWidget(const ClientWidgetParam & params) : ClientWidget(params)
{
  layout_ = new QHBoxLayout(this);
  table_ = new QTableWidget();
  table_->verticalHeader()->hide();
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
  table_->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
#else
  table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
#endif
  layout_->addWidget(table_);
}

void TableWidget::header(const std::vector<std::string> & header)
{
  rowCount_ = 0;
  if(header_ == header)
  {
    return;
  }
  header_ = header;
  table_->setColumnCount(header_.size());
  for(size_t j = 0; j < header_.size(); ++j)
  {
    auto item = table_->horizontalHeaderItem(j);
    if(!item)
    {
      item = new QTableWidgetItem();
      table_->setHorizontalHeaderItem(j, item);
    }
    item->setText(header_[j].c_str());
  }
}

void TableWidget::row(const std::vector<std::string> & data)
{
  for(size_t j = 0; j < data.size(); ++j)
  {
    updateItem(rowCount_, j, data[j]);
  }
  rowCount_ += 1;
}

void TableWidget::finalize()
{
  while(table_->rowCount() > rowCount_)
  {
    table_->removeRow(table_->rowCount() - 1);
  }
  auto height = table_->horizontalHeader()->height() + 2;
  for(int i = 0; i < table_->rowCount(); ++i)
  {
    height += table_->rowHeight(i);
  }
  auto scrollbar = table_->horizontalScrollBar();
  if(scrollbar && scrollbar->isVisible())
  {
    height += table_->horizontalScrollBar()->height();
  }
  table_->setMinimumHeight(height);
  table_->setMaximumHeight(height);
}

void TableWidget::updateItem(int row, int column, const std::string & text)
{
  if(table_->rowCount() < row + 1)
  {
    table_->setRowCount(row + 1);
  }
  if(table_->columnCount() < column + 1)
  {
    table_->setColumnCount(column + 1);
  }
  auto item = dynamic_cast<QLabel *>(table_->cellWidget(row, column));
  if(!item)
  {
    item = new QLabel();
    table_->setCellWidget(row, column, item);
  }
  item->setText(text.c_str());
}

} // namespace mc_rtc_rviz
