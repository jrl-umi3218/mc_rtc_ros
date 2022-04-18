/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "ArrayLabelWidget.h"

namespace mc_rtc_rviz
{

ArrayLabelWidget::ArrayLabelWidget(const ClientWidgetParam & param, const std::vector<std::string> & labels)
: ArrayInputWidget(param, labels)
{
  labels_layout_->removeWidget(lock_button_);
  delete lock_button_;
  lock_button_ = nullptr;
  if(!labels.size())
  {
    normLabel_ = new QLabel();
    normLabel_->setWordWrap(true);
    labels_layout_->addWidget(normLabel_);
  }
}

void ArrayLabelWidget::update(const Eigen::VectorXd & dataIn)
{
  if(normLabel_)
  {
    std::stringstream ss;
    ss << "<font>";
    for(Eigen::DenseIndex i = 0; i < dataIn.size(); ++i)
    {
      ss << dataIn(i);
      if(i < dataIn.size() - 1)
      {
        ss << ", ";
      }
    }
    ss << "</font>";
    normLabel_->setToolTip(ss.str().c_str());
    normLabel_->setText("norm = " + QString::number(dataIn.norm()));
    if(dataIn.size() <= 6)
    {
      ArrayInputWidget::update(dataIn);
    }
  }
  else
  {
    ArrayInputWidget::update(dataIn);
  }
}

} // namespace mc_rtc_rviz
