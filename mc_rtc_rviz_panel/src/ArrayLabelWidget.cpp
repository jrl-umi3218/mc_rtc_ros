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

void ArrayLabelWidget::update(const Eigen::VectorXd & data)
{
  if(normLabel_)
  {
    std::stringstream ss;
    ss << "<font>";
    for(size_t i = 0; i < data.size(); ++i)
    {
      ss << data(i);
      if(i < data.size() - 1)
      {
        ss << ", ";
      }
    }
    ss << "</font>";
    normLabel_->setToolTip(ss.str().c_str());
    normLabel_->setText(QString::number(data.norm()));
  }
  else
  {
    ArrayInputWidget::update(data);
  }
}

} // namespace mc_rtc_rviz
