#include "ArrayLabelWidget.h"

namespace mc_rtc_rviz
{

ArrayLabelWidget::ArrayLabelWidget(const ClientWidgetParam & param,
                                   const std::vector<std::string> & labels)
: LabelWidget(param),
  labels_(labels)
{
}

void ArrayLabelWidget::update(const Eigen::VectorXd & v)
{
  if(labels_.size() == 0)
  {
    label_->setText(QString::number(v.norm()));
    std::stringstream tooltip;
    for(size_t i = 0; i < v.size(); ++i)
    {
      tooltip << v(i);
      if(i < v.size() - 1)
      {
        tooltip << ", ";
      }
    }
    label_->setToolTip(tooltip.str().c_str());
  }
  else
  {
    std::stringstream text;
    for(size_t i = 0; i < v.size(); ++i)
    {
      if(i < labels_.size())
      {
        text << labels_[i] << ": ";
      }
      text << v(i);
      if(i < v.size() - 1)
      {
        text << ", ";
      }
    }
    label_->setText(text.str().c_str());
    label_->setToolTip(QString::number(v.norm()));
  }
}

}
