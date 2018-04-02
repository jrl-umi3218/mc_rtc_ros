#pragma once

#include "LabelWidget.h"

namespace mc_rtc_rviz
{
  struct ArrayLabelWidget : public LabelWidget
  {
    ArrayLabelWidget(const ClientWidgetParam & param,
                     const std::vector<std::string> & labels);

    void update(const Eigen::VectorXd & in);
  protected:
    std::vector<std::string> labels_;
  };
}
