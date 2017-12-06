#include "LabelWidget.h"

LabelWidget::LabelWidget(const std::string & name,
                          const mc_rtc::Configuration & data)
: nameLabel(new QLabel(name.c_str())),
  dataLabel(new QLabel())
{
  layout->addWidget(nameLabel);
  layout->addWidget(dataLabel);
  is_vector = data("GUI")("vector", false);
}

void LabelWidget::update(const mc_rtc::Configuration & data)
{
  std::string dataStr;
  if(is_vector)
  {
    Eigen::VectorXd dataV = data;
    std::stringstream ss;
    ss << dataV.norm();
    dataStr = ss.str();
  }
  else
  {
    dataStr = data.dump().substr(0, 25);
  }
  dataLabel->setText(dataStr.c_str());
}
