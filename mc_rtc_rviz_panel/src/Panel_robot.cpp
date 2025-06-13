/*
 * Copyright 2016-2025 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "Panel.h"
#include "RobotModelDisplay.h"

namespace mc_rtc_rviz
{

void Panel::robot(const WidgetId & id,
                  const std::vector<std::string> & parameters,
                  const std::vector<std::vector<double>> & q,
                  const sva::PTransformd & posW)
{
  Q_EMIT signal_robot(id, parameters, q, posW);
}

void Panel::got_robot(const WidgetId & id,
                      const std::vector<std::string> & parameters,
                      const std::vector<std::vector<double>> & /*q*/,
                      const sva::PTransformd & /*posW*/)
{
  auto & w = get_widget<RobotModelDisplay>(id, displayContext(), displayGroup());
  w.update(parameters[0]);
}

} // namespace mc_rtc_rviz
