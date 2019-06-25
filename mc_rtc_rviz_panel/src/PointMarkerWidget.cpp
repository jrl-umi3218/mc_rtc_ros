/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "PointMarkerWidget.h"

#include "utils.h"

#include <mc_rbdyn/configuration_io.h>

namespace mc_rtc_rviz
{

PointMarkerWidget::PointMarkerWidget(const ClientWidgetParam & params,
                                     visualization_msgs::MarkerArray & markers,
                                     ClientWidget * label)
: ClientWidget(params), markers_(markers), visible_(visible()), was_visible_(visible_)
{
  button_ = label->showHideButton();
  if(button_)
  {
    button_->setCheckable(true);
    button_->setChecked(!visible_);
    connect(button_, SIGNAL(toggled(bool)), this, SLOT(toggled(bool)));
  }
}

void PointMarkerWidget::update(const Eigen::Vector3d & pos, const mc_rtc::gui::PointConfig & c)
{
  if(c.scale > 0 && visible_)
  {
    markers_.markers.push_back(getPointMarker(id2name(id()), pos, c.color, c.scale));
  }
  else if(c.scale > 0 && was_visible_)
  {
    markers_.markers.push_back(getPointMarker(id2name(id()), pos, c.color, c.scale));
    markers_.markers.back().action = visualization_msgs::Marker::DELETE;
  }
  was_visible_ = visible_;
}

void PointMarkerWidget::toggled(bool hide)
{
  visible_ = !hide;
  if(button_)
  {
    button_->setText(hide ? "Show" : "Hide");
  }
  visible(!hide);
}

} // namespace mc_rtc_rviz
