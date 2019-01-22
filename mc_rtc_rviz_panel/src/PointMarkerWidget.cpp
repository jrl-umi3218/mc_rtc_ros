#include "PointMarkerWidget.h"

#include <mc_rbdyn/configuration_io.h>
#include "utils.h"

namespace mc_rtc_rviz
{

PointMarkerWidget::PointMarkerWidget(const ClientWidgetParam & params,
                                     visualization_msgs::MarkerArray & markers)
: ClientWidget(params),
  markers_(markers)
{
}

void PointMarkerWidget::update(const Eigen::Vector3d & pos, const mc_rtc::gui::PointConfig & c)
{
  if(c.scale > 0)
  {
    markers_.markers.push_back(getPointMarker(id2name(id()), pos, c.color, c.scale));
  }
}

}
