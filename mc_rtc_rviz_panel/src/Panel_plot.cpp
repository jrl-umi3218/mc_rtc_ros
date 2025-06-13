/*
 * Copyright 2016-2025 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "Panel.h"
#include "PlotTabWidget.h"

namespace mc_rtc_rviz
{

void Panel::start_plot(uint64_t id, const std::string & title)
{
  Q_EMIT signal_start_plot(id, title);
}

void Panel::plot_setup_xaxis(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  Q_EMIT signal_plot_setup_xaxis(id, legend, range);
}

void Panel::plot_setup_yaxis_left(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  Q_EMIT signal_plot_setup_yaxis_left(id, legend, range);
}

void Panel::plot_setup_yaxis_right(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  Q_EMIT signal_plot_setup_yaxis_right(id, legend, range);
}

void Panel::plot_point(uint64_t id,
                       uint64_t did,
                       const std::string & legend,
                       double x,
                       double y,
                       mc_rtc::gui::Color color,
                       mc_rtc::gui::plot::Style style,
                       mc_rtc::gui::plot::Side side)
{
  Q_EMIT signal_plot_point(id, did, legend, x, y, color, style, side);
}

void Panel::plot_polygon(uint64_t id,
                         uint64_t did,
                         const std::string & legend,
                         const mc_rtc::gui::plot::PolygonDescription & polygon,
                         mc_rtc::gui::plot::Side side)
{
  Q_EMIT signal_plot_polygon(id, did, legend, polygon, side);
}

void Panel::plot_polygons(uint64_t id,
                          uint64_t did,
                          const std::string & legend,
                          const std::vector<mc_rtc::gui::plot::PolygonDescription> & polygons,
                          mc_rtc::gui::plot::Side side)
{
  Q_EMIT signal_plot_polygons(id, did, legend, polygons, side);
}

void Panel::end_plot(uint64_t id)
{
  Q_EMIT signal_end_plot(id);
}

void Panel::got_start_plot(uint64_t id, const std::string & title)
{
  got_category({}, "Plots");
  auto & w = get_widget<PlotTabWidget>({{"Plots"}, ""});
  w.start_plot(id, title);
}

void Panel::got_plot_setup_xaxis(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  auto & w = get_widget<PlotTabWidget>({{"Plots"}, ""});
  w.plot_setup_xaxis(id, legend, range);
}

void Panel::got_plot_setup_yaxis_left(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  auto & w = get_widget<PlotTabWidget>({{"Plots"}, ""});
  w.plot_setup_yaxis_left(id, legend, range);
}

void Panel::got_plot_setup_yaxis_right(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  auto & w = get_widget<PlotTabWidget>({{"Plots"}, ""});
  w.plot_setup_yaxis_right(id, legend, range);
}

void Panel::got_plot_point(uint64_t id,
                           uint64_t did,
                           const std::string & legend,
                           double x,
                           double y,
                           mc_rtc::gui::Color color,
                           mc_rtc::gui::plot::Style style,
                           mc_rtc::gui::plot::Side side)
{
  auto & w = get_widget<PlotTabWidget>({{"Plots"}, ""});
  w.plot_point(id, did, legend, x, y, color, style, side);
}

void Panel::got_plot_polygon(uint64_t id,
                             uint64_t did,
                             const std::string & legend,
                             const mc_rtc::gui::plot::PolygonDescription & polygon,
                             mc_rtc::gui::plot::Side side)
{
  auto & w = get_widget<PlotTabWidget>({{"Plots"}, ""});
  w.plot_polygon(id, did, legend, polygon, side);
}

void Panel::got_plot_polygons(uint64_t id,
                              uint64_t did,
                              const std::string & legend,
                              const std::vector<mc_rtc::gui::plot::PolygonDescription> & polygons,
                              mc_rtc::gui::plot::Side side)
{
  auto & w = get_widget<PlotTabWidget>({{"Plots"}, ""});
  w.plot_polygons(id, did, legend, polygons, side);
}

void Panel::got_end_plot(uint64_t id)
{
  auto & w = get_widget<PlotTabWidget>({{"Plots"}, ""});
  w.end_plot(id);
}

} // namespace mc_rtc_rviz
