/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include "ClientWidget.h"

#include <qwt/qwt_legend.h>
#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_curve.h>
#include <qwt/qwt_plot_grid.h>

namespace mc_rtc_rviz
{

struct PlotWidget : public QDialog
{
  PlotWidget(const std::string & wtitle, const std::string & title, QWidget * parent);

  const std::string & title() const;

  void setup_xaxis(const std::string & legend, const mc_rtc::gui::plot::Range & range);

  void setup_yaxis_left(const std::string & legend, const mc_rtc::gui::plot::Range & range);

  void setup_yaxis_right(const std::string & legend, const mc_rtc::gui::plot::Range & range);

  void plot(uint64_t id,
            const std::string & legend,
            double x,
            double y,
            mc_rtc::gui::Color color,
            mc_rtc::gui::plot::Style style,
            mc_rtc::gui::plot::Side side);

  void refresh();

  bool seen()
  {
    if(seen_)
    {
      seen_ = false;
      return true;
    }
    return false;
  }

  void mark_done();

  void closeEvent(QCloseEvent * event) override;

private:
  std::string title_;
  QwtPlot * plot_;
  QwtPlotGrid * grid_;
  QwtLegend * legend_;
  std::vector<QwtPlotCurve *> curves_;
  std::vector<QVector<QPointF>> samples_;
  bool seen_ = true;
  bool done_ = false;
  bool has_left_plot_ = false;
  mc_rtc::gui::plot::Range xRangeCurrent_;
  mc_rtc::gui::plot::Range xRange_;
  mc_rtc::gui::plot::Range yLeftRangeCurrent_;
  mc_rtc::gui::plot::Range yLeftRange_;
  mc_rtc::gui::plot::Range yRightRangeCurrent_;
  mc_rtc::gui::plot::Range yRightRange_;
};

} // namespace mc_rtc_rviz
