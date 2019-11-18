/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include "ClientWidget.h"

#include <qwt/qwt_legend.h>
#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_curve.h>
#include <qwt/qwt_plot_grid.h>
#include <qwt/qwt_plot_shapeitem.h>

#include <unordered_map>

namespace mc_rtc_rviz
{

struct PlotWidget : public QDialog
{
  Q_OBJECT
public:
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

  void plot(uint64_t id,
            const std::string & legend,
            const mc_rtc::gui::plot::PolygonDescription & polygon,
            mc_rtc::gui::plot::Side side);

  void plot(uint64_t id,
            const std::string & legend,
            const std::vector<mc_rtc::gui::plot::PolygonDescription> & polygons,
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
  struct Curve
  {
    Curve() {}
    Curve(QwtPlot * plot, const std::string & legend, mc_rtc::gui::plot::Side side);
    Curve(const Curve &) = delete;
    Curve & operator=(const Curve &) = delete;
    Curve(Curve && rhs);
    Curve & operator=(Curve && rhs);
    ~Curve();
    QRectF update(double x, double y, mc_rtc::gui::Color color, mc_rtc::gui::plot::Style style);

  private:
    QwtPlotCurve * curve_ = nullptr;
    QVector<QPointF> samples_;
    QRectF rect_;
  };
  struct Polygon
  {
    Polygon() {}
    Polygon(QwtPlot * plot, const std::string & legend, mc_rtc::gui::plot::Side side);
    Polygon(const Polygon &) = delete;
    Polygon & operator=(const Polygon &) = delete;
    Polygon(Polygon && rhs);
    Polygon & operator=(Polygon && rhs);
    ~Polygon();
    QRectF update(const mc_rtc::gui::plot::PolygonDescription & poly);

    QwtPlotShapeItem * item()
    {
      return item_;
    }

    mc_rtc::gui::plot::PolygonDescription poly()
    {
      return poly_;
    }

  private:
    mc_rtc::gui::plot::PolygonDescription poly_;
    QwtPlotShapeItem * item_ = nullptr;
    QPolygonF polygon_;
  };
  struct PolygonsVector
  {
    PolygonsVector() {}
    PolygonsVector(QwtPlot * plot, const std::string & legend, mc_rtc::gui::plot::Side side);
    QRectF update(const std::vector<mc_rtc::gui::plot::PolygonDescription> & polys);

  private:
    QwtPlot * plot_;
    std::string legend_;
    mc_rtc::gui::plot::Side side_;
    std::vector<mc_rtc::gui::plot::PolygonDescription> polys_;
    std::vector<Polygon> polygons_;
  };
  void update(mc_rtc::gui::plot::Side side, QRectF rect);
  std::unordered_map<uint64_t, Curve> curves_;
  std::unordered_map<uint64_t, Polygon> polygons_;
  std::unordered_map<uint64_t, PolygonsVector> polygonsVectors_;
  std::unordered_map<mc_rtc::gui::plot::Side, QRectF> boundingRects_;
  bool seen_ = true;
  bool done_ = false;
  bool has_left_plot_ = false;
  bool has_right_plot_ = false;
  mc_rtc::gui::plot::Range xRange_;
  mc_rtc::gui::plot::Range yLeftRange_;
  mc_rtc::gui::plot::Range yRightRange_;
  bool limit_xrange_ = false;
private slots:
  void limit_xrange_cbox_changed(int);
  void save_button_released();
};

} // namespace mc_rtc_rviz
