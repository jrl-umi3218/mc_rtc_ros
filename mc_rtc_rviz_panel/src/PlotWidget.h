/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL, BIT
 */

#pragma once

#include "ClientWidget.h"

#include <qwt_legend.h>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>

#include <map>
#include <unordered_map>

struct QwtPlotShapeItem;

namespace mc_rtc_rviz
{

struct PlotWidget : public QWidget
{
  Q_OBJECT
public:
  PlotWidget(const std::string & title, QWidget * parent);

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

    const QRectF & rect() const
    {
      return rect_;
    }

  private:
    mc_rtc::gui::plot::PolygonDescription poly_;
    QwtPlotShapeItem * item_ = nullptr;
    QPolygonF polygon_;
    QRectF rect_;
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
  std::map<mc_rtc::gui::plot::Side, QRectF> boundingRects_;
  bool has_left_plot_ = false;
  bool has_right_plot_ = false;
  mc_rtc::gui::plot::Range xRange_;
  mc_rtc::gui::plot::Range yLeftRange_;
  mc_rtc::gui::plot::Range yRightRange_;
  bool limit_xrange_ = false;
  double show_duration_ = 10;
  QPushButton * pause_button_;
  bool paused_ = false;
private slots:
  void limit_xrange_cbox_changed(int);
  void show_duration_changed(double);
  void save_button_clicked();
  void pause_button_clicked();
};

} // namespace mc_rtc_rviz
