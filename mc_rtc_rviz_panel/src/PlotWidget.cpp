#include "PlotWidget.h"

#include <qwt_global.h>
#if QWT_VERSION < 0x060100
#  error "This requires Qwt >= 6.1.0 to build"
#endif

#include <qwt_plot_renderer.h>
#include <qwt_plot_shapeitem.h>
#include <qwt_symbol.h>

namespace mc_rtc_rviz
{

namespace
{

using Color = mc_rtc::gui::Color;
using Side = mc_rtc::gui::plot::Side;
using Style = mc_rtc::gui::plot::Style;
using PolygonDescription = mc_rtc::gui::plot::PolygonDescription;

QColor convert(const Color & color)
{
  return QColor::fromRgbF(color.r, color.g, color.b, color.a);
}

template<typename T>
bool setPen(T * curve, Color color, Style style)
{
  if(style == Style::Point)
  {
    setPen(curve, color, Style::Dotted);
    return false;
  }
  auto qc = convert(color);
  auto pstyle = Qt::SolidLine;
  if(style == Style::Dashed)
  {
    pstyle = Qt::DashLine;
  }
  if(style == Style::Dotted)
  {
    pstyle = Qt::DotLine;
  }
  curve->setPen(qc, 0.0, pstyle);
  return true;
}

} // namespace

PlotWidget::Curve::Curve(QwtPlot * plot, const std::string & legend, mc_rtc::gui::plot::Side side)
{
  curve_ = new QwtPlotCurve(QwtText(legend.c_str()));
  curve_->attach(plot);
  curve_->setRenderHint(QwtPlotItem::RenderAntialiased);
  curve_->setSamples(samples_);
  if(side == Side::Left)
  {
    curve_->setYAxis(QwtPlot::yLeft);
  }
  else
  {
    curve_->setYAxis(QwtPlot::yRight);
  }
}

PlotWidget::Curve::Curve(Curve && rhs)
{
  if(curve_)
  {
    curve_->detach();
    delete curve_;
  }
  curve_ = rhs.curve_;
  samples_ = rhs.samples_;
  rect_ = rhs.rect_;
  rhs.curve_ = nullptr;
}

PlotWidget::Curve & PlotWidget::Curve::operator=(Curve && rhs)
{
  if(&rhs == this)
  {
    return *this;
  }
  if(curve_)
  {
    curve_->detach();
    delete curve_;
  }
  curve_ = rhs.curve_;
  samples_ = rhs.samples_;
  rect_ = rhs.rect_;
  rhs.curve_ = nullptr;
  return *this;
}

QRectF PlotWidget::Curve::update(double x, double y, mc_rtc::gui::Color color, mc_rtc::gui::plot::Style style)
{
  if(samples_.size() == 0)
  {
    rect_.setLeft(x);
    rect_.setRight(x);
    rect_.setBottom(y);
    rect_.setTop(y);
  }
  if(setPen(curve_, color, style))
  {
    curve_->setStyle(QwtPlotCurve::Lines);
  }
  else
  {
    samples_.resize(0);
    auto symbol = new QwtSymbol(QwtSymbol::XCross);
    auto qc = convert(color);
    symbol->setColor(qc);
    symbol->setPen(qc);
    symbol->setSize(10);
    curve_->setSymbol(symbol);
    curve_->setStyle(QwtPlotCurve::NoCurve);
  }
  samples_.push_back({x, y});
  curve_->setSamples(samples_);
  rect_.setLeft(std::min(x, rect_.left()));
  rect_.setRight(std::max(x, rect_.right()));
  rect_.setBottom(std::max(y, rect_.bottom()));
  rect_.setTop(std::min(y, rect_.top()));
  return rect_;
}

PlotWidget::Curve::~Curve()
{
  if(curve_)
  {
    curve_->detach();
    delete curve_;
  }
}

PlotWidget::Polygon::Polygon(QwtPlot * plot, const std::string & legend, mc_rtc::gui::plot::Side side)
{
  item_ = new QwtPlotShapeItem(QwtText(legend.c_str()));
  item_->attach(plot);
  item_->setRenderHint(QwtPlotItem::RenderAntialiased);
  item_->setPolygon(polygon_);
  if(side == Side::Left)
  {
    item_->setYAxis(QwtPlot::yLeft);
  }
  else
  {
    item_->setYAxis(QwtPlot::yRight);
  }
}

PlotWidget::Polygon::Polygon(Polygon && rhs)
{
  if(item_)
  {
    item_->detach();
    delete item_;
  }
  poly_ = rhs.poly_;
  item_ = rhs.item_;
  polygon_ = rhs.polygon_;
  rect_ = rhs.rect_;
  rhs.item_ = nullptr;
  if(item_)
  {
    item_->setPolygon(polygon_);
  }
}

PlotWidget::Polygon & PlotWidget::Polygon::operator=(Polygon && rhs)
{
  if(&rhs == this)
  {
    return *this;
  }
  if(item_)
  {
    item_->detach();
    delete item_;
  }
  poly_ = rhs.poly_;
  item_ = rhs.item_;
  polygon_ = rhs.polygon_;
  rect_ = rhs.rect_;
  rhs.item_ = nullptr;
  if(item_)
  {
    item_->setPolygon(polygon_);
  }
  return *this;
}

QRectF PlotWidget::Polygon::update(const PolygonDescription & poly)
{
  if(poly_ == poly)
  {
    return rect_;
  }
  poly_ = poly;
  polygon_.clear();
  if(poly_.points().size())
  {
    const auto & p = poly_.points()[0];
    rect_.setLeft(p[0]);
    rect_.setRight(p[0]);
    rect_.setTop(p[1]);
    rect_.setBottom(p[1]);
  }
  else
  {
    rect_ = QRectF();
  }
  for(const auto & p : poly_.points())
  {
    polygon_ << QPointF{p[0], p[1]};
    rect_.setLeft(std::min(p[0], rect_.left()));
    rect_.setRight(std::max(p[0], rect_.right()));
    rect_.setBottom(std::max(p[1], rect_.bottom()));
    rect_.setTop(std::min(p[1], rect_.top()));
  }
  if(poly_.closed())
  {
    const auto & p = poly_.points().front();
    polygon_ << QPointF{p[0], p[1]};
  }
  setPen(item_, poly_.outline(), poly_.style());
  const auto & fill = poly_.fill();
  if(poly_.closed())
  {
    item_->setBrush(convert(fill));
  }
  item_->setPolygon(polygon_);
  return rect_;
}

PlotWidget::Polygon::~Polygon()
{
  if(item_)
  {
    item_->detach();
    delete item_;
  }
}

PlotWidget::PolygonsVector::PolygonsVector(QwtPlot * plot, const std::string & legend, mc_rtc::gui::plot::Side side)
: plot_(plot), legend_(legend), side_(side)
{
}

QRectF PlotWidget::PolygonsVector::update(const std::vector<PolygonDescription> & poly)
{
  polygons_.resize(poly.size());
  for(size_t i = 0; i < polygons_.size(); ++i)
  {
    if(polygons_[i].poly() != poly[i])
    {
      polygons_[i] = Polygon(plot_, legend_, side_);
      polygons_[i].update(poly[i]);
    }
  }
  QRectF rect;
  if(polygons_.size())
  {
    rect = polygons_[0].rect();
  }
  for(size_t i = 1; i < polygons_.size(); ++i)
  {
    rect = rect.united(polygons_[i].rect());
  }
  return rect;
}

PlotWidget::PlotWidget(const std::string & title, QWidget * parent) : QWidget(parent), title_(title)
{
  auto layout = new QVBoxLayout(this);
  plot_ = new QwtPlot(QwtText(title.c_str()), this);
  plot_->setCanvasBackground(Qt::white);
  legend_ = new QwtLegend();
  plot_->insertLegend(legend_, QwtPlot::TopLegend);
  grid_ = new QwtPlotGrid();
  grid_->setPen(QColor::fromRgbF(0.0, 0.0, 0.0, 0.5), 0.0, Qt::DashLine);
  grid_->attach(plot_);
  layout->addWidget(plot_);
  auto limit_xrange_cbox = new QCheckBox("Only show the last 10 seconds", this);
  layout->addWidget(limit_xrange_cbox);
  connect(limit_xrange_cbox, SIGNAL(stateChanged(int)), this, SLOT(limit_xrange_cbox_changed(int)));
  auto save_button = new QPushButton("Save as...", this);
  layout->addWidget(save_button);
  connect(save_button, SIGNAL(released()), this, SLOT(save_button_released()));
  show();
}

const std::string & PlotWidget::title() const
{
  return title_;
}

void PlotWidget::setup_xaxis(const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  if(legend.size())
  {
    plot_->setAxisTitle(QwtPlot::xBottom, QwtText(legend.c_str()));
  }
  xRange_ = range;
}

void PlotWidget::setup_yaxis_left(const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  if(legend.size())
  {
    plot_->setAxisTitle(QwtPlot::yLeft, QwtText(legend.c_str()));
  }
  yLeftRange_ = range;
}

void PlotWidget::setup_yaxis_right(const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  if(legend.size())
  {
    plot_->setAxisTitle(QwtPlot::yRight, QwtText(legend.c_str()));
  }
  yRightRange_ = range;
}

void PlotWidget::plot(uint64_t id,
                      const std::string & legend,
                      double x,
                      double y,
                      mc_rtc::gui::Color color,
                      mc_rtc::gui::plot::Style style,
                      mc_rtc::gui::plot::Side side)
{
  if(!curves_.count(id))
  {
    curves_[id] = Curve(plot_, legend, side);
  }
  update(side, curves_[id].update(x, y, color, style));
}

void PlotWidget::plot(uint64_t id,
                      const std::string & legend,
                      const PolygonDescription & polygon,
                      mc_rtc::gui::plot::Side side)
{
  if(!polygons_.count(id))
  {
    polygons_[id] = Polygon(plot_, legend, side);
  }
  update(side, polygons_[id].update(polygon));
}

void PlotWidget::plot(uint64_t id,
                      const std::string & legend,
                      const std::vector<PolygonDescription> & polygons,
                      mc_rtc::gui::plot::Side side)
{
  if(!polygonsVectors_.count(id))
  {
    polygonsVectors_[id] = PolygonsVector(plot_, legend, side);
  }
  update(side, polygonsVectors_[id].update(polygons));
}

void PlotWidget::update(Side side, QRectF rect)
{
  if(side == Side::Left)
  {
    if(!has_left_plot_)
    {
      has_left_plot_ = true;
      boundingRects_[side] = rect;
    }
    boundingRects_[side] = boundingRects_[side].united(rect);
  }
  else
  {
    if(!has_right_plot_)
    {
      has_right_plot_ = true;
      boundingRects_[side] = rect;
    }
    boundingRects_[side] = boundingRects_[side].united(rect);
  }
}

void PlotWidget::refresh()
{
  if(!has_left_plot_)
  {
    plot_->enableAxis(QwtPlot::yLeft, false);
  }
  if(has_right_plot_)
  {
    plot_->enableAxis(QwtPlot::yRight);
  }
  auto inf = mc_rtc::gui::plot::Range::inf;
  auto setScale = [this, inf](int id, const mc_rtc::gui::plot::Range & range, double min, double max, bool limit) {
    min = range.min != -inf ? range.min : min;
    max = range.max != inf ? range.max : max;
    if(min == inf)
    {
      min = 0;
      max = 1;
    }
    if(min == max)
    {
      max = min + 0.1;
    }
    double r = max - min;
    if(limit && r > 10)
    {
      min = max - 10;
    }
    plot_->setAxisScale(id, min, max);
  };
  auto & yLRect = boundingRects_[Side::Left];
  auto & yRRect = boundingRects_[Side::Right];
  auto xRect = yLRect.united(yRRect);
  if(has_left_plot_ && !has_right_plot_)
  {
    xRect = yLRect;
  }
  if(has_right_plot_ && !has_left_plot_)
  {
    xRect = yRRect;
  }
  setScale(QwtPlot::xBottom, xRange_, xRect.left(), xRect.right(), limit_xrange_);
  setScale(QwtPlot::yLeft, yLeftRange_, yLRect.top(), yLRect.bottom(), false);
  setScale(QwtPlot::yRight, yRightRange_, yRRect.top(), yRRect.bottom(), false);
  plot_->replot();
}

void PlotWidget::limit_xrange_cbox_changed(int state)
{
  limit_xrange_ = (state == Qt::Checked);
}

void PlotWidget::save_button_released()
{
  QwtPlotRenderer renderer(this);
  renderer.exportTo(plot_, (title_ + ".svg").c_str());
}

} // namespace mc_rtc_rviz
