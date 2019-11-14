#include "PlotWidget.h"

namespace mc_rtc_rviz
{

PlotWidget::PlotWidget(const std::string & wtitle, const std::string & title, QWidget * parent)
: QDialog(parent), title_(title)
{
  auto inf = mc_rtc::gui::plot::Range::inf;
  xRangeCurrent_ = {inf, -inf};
  yLeftRangeCurrent_ = {inf, -inf};
  yRightRangeCurrent_ = {inf, -inf};
  setWindowTitle(wtitle.c_str());
  auto layout = new QVBoxLayout(this);
  plot_ = new QwtPlot(QwtText(title.c_str()), this);
  plot_->setCanvasBackground(Qt::white);
  legend_ = new QwtLegend();
  plot_->insertLegend(legend_, QwtPlot::TopLegend);
  grid_ = new QwtPlotGrid();
  grid_->setPen(QColor::fromRgbF(0.0, 0.0, 0.0, 0.5), 0.0, Qt::DashLine);
  grid_->attach(plot_);
  layout->addWidget(plot_);
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
  xRangeCurrent_.min = std::min(xRangeCurrent_.min, x);
  xRangeCurrent_.max = std::max(xRangeCurrent_.max, x);
  if(curves_.size() <= id)
  {
    size_t olds = curves_.size();
    curves_.resize(id + 1);
    samples_.resize(id + 1);
    for(size_t i = olds; i < id; ++i)
    {
      curves_[i] = nullptr;
    }
  }
  if(curves_[id] == nullptr)
  {
    curves_[id] = new QwtPlotCurve(QwtText(legend.c_str()));
  }
  auto & curve = curves_[id];
  auto & samples = samples_[id];
  samples.push_back({x, y});
  curve->setRenderHint(QwtPlotItem::RenderAntialiased);
  auto qc = QColor::fromRgbF(color.r, color.g, color.b, color.a);
  if(style != mc_rtc::gui::plot::Style::Scatter)
  {
    curve->setStyle(QwtPlotCurve::Lines);
    auto qstyle = Qt::SolidLine;
    if(style == mc_rtc::gui::plot::Style::Dashed)
    {
      qstyle = Qt::DashLine;
    }
    if(style == mc_rtc::gui::plot::Style::Dotted)
    {
      qstyle = Qt::DotLine;
    }
    curve->setPen(qc, 0.0, qstyle);
  }
  else
  {
    curve->setStyle(QwtPlotCurve::Dots);
  }
  curve->setSamples(samples);
  if(side == mc_rtc::gui::plot::Side::Left)
  {
    yLeftRangeCurrent_.min = std::min(yLeftRangeCurrent_.min, y);
    yLeftRangeCurrent_.max = std::max(yLeftRangeCurrent_.max, y);
    has_left_plot_ = true;
    curve->setYAxis(QwtPlot::yLeft);
  }
  else
  {
    yRightRangeCurrent_.min = std::min(yRightRangeCurrent_.min, y);
    yRightRangeCurrent_.max = std::max(yRightRangeCurrent_.max, y);
    plot_->enableAxis(QwtPlot::yRight);
    curve->setYAxis(QwtPlot::yRight);
  }
  curve->attach(plot_);
}

void PlotWidget::refresh()
{
  seen_ = true;
  if(!has_left_plot_)
  {
    plot_->enableAxis(QwtPlot::yLeft, false);
  }
  auto inf = mc_rtc::gui::plot::Range::inf;
  auto setScale = [this, inf](int id, const mc_rtc::gui::plot::Range & range,
                              const mc_rtc::gui::plot::Range & currentRange) {
    double min = range.min != -inf ? range.min : currentRange.min;
    double max = range.max != inf ? range.max : currentRange.max;
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
    max += r * 0.05;
    min -= r * 0.05;
    plot_->setAxisScale(id, min, max);
  };
  setScale(QwtPlot::xBottom, xRange_, xRangeCurrent_);
  setScale(QwtPlot::yLeft, yLeftRange_, yLeftRangeCurrent_);
  setScale(QwtPlot::yRight, yRightRange_, yRightRangeCurrent_);
  plot_->replot();
}

void PlotWidget::mark_done()
{
  if(!done_)
  {
    done_ = true;
    setWindowTitle(windowTitle() + " (DONE)");
  }
}

void PlotWidget::closeEvent(QCloseEvent * event)
{
  if(done_)
  {
    event->accept();
  }
  else
  {
    event->ignore();
  }
}

} // namespace mc_rtc_rviz
