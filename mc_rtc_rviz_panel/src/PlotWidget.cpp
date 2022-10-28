#include "PlotWidget.h"

#include <qwt_global.h>
#if QWT_VERSION < 0x060100
#  error "This requires Qwt >= 6.1.0 to build"
#endif

#include <qwt_plot_renderer.h>
#include <qwt_plot_shapeitem.h>
#include <qwt_symbol.h>
#include <qwt_text.h>

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
bool setPen(T * curve, Color color, Style style, double width = 0.)
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
  curve->setPen(qc, width, pstyle);
  return true;
}

} // namespace

PlotWidget::Curve::Curve(QwtPlot * plot, const std::string & legend, mc_rtc::gui::plot::Side side)
{
  curve_ = new QwtPlotCurve(QwtText(legend.c_str()));
  curve_->setLegendAttribute(QwtPlotCurve::LegendShowLine);
  curve_->setLegendIconSize({40, 8});
  curve_->attach(plot);
  curve_->setRenderHint(QwtPlotItem::RenderAntialiased);
  curve_->setRawSamples(samples_x_.data(), samples_y_.data(), static_cast<int>(samples_x_.size()));
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
  samples_x_ = rhs.samples_x_;
  samples_y_ = rhs.samples_y_;
  rect_ = rhs.rect_;
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
  samples_x_ = rhs.samples_x_;
  samples_y_ = rhs.samples_y_;
  rect_ = rhs.rect_;
  rhs.curve_ = nullptr;
  return *this;
}

QRectF PlotWidget::Curve::update(double x,
                                 double y,
                                 mc_rtc::gui::Color color,
                                 mc_rtc::gui::plot::Style style,
                                 double line_width,
                                 double show_duration)
{
  if(samples_x_.size() == 0)
  {
    rect_.setLeft(x);
    rect_.setRight(x);
    rect_.setBottom(y);
    rect_.setTop(y);
  }
  if(setPen(curve_, color, style, line_width))
  {
    curve_->setStyle(QwtPlotCurve::Lines);
  }
  else
  {
    samples_x_.resize(0);
    samples_y_.resize(0);
    auto symbol = new QwtSymbol(QwtSymbol::XCross);
    auto qc = convert(color);
    symbol->setColor(qc);
    symbol->setPen(qc);
    symbol->setSize(10);
    curve_->setSymbol(symbol);
    curve_->setStyle(QwtPlotCurve::NoCurve);
  }
  samples_x_.push_back(x);
  samples_y_.push_back(y);
  size_t data_offset = 0;
  size_t data_size = samples_x_.size();
  if(show_duration > 0 && samples_x_.size() > 1)
  {
    double dt = samples_x_[1] - samples_x_[0];
    size_t n_points = std::ceil(show_duration / dt);
    if(samples_x_.size() > n_points)
    {
      data_offset = samples_x_.size() - n_points - 1;
      data_size = n_points;
    }
  }
  curve_->setRawSamples(&samples_x_[data_offset], &samples_y_[data_offset], static_cast<int>(data_size));
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
  auto show_layout = new QHBoxLayout();
  auto limit_xrange_cbox = new QCheckBox("Only show the last ", this);
  connect(limit_xrange_cbox, SIGNAL(stateChanged(int)), this, SLOT(limit_xrange_cbox_changed(int)));
  show_layout->addWidget(limit_xrange_cbox);
  auto duration_input = new QDoubleSpinBox(this);
  duration_input->setMinimum(0);
  duration_input->setSuffix("s");
  duration_input->setValue(show_duration_);
  connect(duration_input, SIGNAL(valueChanged(double)), this, SLOT(show_duration_changed(double)));
  show_layout->addWidget(duration_input);
  layout->addLayout(show_layout);

  // Controls when paused (zoom, scroll, etc)
  auto setIcon = [](QPushButton * button, const QString & iconName, const QString & fallbackText) {
    if(QIcon::hasThemeIcon(iconName))
    {
      button->setIcon(QIcon::fromTheme(iconName));
      button->setToolTip(fallbackText);
    }
    else
    {
      button->setText(fallbackText);
    }
  };
  controls_widget_ = new QWidget();
  controls_widget_->setVisible(false);
  layout->addWidget(controls_widget_);
  auto clayout = new QHBoxLayout(controls_widget_);
  zoom_button_ = new QPushButton();
  setIcon(zoom_button_, "zoom-in", "Zoom");
  zoom_button_->setCheckable(true);
  connect(zoom_button_, SIGNAL(clicked()), this, SLOT(zoom_button_clicked()));
  auto reset_zoom_button = new QPushButton();
  setIcon(reset_zoom_button, "go-home", "Reset");
  connect(reset_zoom_button, SIGNAL(clicked()), this, SLOT(zoom_reset_button_clicked()));
  auto prev_zoom_button = new QPushButton();
  setIcon(prev_zoom_button, "edit-undo", "Undo zoom");
  connect(prev_zoom_button, SIGNAL(clicked()), this, SLOT(zoom_prev_button_clicked()));
  auto next_zoom_button = new QPushButton();
  setIcon(next_zoom_button, "edit-redo", "Redo zoom");
  connect(next_zoom_button, SIGNAL(clicked()), this, SLOT(zoom_next_button_clicked()));
  pan_button_ = new QPushButton();
  setIcon(pan_button_, "transform-move", "Move");
  pan_button_->setCheckable(true);
  connect(pan_button_, SIGNAL(clicked()), this, SLOT(pan_button_clicked()));
  clayout->addWidget(reset_zoom_button);
  clayout->addWidget(zoom_button_);
  clayout->addWidget(prev_zoom_button);
  clayout->addWidget(next_zoom_button);
  clayout->addWidget(pan_button_);

  auto hlayout = new QHBoxLayout();
  pause_button_ = new QPushButton();
  pause_button_->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
  pause_button_->setCheckable(true);
  pause_button_->setChecked(true);
  hlayout->addWidget(pause_button_);
  connect(pause_button_, SIGNAL(clicked()), this, SLOT(pause_button_clicked()));
  auto save_button = new QPushButton(this);
  setIcon(save_button, "document-save-as", "Save as...");
  hlayout->addWidget(save_button);
  connect(save_button, SIGNAL(clicked()), this, SLOT(save_button_clicked()));
  options_button_ = new QPushButton("More");
  options_button_->setCheckable(true);
  connect(options_button_, SIGNAL(clicked()), this, SLOT(toggle_options_widget()));
  hlayout->addWidget(options_button_);
  layout->addLayout(hlayout);

  // Additional options group (line width, etc), hidden by default
  options_widget_ = new QGroupBox("Style Options", this);
  options_widget_->setVisible(false);
  auto options_layout = new QVBoxLayout();
  auto line_width_layout = new QHBoxLayout();
  auto line_width_label = new QLabel("Line width");
  line_width_layout->addWidget(line_width_label);
  auto line_width_input = new QDoubleSpinBox(this);
  line_width_input->setValue(line_width_);
  line_width_input->setMinimum(0);
  line_width_layout->addWidget(line_width_input);
  connect(line_width_input, SIGNAL(valueChanged(double)), this, SLOT(line_width_changed(double)));
  options_layout->addLayout(line_width_layout);
  options_widget_->setLayout(options_layout);
  layout->addWidget(options_widget_);
  show();

  zoom_ = new QwtPlotZoomer(QwtPlot::xBottom, QwtPlot::yLeft, plot_->canvas());
  zoom_->setEnabled(false);
  pan_ = new QwtPlotPanner(plot_->canvas());
  pan_->setEnabled(false);
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
  update(side, curves_[id].update(x, y, color, style, line_width_, limit_xrange_ ? show_duration_ : -1));
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
  if(paused_)
  {
    plot_->replot();
    return;
  }
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
    if(limit && r > show_duration_)
    {
      min = max - show_duration_;
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

void PlotWidget::show_duration_changed(double value)
{
  show_duration_ = value;
}

void PlotWidget::pause_button_clicked()
{
  paused_ = !paused_;
  if(paused_)
  {
    pause_button_->setChecked(false);
    pause_button_->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
    controls_widget_->setVisible(true);
    zoom_->setZoomBase(true);
  }
  else
  {
    pause_button_->setChecked(true);
    pause_button_->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
    controls_widget_->setVisible(false);
  }
}

void PlotWidget::zoom_button_clicked()
{
  if(zoom_button_->isChecked())
  {
    pan_->setEnabled(false);
    pan_button_->setChecked(false);
    zoom_->setEnabled(true);
  }
  else
  {
    zoom_->setEnabled(false);
  }
}

void PlotWidget::zoom_reset_button_clicked()
{
  zoom_->zoom(0);
}

void PlotWidget::zoom_prev_button_clicked()
{
  zoom_->zoom(-1);
}

void PlotWidget::zoom_next_button_clicked()
{
  zoom_->zoom(1);
}

void PlotWidget::pan_button_clicked()
{
  if(pan_button_->isChecked())
  {
    zoom_->setEnabled(false);
    zoom_button_->setChecked(false);
    pan_->setEnabled(true);
  }
  else
  {
    pan_->setEnabled(false);
  }
}

void PlotWidget::save_button_clicked()
{
  QwtPlotRenderer renderer(this);
  renderer.exportTo(plot_, (title_ + ".svg").c_str());
}

void PlotWidget::toggle_options_widget()
{
  options_widget_->setVisible(!options_widget_->isVisible());
  if(options_widget_->isVisible())
  {
    options_button_->setText("Less");
  }
  else
  {
    options_button_->setText("More");
  }
}

void PlotWidget::line_width_changed(double width)
{
  line_width_ = width;
}

} // namespace mc_rtc_rviz
