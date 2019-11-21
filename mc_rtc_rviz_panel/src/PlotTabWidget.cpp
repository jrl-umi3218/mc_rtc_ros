#include "PlotTabWidget.h"

#include "PlotWidget.h"

namespace mc_rtc_rviz
{

PlotTabBar::PlotTabBar(QWidget * parent) : QTabBar(parent) {}

void PlotTabBar::mouseReleaseEvent(QMouseEvent * event)
{
  if(event->button() == Qt::MiddleButton)
  {
    auto pos = event->pos();
    for(int i = 0; i < count(); ++i)
    {
      if(tabRect(i).contains(pos))
      {
        auto tab = static_cast<QTabWidget *>(parent());
        Q_EMIT tab->tabCloseRequested(i);
        return;
      }
    }
  }
  QTabBar::mouseReleaseEvent(event);
}

PlotTab::PlotTab(QWidget * parent) : QTabWidget(parent)
{
  connect(this, SIGNAL(tabCloseRequested(int)), parent, SLOT(closeTabOnRequest(int)));
}

void PlotTab::setTabBar(QTabBar * bar)
{
  QTabWidget::setTabBar(bar);
}

PlotTabWidget::PlotTabWidget(const ClientWidgetParam & param) : ClientWidget(param)
{
  auto layout = new QHBoxLayout(this);
  tab_ = new PlotTab(this);
  tab_->setTabBar(new PlotTabBar(tab_));
  layout->addWidget(tab_);
  tab_->setTabsClosable(true);
  new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_PageUp), this, SLOT(previousTab()));
  new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_PageDown), this, SLOT(nextTab()));
  new QShortcut(QKeySequence(Qt::CTRL + Qt::Key_W), this, SLOT(closeCurrentTab()));
}

void PlotTabWidget::resetSeen()
{
  seen_.clear();
}

bool PlotTabWidget::wasSeen()
{
  for(auto it = std::begin(plots_); it != std::end(plots_);)
  {
    if(!seen_.count(it->first))
    {
      auto idx = tab_->indexOf(it->second);
      tab_->setTabText(idx, tab_->tabText(idx) + " (DONE)");
      inactive_plots_.push_back(it->second);
      it = plots_.erase(it);
    }
    else
    {
      ++it;
    }
  }
  return (plots_.size() + inactive_plots_.size()) != 0;
}

void PlotTabWidget::start_plot(uint64_t id, const std::string & title)
{
  seen_[id] = true;
  if(!plots_.count(id))
  {
    size_t i = 0;
    for(const auto & p : plots_)
    {
      if(p.second->title() == title)
      {
        i += 1;
      }
    }
    std::string tabTitle = title;
    if(i != 0)
    {
      tabTitle += " (" + std::to_string(i) + ")";
    }
    plots_[id] = new PlotWidget(title, this);
    tab_->addTab(plots_[id], tabTitle.c_str());
    tab_->setCurrentIndex(tab_->count() - 1);
  }
}

void PlotTabWidget::plot_setup_xaxis(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  plots_[id]->setup_xaxis(legend, range);
}

void PlotTabWidget::plot_setup_yaxis_left(uint64_t id,
                                          const std::string & legend,
                                          const mc_rtc::gui::plot::Range & range)
{
  plots_[id]->setup_yaxis_left(legend, range);
}

void PlotTabWidget::plot_setup_yaxis_right(uint64_t id,
                                           const std::string & legend,
                                           const mc_rtc::gui::plot::Range & range)
{
  plots_[id]->setup_yaxis_right(legend, range);
}

void PlotTabWidget::plot_point(uint64_t id,
                               uint64_t did,
                               const std::string & legend,
                               double x,
                               double y,
                               mc_rtc::gui::Color color,
                               mc_rtc::gui::plot::Style style,
                               mc_rtc::gui::plot::Side side)
{
  plots_[id]->plot(did, legend, x, y, color, style, side);
}

void PlotTabWidget::plot_polygon(uint64_t id,
                                 uint64_t did,
                                 const std::string & legend,
                                 const mc_rtc::gui::plot::PolygonDescription & polygon,
                                 mc_rtc::gui::plot::Side side)
{
  plots_[id]->plot(did, legend, polygon, side);
}

void PlotTabWidget::plot_polygons(uint64_t id,
                                  uint64_t did,
                                  const std::string & legend,
                                  const std::vector<mc_rtc::gui::plot::PolygonDescription> & polygons,
                                  mc_rtc::gui::plot::Side side)
{
  plots_[id]->plot(did, legend, polygons, side);
}

void PlotTabWidget::end_plot(uint64_t id)
{
  plots_[id]->refresh();
}

void PlotTabWidget::closeTabOnRequest(int i)
{
  auto plot = static_cast<PlotWidget *>(tab_->widget(i));
  for(auto it = inactive_plots_.begin(); it != inactive_plots_.end(); ++it)
  {
    if(*it == plot)
    {
      inactive_plots_.erase(it);
      delete plot;
      return;
    }
  }
}

void PlotTabWidget::previousTab()
{
  if(tab_->currentIndex() != 0)
  {
    tab_->setCurrentIndex(tab_->currentIndex() - 1);
  }
  else
  {
    tab_->setCurrentIndex(tab_->count() - 1);
  }
}

void PlotTabWidget::nextTab()
{
  if(tab_->currentIndex() != tab_->count() - 1)
  {
    tab_->setCurrentIndex(tab_->currentIndex() + 1);
  }
  else
  {
    tab_->setCurrentIndex(0);
  }
}

void PlotTabWidget::closeCurrentTab()
{
  Q_EMIT tab_->tabCloseRequested(tab_->currentIndex());
}

} // namespace mc_rtc_rviz
