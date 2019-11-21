#include "PlotTabWidget.h"

#include "PlotWidget.h"

namespace mc_rtc_rviz
{

PlotTabWidget::PlotTabWidget(const ClientWidgetParam & param) : ClientWidget(param)
{
  auto layout = new QHBoxLayout(this);
  tab_ = new QTabWidget(this);
  layout->addWidget(tab_);
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
  for(auto it = std::begin(inactive_plots_); it != std::end(inactive_plots_);)
  {
    auto plot = *it;
    ++it;
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

} // namespace mc_rtc_rviz
