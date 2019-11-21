#include "PlotTabWidget.h"

#include "PlotWidget.h"

namespace mc_rtc_rviz
{

PlotDialog::PlotDialog(PlotWidget * plot, PlotTabWidget * parent, QString title)
: QDialog(parent), plot_(plot), parent_(parent)
{
  setWindowTitle(title);
  auto layout = new QVBoxLayout(this);
  layout->addWidget(plot);
  plot->show();
  show();
  connect(this, SIGNAL(rejected()), this, SLOT(handle_close()));
}

void PlotDialog::closeEvent(QCloseEvent * event)
{
  handle_close();
  event->accept();
}

void PlotDialog::handle_close()
{
  parent_->close_dialog(this);
}

PopTabButton::PopTabButton(int index, PlotTabWidget * tab, QWidget * parent)
: QPushButton(QIcon(":/icons/view-fullscreen.svg"), "", parent), index_(index), tab_(tab)
{
  setFlat(true);
  connect(this, SIGNAL(released()), this, SLOT(clicked()));
}

void PopTabButton::update(int idx)
{
  index_ = idx;
}

void PopTabButton::clicked()
{
  tab_->popTab(index_);
}

CloseTabButton::CloseTabButton(int index, PlotTabWidget * tab, QWidget * parent)
: QPushButton(QIcon(":/icons/close.svg"), "", parent), index_(index), tab_(tab)
{
  setFlat(true);
  connect(this, SIGNAL(released()), this, SLOT(clicked()));
}

void CloseTabButton::update(int idx)
{
  index_ = idx;
}

void CloseTabButton::clicked()
{
  Q_EMIT tab_->tab()->tabCloseRequested(index_);
}

PlotTabBar::PlotTabBar(QWidget * parent, PlotTabWidget * tab) : QTabBar(parent), tab_(tab) {}

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

void PlotTabBar::tabInserted(int idx)
{
  QTabBar::tabInserted(idx);
  setTabButton(idx, QTabBar::LeftSide, new PopTabButton(idx, tab_, this));
}

void PlotTabBar::tabLayoutChange()
{
  for(int i = 0; i < count(); ++i)
  {
    auto pop_button = static_cast<PopTabButton *>(tabButton(i, QTabBar::LeftSide));
    if(pop_button)
    {
      pop_button->update(i);
    }
    auto close_button = static_cast<CloseTabButton *>(tabButton(i, QTabBar::RightSide));
    if(close_button)
    {
      close_button->update(i);
    }
  }
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
  tab_->setTabBar(new PlotTabBar(tab_, this));
  layout->addWidget(tab_);
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
      if(idx != -1)
      {
        tab_->setTabText(idx, tab_->tabText(idx) + " (DONE)");
        tab_->tabBar()->setTabButton(idx, QTabBar::RightSide, new CloseTabButton(idx, this, tab_->tabBar()));
      }
      else
      {
        for(size_t i = 0; i < dialogs_.size(); ++i)
        {
          auto & diag = dialogs_[i];
          if(diag->plot() == it->second)
          {
            diag->setWindowTitle(diag->windowTitle() + " (DONE)");
          }
        }
      }
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
  remove_plot_widget(plot);
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

void PlotTabWidget::popTab(int idx)
{
  auto plot = static_cast<PlotWidget *>(tab_->widget(idx));
  dialogs_.push_back(new PlotDialog(plot, this, tab_->tabText(idx)));
}

void PlotTabWidget::close_dialog(PlotDialog * dialog)
{
  auto plot = dialog->plot();
  add_plot_widget(dialog->windowTitle(), plot);
  for(auto it = dialogs_.begin(); it != dialogs_.end(); ++it)
  {
    if(*it == dialog)
    {
      dialogs_.erase(it);
      return;
    }
  }
}

bool PlotTabWidget::remove_plot_widget(PlotWidget * widget)
{
  for(auto it = inactive_plots_.begin(); it != inactive_plots_.end(); ++it)
  {
    if(*it == widget)
    {
      inactive_plots_.erase(it);
      delete widget;
      return true;
    }
  }
  return false;
}

void PlotTabWidget::add_plot_widget(QString title, PlotWidget * widget)
{
  tab_->addTab(widget, title);
  if(title.endsWith("(DONE)"))
  {
    auto idx = tab_->count() - 1;
    tab_->tabBar()->setTabButton(idx, QTabBar::RightSide, new CloseTabButton(idx, this, tab_->tabBar()));
  }
  widget->show();
}

} // namespace mc_rtc_rviz
