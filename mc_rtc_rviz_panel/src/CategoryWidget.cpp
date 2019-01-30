#include "CategoryWidget.h"

static constexpr int MAX_TAB_LEVEL = 2;

namespace mc_rtc_rviz
{

CategoryWidget::CategoryWidget(const ClientWidgetParam & param) : ClientWidget(param)
{
  auto l = new QVBoxLayout(this);
  auto parentCategory = dynamic_cast<CategoryWidget *>(param.parent);
  if(parentCategory)
  {
    level = parentCategory->level + 1;
  }
  if(level < MAX_TAB_LEVEL)
  {
    tabs_ = new QTabWidget(this);
    l->addWidget(tabs_);
  }
  else if(level == MAX_TAB_LEVEL)
  {
    /** Create a scroll-area to welcome the widget */
    auto scroll = new QScrollArea(this);
    auto page = new QWidget(this);
    page_layout_ = new QVBoxLayout(page);
    page_layout_->setSizeConstraint(QLayout::SetMinAndMaxSize);
    page_layout_->addStretch();
    page->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    scroll->setWidgetResizable(true);
    scroll->setWidget(page);
    l->addWidget(scroll);
  }
  else
  {
    toggle_ = new QPushButton(name().c_str(), this);
    toggle_->setCheckable(true);
    l->addWidget(toggle_);
    connect(toggle_, SIGNAL(toggled(bool)), this, SLOT(toggled(bool)));
  }
}

void CategoryWidget::addWidget(ClientWidget * w)
{
  widgets_.push_back(w);
  if(tabs_)
  {
    if(dynamic_cast<CategoryWidget *>(w) == nullptr)
    {
      /** Create a scroll-area to welcome the widget */
      if(!page_layout_)
      {
        auto scroll = new QScrollArea(this);
        auto page = new QWidget(this);
        page_layout_ = new QVBoxLayout(page);
        page_layout_->setSizeConstraint(QLayout::SetMinAndMaxSize);
        page_layout_->addWidget(w);
        page_layout_->addStretch();
        page->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        scroll->setWidgetResizable(true);
        scroll->setWidget(page);
        tab_idx_ = tabs_->addTab(scroll, w->name().c_str());
      }
      else
      {
        tabs_->setTabText(tab_idx_, id().name.c_str());
        page_layout_->insertWidget(page_layout_->count() - 1, w);
      }
    }
    else
    {
      tabs_->addTab(w, w->name().c_str());
    }
  }
  else if(page_layout_)
  {
    page_layout_->insertWidget(page_layout_->count() - 1, w);
  }
  else
  {
    assert(toggle_);
    layout()->addWidget(w);
    if(toggle_->isChecked())
    {
      w->show();
    }
    else
    {
      w->hide();
    }
  }
}

void CategoryWidget::removeWidget(ClientWidget * w)
{
  if(tabs_)
  {
    if(page_layout_)
    {
      page_layout_->removeWidget(w);
    }
    for(int i = 0; i < tabs_->count(); ++i)
    {
      if(tabs_->tabText(i).toStdString() == w->name())
      {
        tabs_->removeTab(i);
        return;
      }
    }
  }
  else if(page_layout_)
  {
    page_layout_->removeWidget(w);
  }
  else
  {
    layout()->removeWidget(w);
  }
}

size_t CategoryWidget::clean()
{
  widgets_.erase(std::remove_if(widgets_.begin(), widgets_.end(),
                                [this](ClientWidget * w) {
                                  if(!w->seen())
                                  {
                                    removeWidget(w);
                                    delete w;
                                    return true;
                                  }
                                  return false;
                                }),
                 widgets_.end());
  return widgets_.size();
}

ClientWidget * CategoryWidget::widget(const std::string & name)
{
  for(auto & w : widgets_)
  {
    if(w->name() == name)
    {
      w->seen(true);
      return w;
    }
  }
  return nullptr;
}

void CategoryWidget::toggled(bool c)
{
  for(const auto & w : widgets_)
  {
    if(c)
    {
      w->show();
    }
    else
    {
      w->hide();
    }
  }
}

} // namespace mc_rtc_rviz
