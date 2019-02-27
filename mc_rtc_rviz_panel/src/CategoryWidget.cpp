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
    layout_ = l;
    toggle_ = new QPushButton(name().c_str(), this);
    toggle_->setCheckable(true);
    l->addWidget(toggle_);
    connect(toggle_, SIGNAL(toggled(bool)), this, SLOT(toggled(bool)));
  }
}

void CategoryWidget::addWidget(ClientWidget * w)
{
  QHBoxLayout * stackLayout = nullptr;
  widgets_.push_back(w);
  if(w->sid() != -1)
  {
    if(stack_layouts_.count(w->sid()))
    {
      stack_layouts_[w->sid()]->addWidget(w);
      return;
    }
    stackLayout = new QHBoxLayout();
    stackLayout->addWidget(w);
    stack_layouts_[w->sid()] = stackLayout;
  }
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
        if(stackLayout)
        {
          page_layout_->addLayout(stackLayout);
        }
        else
        {
          page_layout_->addWidget(w);
        }
        page_layout_->addStretch();
        page->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        scroll->setWidgetResizable(true);
        scroll->setWidget(page);
        tab_idx_ = tabs_->addTab(scroll, w->name().c_str());
      }
      else
      {
        tabs_->setTabText(tab_idx_, id().name.c_str());
        if(stackLayout)
        {
          page_layout_->insertLayout(page_layout_->count() - 1, stackLayout);
        }
        else
        {
          page_layout_->insertWidget(page_layout_->count() - 1, w);
        }
      }
    }
    else
    {
      tabs_->addTab(w, w->name().c_str());
    }
  }
  else if(page_layout_)
  {
    if(stackLayout)
    {
      page_layout_->insertLayout(page_layout_->count() - 1, stackLayout);
    }
    else
    {
      page_layout_->insertWidget(page_layout_->count() - 1, w);
    }
  }
  else
  {
    assert(toggle_);
    assert(layout_);
    if(stackLayout)
    {
      layout_->addLayout(stackLayout);
    }
    else
    {
      layout_->addWidget(w);
    }
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
  w->seen(false);
  if(w->sid() && stack_layouts_.count(w->sid()))
  {
    int sid = w->sid();
    stack_layouts_[sid]->removeWidget(w);
    if(stack_layouts_[sid]->count() == 0)
    {
      delete stack_layouts_[sid];
      stack_layouts_.erase(sid);
    }
  }
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
        break;
      }
    }
    if(tabs_->count() == 0)
    {
      delete page_layout_;
      page_layout_ = nullptr;
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
  widgets_.erase(std::find(widgets_.begin(), widgets_.end(), w));
  delete w;
}

size_t CategoryWidget::clean()
{
  for(auto it = widgets_.begin(); it != widgets_.end();)
  {
    ClientWidget * w = *it;
    if(!w->seen())
    {
      removeWidget(w);
    }
    else
    {
      ++it;
    }
  }
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
