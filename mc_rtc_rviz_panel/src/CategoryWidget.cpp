#include "CategoryWidget.h"

static constexpr int MAX_TAB_LEVEL = 2;

namespace
{
  static const std::vector<std::string> special_suffixes = {"_point3d", "_rotation", "_transform"};
}

namespace mc_rtc_rviz
{

CategoryWidget::CategoryWidget(const ClientWidgetParam & param)
: ClientWidget(param)
{
  auto l = new QVBoxLayout(this);
  auto parentCategory = dynamic_cast<CategoryWidget*>(param.parent);
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
    connect(toggle_, &QPushButton::toggled,
            this, [this](bool c)
            {
              for(const auto & w : widgets_)
              {
                if(c) { w->show(); } else { w->hide(); }
              }
            });

  }
}

void CategoryWidget::addWidget(ClientWidget * w)
{
  widgets_.push_back(w);
  if(tabs_)
  {
    if(dynamic_cast<CategoryWidget*>(w) == nullptr)
    {
      const auto & name = w->name();
      for(const auto & s : special_suffixes)
      {
        if(name.size() > s.size() && name.substr(name.size() - s.size(), s.size()) == s)
        {
          auto parent_name = name.substr(0, name.size() - s.size());
          for(int i = 0; i < tabs_->count(); ++i)
          {
            if(tabs_->tabText(i).toStdString() == parent_name)
            {
              auto scroll = static_cast<QScrollArea*>(tabs_->widget(i));
              auto layout = static_cast<QVBoxLayout*>(scroll->widget()->layout());;
              layout->insertWidget(layout->count() - 1, w);
              return;
            }
          }
        }
      }
      /** Create a scroll-area to welcome the widget */
      auto scroll = new QScrollArea(this);
      auto page = new QWidget(this);
      auto layout = new QVBoxLayout(page);
      layout->setSizeConstraint(QLayout::SetMinAndMaxSize);
      layout->addWidget(w);
      layout->addStretch();
      page->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
      scroll->setWidgetResizable(true);
      scroll->setWidget(page);
      tabs_->addTab(scroll, w->name().c_str());
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
    if(toggle_->isChecked()) { w->show(); }
    else { w->hide(); }
  }
}

void CategoryWidget::removeWidget(ClientWidget * w)
{
  if(tabs_)
  {
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
                 [this](ClientWidget * w)
                 {
                  if(!w->seen()) { removeWidget(w); return true; }
                  return false;
                 }), widgets_.end());
  return widgets_.size();
}

ClientWidget & CategoryWidget::widget(const std::string & name,
                                      std::function<ClientWidget*()> make_fn)
{
  for(auto & w : widgets_)
  {
    if(w->name() == name) { w->seen(true); return *w; }
  }
  addWidget(make_fn());
  return *widgets_.back();
}

}
