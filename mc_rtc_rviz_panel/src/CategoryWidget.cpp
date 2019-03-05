#include "CategoryWidget.h"

static constexpr int MAX_TAB_LEVEL = 2;

namespace mc_rtc_rviz
{

CategoryWidget::CategoryWidget(const ClientWidgetParam & param) : ClientWidget(param)
{
  parent_ = dynamic_cast<CategoryWidget *>(param.parent);
  bool isRoot = parent_ == nullptr;
  auto l = new QVBoxLayout(this);
  // left, top, right, bottom
  l->setContentsMargins(5, 5, 0, 0);
  main_layout_ = new QVBoxLayout();
  main_layout_->setAlignment(Qt::AlignTop);
  tabs_ = new QTabWidget(this);
  if(isRoot)
  {
    auto scroll = new QScrollArea(this);
    scroll->setFrameShape(QFrame::NoFrame);
    auto page = new QWidget(this);
    auto page_layout_ = new QVBoxLayout(page);
    page_layout_->addLayout(main_layout_);
    page_layout_->addWidget(tabs_);
    scroll->setWidgetResizable(true);
    scroll->setWidget(page);
    l->addWidget(scroll);
  }
  else
  {
    l->addLayout(main_layout_);
    l->addWidget(tabs_);
  }
  connect(tabs_, SIGNAL(currentChanged(int)), this, SLOT(updateSize(int)));
  tabs_->hide();
}

void CategoryWidget::addWidget(ClientWidget * w)
{
  widgets_.push_back(w);
  if(dynamic_cast<CategoryWidget *>(w))
  {
    /** w is a CategoryWidget */
    tabs_->addTab(w, w->name().c_str());
    tabs_->show();
    return;
  }
  if(w->sid() != -1)
  {
    /** w is part of a stack */
    if(!stack_layouts_.count(w->sid()))
    {
      stack_layouts_[w->sid()] = new QHBoxLayout();
      main_layout_->addLayout(stack_layouts_[w->sid()]);
    }
    stack_layouts_[w->sid()]->addWidget(w);
    return;
  }
  main_layout_->addWidget(w);
}

void CategoryWidget::removeWidget(ClientWidget * w)
{
  w->seen(false);
  if(dynamic_cast<CategoryWidget *>(w))
  {
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
      tabs_->hide();
    }
  }
  else if(w->sid() && stack_layouts_.count(w->sid()))
  {
    int sid = w->sid();
    stack_layouts_[sid]->removeWidget(w);
    if(stack_layouts_[sid]->count() == 0)
    {
      delete stack_layouts_[sid];
      stack_layouts_.erase(sid);
    }
  }
  else
  {
    main_layout_->removeWidget(w);
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

void CategoryWidget::updateSizeImpl(bool active)
{
  for(int i = 0; i < tabs_->count(); ++i)
  {
    static_cast<CategoryWidget *>(tabs_->widget(i))->updateSizeImpl(active && tabs_->currentIndex() == i);
  }
  if(active)
  {
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
  }
  else
  {
    setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  }
}

void CategoryWidget::updateSize(int)
{
  auto parent = parent_ ? parent_ : this;
  while(parent->parent_ != nullptr)
  {
    parent = parent->parent_;
  }
  parent->updateSizeImpl(true);
}

} // namespace mc_rtc_rviz
