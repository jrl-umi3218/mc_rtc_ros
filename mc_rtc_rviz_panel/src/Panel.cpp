#include "Panel.h"

#include "ButtonWidget.h"
#include "FormWidget.h"
#include "InputWidget.h"
#include "LabelWidget.h"
#include "Point3DWidget.h"
#include "SchemaWidget.h"
#include "TransformWidget.h"

#include <mc_rtc/Configuration.h>

#include <stack>

namespace
{
  constexpr int MAX_TAB_LEVEL = 2;
  static_assert(MAX_TAB_LEVEL >= 0, "Must have at least zero tab level");

  struct FoldableWidget : public QWidget
  {
    FoldableWidget(const std::string & name, bool always_show)
    {
      auto layout = new QVBoxLayout();
      setLayout(layout);
      topLayout = new QVBoxLayout();
      bottomLayout = new QVBoxLayout();
      layout->addLayout(topLayout);
      layout->addLayout(bottomLayout);
      if(!always_show)
      {
        showButton = new QPushButton(name.c_str());
        showButton->setCheckable(true);
        topLayout->addWidget(showButton);
        connect(showButton, &QPushButton::toggled,
            this, [this](bool)
            {
            for(const auto & w : widgets_)
            {
              if(showButton->isChecked()) { w->show(); }
              else { w->hide(); }
            }
            });
      }
    }
    void addWidget(QWidget * w, bool is_foldable)
    {
      widgets_.push_back(w);
      if(showButton && !showButton->isChecked())
      {
        w->hide();
      }
      else
      {
        w->show();
      }
      if(is_foldable) { bottomLayout->addWidget(w); }
      else { topLayout->addWidget(w); }
    }
    void removeWidget(QWidget * w, bool is_foldable)
    {
      auto it = std::find(widgets_.begin(), widgets_.end(), w);
      if(is_foldable) { bottomLayout->removeWidget(w); }
      else { topLayout->removeWidget(w); }
      if(it != widgets_.end())
      {
        widgets_.erase(it);
      }
    }
    QVBoxLayout * topLayout = nullptr;
    QVBoxLayout * bottomLayout = nullptr;
    QPushButton * showButton = nullptr;
    std::vector<QWidget*> widgets_ = {};
  };
}

namespace mc_rtc_rviz
{

Panel::Panel(QWidget * parent)
: QWidget(parent),
  //FIXME Hard-coded parameters for the client
  mc_control::ControllerClient("ipc:///tmp/mc_rtc_pub.ipc", "ipc:///tmp/mc_rtc_rep.ipc", 10.),
  nh_(),
  int_server_(std::make_shared<interactive_markers::InteractiveMarkerServer>("mc_rtc_rviz_interactive_markers"))
{
  mainLayout = new QVBoxLayout();
  setLayout(mainLayout);
  connect(this, SIGNAL(gotData(const char*)),
          this, SLOT(handle_gui_state_impl(const char*)));
}

namespace
{
  BaseWidget * makeEntry(QWidget * parent,
                         const std::vector<std::string> & category,
                         const std::string & name,
                         const mc_rtc::Configuration & data,
                         mc_control::ControllerClient & client,
                         std::shared_ptr<interactive_markers::InteractiveMarkerServer> int_server)
  {
    auto gui_type = data("GUI")("type");
    if(gui_type == "Label")
    {
      return new LabelWidget(name, data);
    }
    if(gui_type == "Button")
    {
      return new ButtonWidget(name,
        [&client,category,name](const mc_rtc::Configuration & data)
        {
          client.send_request(category, name, data);
        });
    }
    if(gui_type == "Point3D")
    {
      return new Point3DWidget(name, data,
          [&client,category,name](const mc_rtc::Configuration & data)
          {
            client.send_request(category, name, data);
          },
          int_server);
    }
    if(gui_type == "Transform")
    {
      return new TransformWidget(name, data,
          [&client,category,name](const mc_rtc::Configuration & data)
          {
            client.send_request(category, name, data);
          },
          int_server);
    }
    if(gui_type == "Input")
    {
      return new InputWidget(name, data,
          [&client,category,name](const mc_rtc::Configuration & data)
          {
            client.send_request(category, name, data);
          });
    }
    if(gui_type == "Schema")
    {
      return new SchemaWidget(parent, data,
          [&client,category,name](const mc_rtc::Configuration & data)
          {
            client.send_request(category, name, data);
          });
    }
    if(gui_type == "Form")
    {
      return new FormWidget(parent, data,
          [&client,category,name](const mc_rtc::Configuration & data)
          {
            client.send_request(category, name, data);
          });
    }
    std::cerr << "Cannot handle the provided GUI type " << gui_type << std::endl;
    return nullptr;
  }
}

void Panel::handle_gui_state(const char * data_raw, size_t size)
{
  Q_EMIT gotData(data_raw);
}

void Panel::handle_category(const std::string & category,
                            const std::string & name,
                            const std::map<std::string, mc_rtc::Configuration> & items,
                            const mc_rtc::Configuration & data,
                            QWidget * parent,
                            int level,
                            std::vector<std::string> & seen)
{
  if(items.size() == 0) { return; }
  if(items.count("GUI"))
  {
    /** Actual item */
    if(!widgets_.count(category))
    {
      auto q_categories = QString(category.c_str()).split("/");
      std::vector<std::string> categories {};
      for(int i = 1; i < q_categories.count() - 1; ++i)
      {
        const auto & c = q_categories[i];
        categories.emplace_back(c.toStdString());
      }
      auto w = makeEntry(parent, categories, name, data, *this, int_server_);
      widgets_[category] = w;
      if(w)
      {
        if(level <= MAX_TAB_LEVEL)
        {
          parent->layout()->addWidget(w);
          remove_w_[category] = [parent](QWidget * w)
          {
            parent->layout()->removeWidget(w);
          };
        }
        else
        {
          static_cast<FoldableWidget*>(parent)->addWidget(w, false);
          remove_w_[category] = [parent](QWidget * w)
          {
            static_cast<FoldableWidget*>(parent)->removeWidget(w, false);
          };
        }
      }
    }
    seen.push_back(category);
    auto w = static_cast<BaseWidget*>(widgets_[category]);
    if(w && items.count("data"))
    {
      w->update(items.at("data"));
    }
  }
  else
  {
    /** Category */
    if(!widgets_.count(category))
    {
      QWidget * w = nullptr;
      if(level < MAX_TAB_LEVEL)
      {
        w = new QTabWidget();
        w->setLayout(new QVBoxLayout());
      }
      else
      {
        w = new FoldableWidget(name, level == MAX_TAB_LEVEL);
      }
      if(level == 0)
      {
        assert(parent == this);
        this->mainLayout->addWidget(w);
        remove_w_[category] = [this](QWidget * w)
        {
          this->mainLayout->removeWidget(w);
        };
      }
      else if(level <= MAX_TAB_LEVEL)
      {
        auto p = static_cast<QTabWidget*>(parent);
        p->addTab(w, name.c_str());
        remove_w_[category] = [p](QWidget * w)
        {
          p->removeTab(p->indexOf(w));
        };
      }
      else
      {
        auto p = static_cast<FoldableWidget*>(parent);
        p->addWidget(w, true);
        remove_w_[category] = [p](QWidget * w)
        {
          p->removeWidget(w, true);
        };
      }
      widgets_[category] = w;
    }
    seen.push_back(category);
    auto w = widgets_[category];
    for(const auto & i : items)
    {
      handle_category(category + "/" + i.first,
                      i.first,
                      i.second,
                      i.second,
                      w,
                      level + 1, seen);
    }
  }
}

void Panel::handle_gui_state_impl(const char * data_raw)
{
  auto data = mc_rtc::Configuration::fromData(data_raw);
  std::vector<std::string> seen {};
  handle_category("", "", data, data, this, 0, seen);
  /** Handle removed elements */
  std::stack<QWidget*> to_delete;
  for(auto w_iter = widgets_.begin(); w_iter != widgets_.end();)
  {
    const auto & w = *w_iter;
    if(std::find(seen.begin(), seen.end(), w.first) == seen.end())
    {
      if(w.second && remove_w_.count(w.first))
      {
        remove_w_[w.first](w.second);
        remove_w_.erase(w.first);
      }
      to_delete.push(w.second);
      w_iter = widgets_.erase(w_iter);
    }
    else
    {
      ++w_iter;
    }
  }
  while(to_delete.size())
  {
    auto w = to_delete.top();
    delete w;
    to_delete.pop();
  }
  int_server_->applyChanges();
}

MyPanel::MyPanel(QWidget * parent)
: rviz::Panel(parent)
{
  auto layout = new QVBoxLayout();
  panel = new mc_rtc_rviz::Panel(parent);
  layout->QLayout::addWidget(panel);
  setLayout(layout);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mc_rtc_rviz::MyPanel, rviz::Panel)
