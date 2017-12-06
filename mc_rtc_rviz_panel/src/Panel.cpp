#include "Panel.h"
#include "InputWidget.h"
#include "LabelWidget.h"
#include "Point3DWidget.h"

#include <mc_rtc/Configuration.h>

namespace mc_rtc_rviz
{

Panel::Panel(QWidget * parent)
: rviz::Panel(parent),
  mc_control::ControllerClient("ipc:///tmp/mc_rtc_pub.ipc", "ipc:///tmp/mc_rtc_rep.ipc"),
  tabW_(parent),
  nh_(),
  int_server_("mc_rtc_rviz_interactive_markers")
{
  auto mainLayout = new QVBoxLayout();
  mainLayout->addWidget(&tabW_);
  setLayout(mainLayout);
  connect(this, SIGNAL(gotData(const char*)),
          this, SLOT(handle_gui_state_impl(const char*)));
}

namespace
{
  BaseWidget * makeEntry(const std::vector<std::string> & category,
                         const std::string & name,
                         const mc_rtc::Configuration & data,
                         mc_control::ControllerClient & client,
                         interactive_markers::InteractiveMarkerServer & int_server)
  {
    auto gui_type = data("GUI")("type");
    if(gui_type == "Label")
    {
      return new LabelWidget(name, data);
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
    if(gui_type == "Input")
    {
      return new InputWidget(name, data,
          [&client,category,name](const mc_rtc::Configuration & data)
          {
            client.send_request(category, name, data);
          });
    }
    std::cerr << "Cannot handle the provided GUI type " << gui_type << std::endl;
    return nullptr;
  }

  QWidget * makeCategoryWidget()
  {
    auto ret = new QWidget();
    ret->setLayout(new QVBoxLayout());
    return ret;
  }

  QWidget * makeGroupWidget(const std::string & name,
                            QWidget * parent)
  {
    auto gb = new QGroupBox(name.c_str(), parent);
    gb->setLayout(new QVBoxLayout());
    parent->layout()->addWidget(gb);
    return gb;
  }

  void updateCategoryWidget(QWidget * cat,
      const std::string & cat_name,
      std::map<std::string, QWidget *> & cat_widgets,
      const std::map<std::string, mc_rtc::Configuration> & entries,
      mc_control::ControllerClient & client,
      interactive_markers::InteractiveMarkerServer & int_server)
  {
    for(const auto & e : entries)
    {
      const auto & e_name = e.first;
      std::map<std::string, mc_rtc::Configuration> data = e.second;
      if(!cat_widgets.count(e_name))
      {
        cat_widgets[e_name] = makeGroupWidget(e_name, cat);
      }
      auto group_w = cat_widgets[e_name];
      for(const auto & d : data)
      {
        std::string entry = e_name + "/" + d.first;
        if(!cat_widgets.count(entry))
        {
          auto w = makeEntry({cat_name, e_name}, d.first, d.second, client, int_server);
          cat_widgets[entry] = w;
          if(w)
          {
            //group_w->layout()->addWidget(w);
            static_cast<QVBoxLayout*>(group_w->layout())->addLayout(w->layout);
          }
        }
        auto w = cat_widgets[entry];
        if(w)
        {
          if(d.second.has("data"))
          {
            static_cast<BaseWidget*>(w)->update(d.second("data"));
          }
        }
      }
    }
  }
}

void Panel::handle_gui_state(const char * data_raw, size_t size)
{
  Q_EMIT gotData(data_raw);
}

void Panel::handle_gui_state_impl(const char * data_raw)
{
  std::map<std::string, mc_rtc::Configuration> data = mc_rtc::Configuration::fromData(data_raw);
  for(const auto & c : data)
  {
    const auto & category = c.first;
    const auto & c_data = c.second;
    if(!tabs_.count(category))
    {
      tabs_[category] = makeCategoryWidget();
      tabs_index_[category] = tabW_.addTab(tabs_[category], category.c_str());
    }
    updateCategoryWidget(tabs_[category], category, widgets_[category], c_data, *this, int_server_);
  }
  /** FIXME Handle disappearance of some tabs */
  int_server_.applyChanges();
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mc_rtc_rviz::Panel, rviz::Panel)
