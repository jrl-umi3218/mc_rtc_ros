#include "Panel.h"

#include <mc_rtc/Configuration.h>

namespace mc_rtc_rviz
{

Panel::Panel(QWidget * parent)
: rviz::Panel(parent),
  mc_control::ControllerClient("ipc:///tmp/mc_rtc_pub.ipc", "ipc:///tmp/mc_rtc_rep.ipc"),
  tabW_(parent)
{
  auto mainLayout = new QVBoxLayout();
  mainLayout->addWidget(&tabW_);
  setLayout(mainLayout);
  connect(this, SIGNAL(gotData(const char*)),
          this, SLOT(handle_gui_state_impl(const char*)));
}

namespace
{
  QLayout * makeEntry(const std::string & name,
                      const mc_rtc::Configuration & data,
                      const mc_rtc::Configuration & gui_data)
  {
    auto gui_type = gui_data("type");
    if(gui_type == "Label")
    {
      auto hlayout = new QHBoxLayout();
      hlayout->addWidget(new QLabel(name.c_str()));

      std::string dataStr;
      bool is_vector = gui_data("vector", false);
      if(is_vector)
      {
        // FIXME In this case we may want to display the full vector in a foldable section / tooltip
        Eigen::VectorXd dataV = data;
        std::stringstream ss;
        ss << dataV.norm();
        dataStr = ss.str();
      }
      else
      {
        dataStr = data.dump().substr(0, 25);
      }
      hlayout->addWidget(new QLabel(dataStr.c_str()));

      return hlayout;
    }
    if(gui_type == "Point3D")
    {
      return nullptr;
    }
    if(gui_type == "Input")
    {
      return nullptr;
    }
    std::cerr << "Cannot handle the provided GUI type " << gui_type << std::endl;
    return nullptr;
  }

  QWidget * makeCategoryWidget(const std::map<std::string, mc_rtc::Configuration> & entries)
  {
    auto ret = new QWidget();
    auto layout = new QVBoxLayout();
    for(const auto & e : entries)
    {
      auto gb = new QGroupBox(e.first.c_str(), ret);
      auto gb_layout = new QVBoxLayout();
      layout->addWidget(gb);
      std::map<std::string, mc_rtc::Configuration> data = e.second;
      for(const auto & d : data)
      {
        auto is_gui = d.first.find("_MC_RTC_GUI") != std::string::npos;
        if(!is_gui)
        {
          auto l = makeEntry(d.first, d.second, data[d.first + "_MC_RTC_GUI"]);
          if(l) { gb_layout->addLayout(l); }
        }
      }
      gb->setLayout(gb_layout);
    }
    ret->setLayout(layout);
    return ret;
  }

  void updateCategoryWidget(QWidget * w, const std::map<std::string, mc_rtc::Configuration> & entries)
  {
  }
}

void Panel::handle_gui_state(const char * data_raw, size_t size)
{
  Q_EMIT gotData(data_raw);
}

void Panel::handle_gui_state_impl(const char * data_raw)
{
  std::map<std::string, mc_rtc::Configuration> data = mc_rtc::Configuration::fromData(data_raw);
  auto methods = data["METHODS"];
  for(const auto & c : data)
  {
    if(c.first != "METHODS") // Reserved name for methods
    {
      if(tabs_.count(c.first))
      {
        updateCategoryWidget(tabs_[c.first], c.second);
      }
      else
      {
        tabs_[c.first] = makeCategoryWidget(c.second);
        tabs_index_[c.first] = tabW_.addTab(tabs_[c.first], c.first.c_str());
      }
    }
  }
  /** FIXME Handle disappearance of some tabs */
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mc_rtc_rviz::Panel, rviz::Panel)
