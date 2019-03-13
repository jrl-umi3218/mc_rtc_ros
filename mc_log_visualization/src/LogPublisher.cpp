#include "LogPublisher.h"

LogPublisher::LogPublisher(ros::NodeHandle & nh, const std::string & logfile, mc_rbdyn::RobotModulePtr mod, double dt)
: nh(nh), mod(mod), dt(dt), rate(1 / dt), rt(rate)
{
  log.load(logfile);
  LogRobot::Configuration conf;
  conf.rm = mod;
  conf.dt = dt;
  conf.encoders = "qIn";
  {
    conf.id = "control";
    conf.configuration = "qOut";
    conf.base = "ff";
    robot.reset(new LogRobot(conf));
  }
  {
    conf.id = "real";
    conf.configuration = "qIn";
    if(log.has("realRobot_posW"))
    {
      conf.base = "realRobot_posW";
    }
    else
    {
      conf.base = "ff";
      conf.base_rotation = "rpyIn";
      conf.base_rotation_is_imu = true;
    }
    real_robot.reset(new LogRobot(conf));
  }
}

void LogPublisher::pubThread()
{
  unsigned int pub_i = 0;

  rt = ros::Rate(rate);
  while(running)
  {
    robot->update(log, cur_i);
    real_robot->update(log, cur_i);

    /* Playback speed logic */
    pub_i++;
    if(!paused && pub_i % playback_den == 0)
    {
      if(!reversed)
      {
        cur_i += playback_num;
      }
      else
      {
        cur_i -= playback_num;
      }
      pub_i = 0;
    }
    if(cur_i >= max_i)
    {
      if(reversed)
      {
        cur_i = max_i - 1;
      }
      else
      {
        cur_i = min_i;
      }
    }
    if(cur_i < min_i)
    {
      cur_i = max_i;
    }
    cur_t = log.get("t", cur_i, cur_t);
    server->handle_requests(gui);
    server->publish(gui);
    rt.sleep();
  }
}

void LogPublisher::addRemoveExtraDataButton(const std::string & section, const std::string & entry)
{
  gui.addElement(extraDataRemoveCategory, mc_rtc::gui::Button("Remove " + entry, [this, section, entry]() {
                   gui.removeElement(extraDataCategory, entry);
                   gui.removeElement(extraDataRemoveCategory, "Remove " + entry);
                   config(section).remove(entry);
                   saveConfig();
                 }));
}

void LogPublisher::rebuildGUI()
{
  std::vector<std::string> category = {"Log visualizer"};
  gui.removeCategory(category);

  auto makeTimeElement = [this]() {
    return mc_rtc::gui::NumberSlider("#Time", [this]() { return cur_t; },
                                     [this](double time) {
                                       size_t i = min_i;
                                       double t_i = log.get<double>("t", i, 0);
                                       double t_ii = log.get<double>("t", i + 1, 0);
                                       while(t_ii < time && i < max_i - 1)
                                       {
                                         i++;
                                         t_i = t_ii;
                                         t_ii = log.get<double>("t", i + 1, t_i);
                                       }
                                       cur_i = i;
                                       cur_t = t_i;
                                     },
                                     min_t, max_t);
  };
  gui.addElement(category, mc_rtc::gui::ElementsStacking::Horizontal,
                 mc_rtc::gui::Checkbox("Paused", [this]() { return paused; }, [this]() { paused = !paused; }),
                 makeTimeElement(),
                 mc_rtc::gui::Checkbox("Reverse", [this]() { return reversed; }, [this]() { reversed = !reversed; }));

  gui.addElement(category, mc_rtc::gui::ElementsStacking::Horizontal,
                 mc_rtc::gui::Button("-",
                                     [this]() {
                                       if(playback_num == 1)
                                       {
                                         playback_den *= 2;
                                       }
                                       else
                                       {
                                         playback_num /= 2;
                                       }
                                     }),
                 mc_rtc::gui::Label("Playback speed:",
                                    [this]() {
                                      std::stringstream ss;
                                      ss << playback_num;
                                      if(playback_den != 1)
                                      {
                                        ss << "/" << playback_den;
                                      }
                                      return ss.str();
                                    }),
                 mc_rtc::gui::Button("+", [this]() {
                   if(playback_den == 1)
                   {
                     playback_num *= 2;
                   }
                   else
                   {
                     playback_den /= 2;
                   }
                 }));

  auto makeStepButton = [this](int mul) {
    int dt_ms = mul * 1000 * dt;
    std::stringstream ss;
    if(dt_ms > 0)
    {
      ss << "+";
    }
    ss << dt_ms << " ms";
    return mc_rtc::gui::Button(ss.str(), [this, mul]() {
      paused = true;
      cur_i = std::max(min_i, std::min(max_i, cur_i + mul));
    });
  };
  gui.addElement(category, mc_rtc::gui::ElementsStacking::Horizontal, makeStepButton(-100), makeStepButton(-10),
                 makeStepButton(-1), makeStepButton(1), makeStepButton(10), makeStepButton(100));

  std::vector<std::string> keys;
  for(const auto & k : log.entries())
  {
    if(k == "t" || (k.size() > 1 && k[0] == 't' && k[1] == '_'))
    {
      keys.push_back(k);
    }
  }
  gui.addElement(category, mc_rtc::gui::ComboInput("Time range", keys, [this]() { return t_data; },
                                                   [this](const std::string & t) { selectTime(t); }));
  gui.addElement(category, mc_rtc::gui::Button("Stop log replay", [this]() { running = false; }));

  category.push_back("Add extra information");
  const auto & log_entries = log.entries();
  std::vector<std::string> entries{log_entries.begin(), log_entries.end()};
  if(!log.has(extra_selected))
  {
    extra_selected = entries[0];
  }
  gui.addElement(category, mc_rtc::gui::ComboInput("Entry", entries, [this]() { return extra_selected; },
                                                   [this](const std::string & selected) {
                                                     extra_selected = selected;
                                                     rebuildGUI();
                                                   }));
  std::string entry = extra_selected;
  auto types = log.types(entry);
  bool added_something = false;
  for(const auto & t : types)
  {
    switch(t)
    {
      case mc_rtc::log::LogData_Bool:
      case mc_rtc::log::LogData_Double:
      case mc_rtc::log::LogData_UnsignedInt:
      case mc_rtc::log::LogData_UInt64:
      case mc_rtc::log::LogData_String:
      case mc_rtc::log::LogData_Vector2d:
      case mc_rtc::log::LogData_DoubleVector:
      case mc_rtc::log::LogData_MotionVecd:
      case mc_rtc::log::LogData_Quaterniond:
        added_something = true;
        gui.addElement(category, mc_rtc::gui::Button("Add " + entry + " as Label", [this, entry, t]() {
                         config("labels").add(entry, static_cast<unsigned int>(t));
                         saveConfig();
                         addLabel(entry, t);
                       }));
        break;
      case mc_rtc::log::LogData_Vector3d:
        added_something = true;
        gui.addElement(category, mc_rtc::gui::Button("Add " + entry + " as Point3D", [this, entry]() {
                         config("points").add(entry, "point");
                         saveConfig();
                         addVector3dAsPoint3D(entry);
                       }));
        break;
      case mc_rtc::log::LogData_ForceVecd:
        added_something = true;
        gui.addElement(category, mc_rtc::gui::Form("Add " + entry + " as Force",
                                                   [this, entry](const mc_rtc::Configuration & form) {
                                                     std::string surface = form("Surface");
                                                     auto c = config("forces").add(entry);
                                                     c.add("surface", surface);
                                                     saveConfig();
                                                     addForce(entry, surface);
                                                   },
                                                   mc_rtc::gui::FormComboInput("Surface", true, robot->surfaces())));
        break;
      case mc_rtc::log::LogData_PTransformd:
        added_something = true;
        gui.addElement(category, mc_rtc::gui::Button("Add " + entry + " as Transform", [this, entry]() {
                         config("transforms").add(entry, "ptransformd");
                         saveConfig();
                         addPTransformdAsTransform(entry);
                       }));
        break;
      default:
        break;
    };
  }
  if(!added_something)
  {
    gui.addElement(category, mc_rtc::gui::Label("Cannot display " + entry, []() { return ""; }));
  }
}

bool LogPublisher::selectTime(const std::string & t)
{
  size_t i = 0;
  double t_i = 0;
  double t_ii = 0;
  do
  {
    t_i = log.get<double>(t, i, 0);
    t_ii = log.get<double>(t, i + 1, 0);
  } while(t_i == t_ii && ++i < log.size() - 1);
  if(i + 1 == log.size())
  {
    return false;
  }
  if(fabs(t_ii - t_i - dt) > 1e-6)
  {
    i = i + 1;
  }
  size_t start_i = i;
  do
  {
    t_i = log.get<double>(t, i, 0);
    t_ii = log.get<double>(t, i + 1, 0);
  } while(fabs(t_ii - t_i - dt) < 1e-6 && ++i < log.size() - 1);
  if(i == start_i)
  {
    return false;
  }
  min_i = start_i;
  cur_i = min_i;
  max_i = i;
  min_t = log.get<double>(t, min_i, 0);
  cur_t = min_t;
  max_t = log.get<double>(t, max_i, 0);
  t_data = t;
  rebuildGUI();
  return true;
}

void LogPublisher::run()
{
  selectTime("t");

  server.reset(
      new mc_control::ControllerServer(dt, 10 * dt, {"ipc:///tmp/mc_rtc_pub.ipc"}, {"ipc:///tmp/mc_rtc_rep.ipc"}));

  pub_th = std::thread(std::bind(&LogPublisher::pubThread, this));
  rebuildGUI();
  loadConfig();
  pub_th.join();
}

namespace details
{

template<typename T, typename RetT = T>
void addLabel(LogPublisher & log, const std::string & entry, const T & def)
{
  log.gui.addElement(log.extraDataCategory, mc_rtc::gui::Label(entry, [&log, entry, def]() -> RetT {
                       return log.log.get<T>(entry, log.cur_i, def);
                     }));
  log.addRemoveExtraDataButton("labels", entry);
}

template<typename T>
void addLabel(LogPublisher & log, const std::string & entry, const T & def, const std::vector<std::string> & labels)
{
  log.gui.addElement(log.extraDataCategory, mc_rtc::gui::ArrayLabel(entry, labels, [&log, entry, def]() {
                       return log.log.get<T>(entry, log.cur_i, def);
                     }));
  log.addRemoveExtraDataButton("labels", entry);
}
} // namespace details

void LogPublisher::addLabel(const std::string & entry, mc_rtc::log::LogData t)
{
  switch(t)
  {
    case mc_rtc::log::LogData_Bool:
      details::addLabel<bool>(*this, entry, true);
      break;
    case mc_rtc::log::LogData_Double:
      details::addLabel<double>(*this, entry, 0);
      break;
    case mc_rtc::log::LogData_UnsignedInt:
      details::addLabel<int32_t>(*this, entry, 0);
      break;
    case mc_rtc::log::LogData_UInt64:
      details::addLabel<uint64_t, unsigned int>(*this, entry, 0);
      break;
    case mc_rtc::log::LogData_String:
      details::addLabel<std::string>(*this, entry, "");
      break;
    case mc_rtc::log::LogData_Vector2d:
      details::addLabel<Eigen::Vector2d>(*this, entry, Eigen::Vector2d::Zero(), {"x", "y"});
      break;
    case mc_rtc::log::LogData_DoubleVector:
      details::addLabel<Eigen::Vector2d>(*this, entry, {}, {});
      break;
    case mc_rtc::log::LogData_MotionVecd:
      details::addLabel<sva::MotionVecd>(*this, entry, sva::MotionVecd::Zero(), {"wx", "wy", "wz", "vx", "vy", "vz"});
      break;
    case mc_rtc::log::LogData_Quaterniond:
      details::addLabel<Eigen::Quaterniond>(*this, entry, Eigen::Quaterniond::Identity(), {"w", "x", "y", "z"});
      break;
    default:
      break;
  }
}

void LogPublisher::addVector3dAsPoint3D(const std::string & entry)
{
  gui.addElement(extraDataCategory, mc_rtc::gui::Point3D(entry, [this, entry]() {
                   return log.get<Eigen::Vector3d>(entry, cur_i, Eigen::Vector3d::Zero());
                 }));
  addRemoveExtraDataButton("points", entry);
}

void LogPublisher::addForce(const std::string & entry, const std::string & surface)
{
  gui.addElement(extraDataCategory,
                 mc_rtc::gui::Force(
                     entry, [this, entry]() { return log.get<sva::ForceVecd>(entry, cur_i, sva::ForceVecd::Zero()); },
                     [this, surface]() { return robot->robot().surfacePose(surface); }));
  addRemoveExtraDataButton("forces", entry);
}

void LogPublisher::addPTransformdAsTransform(const std::string & entry)
{
  gui.addElement(extraDataCategory, mc_rtc::gui::Transform(entry, [this, entry]() {
                   return log.get<sva::PTransformd>(entry, cur_i, sva::PTransformd::Identity());
                 }));
  addRemoveExtraDataButton("transforms", entry);
}

bfs::path LogPublisher::configPath() const
{
#ifndef WIN32
  return bfs::path(std::getenv("HOME")) / ".config/mc_rtc/log_visualizer.conf";
#else
  // Should work for Windows Vista and up
  retrn config_path = bfs::path(std::getenv("APPDATA")) / "mc_rtc/log_visualizer.conf";
#endif
}

void LogPublisher::loadConfig()
{
  auto p = configPath();
  if(bfs::exists(p))
  {
    config.load(p.string());
  }
  std::vector<std::string> requiredKeys = {"labels", "forces", "points", "transforms"};
  for(const auto & k : requiredKeys)
  {
    if(!config.has(k))
    {
      config.add(k);
    }
  }
  for(const auto & e : config("labels").keys())
  {
    if(log.has(e))
    {
      int type = config("labels")(e);
      addLabel(e, mc_rtc::log::LogData(type));
    }
  }
  for(const auto & p : config("points").keys())
  {
    if(log.has(p))
    {
      addVector3dAsPoint3D(p);
    }
  }
  for(const auto & f : config("forces").keys())
  {
    if(log.has(f))
    {
      std::string surface = config("forces")(f)("surface");
      addForce(f, surface);
    }
  }
  for(const auto & p : config("transforms").keys())
  {
    if(log.has(p))
    {
      addPTransformdAsTransform(p);
    }
  }
}

void LogPublisher::saveConfig() const
{
  config.save(configPath().string());
}
