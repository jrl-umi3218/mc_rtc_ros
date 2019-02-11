#include "LogRobot.h"

#include <mc_control/ControllerServer.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rtc/GUIState.h>
#include <mc_rtc/log/FlatLog.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>

#include <ros/ros.h>

#include <fstream>
#include <memory>
#include <thread>

struct LogPublisher
{
public:
  LogPublisher(ros::NodeHandle & nh, const std::string & logfile, mc_rbdyn::RobotModulePtr mod, double dt)
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

  void pubThread()
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

  void addRemoveExtraDataButton(const std::string & entry)
  {
    gui.addElement({"Log visualizer - Extra data", "Remove"}, mc_rtc::gui::Button("Remove " + entry, [this, entry]() {
                     gui.removeElement({"Log visualizer - Extra data"}, entry);
                     gui.removeElement({"Log visualizer - Extra data", "Remove"}, "Remove " + entry);
                   }));
  }

  template<typename GetT>
  void addPoint3D(const std::string & entry, GetT cb)
  {
    std::vector<std::string> category = {"Log visualizer - Extra data"};
    gui.addElement(category, mc_rtc::gui::Point3D(entry, cb));
    addRemoveExtraDataButton(entry);
  }

  template<typename GetForce, typename GetSurface>
  void addForce(const std::string & entry, GetForce fCb, GetSurface sCb)
  {
    std::vector<std::string> category = {"Log visualizer - Extra data"};
    gui.addElement(category, mc_rtc::gui::Force(entry, fCb, sCb));
    addRemoveExtraDataButton(entry);
  }

  void rebuildGUI()
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
        case mc_rtc::log::LogData_Vector3d:
          added_something = true;
          gui.addElement(category, mc_rtc::gui::Button("Add " + entry + " as Point3D", [this, entry]() {
                           addPoint3D(entry, [this, entry]() {
                             return log.get<Eigen::Vector3d>(entry, cur_i, Eigen::Vector3d::Zero());
                           });
                         }));
          break;
        case mc_rtc::log::LogData_ForceVecd:
          added_something = true;
          gui.addElement(category,
                         mc_rtc::gui::Form("Add " + entry + " as Force",
                                           [this, entry](const mc_rtc::Configuration & form) {
                                             std::string surface = form("Surface");
                                             addForce(
                                                 entry,
                                                 [this, entry]() {
                                                   return log.get<sva::ForceVecd>(entry, cur_i, sva::ForceVecd::Zero());
                                                 },
                                                 [this, surface]() { return robot->robot().surfacePose(surface); });
                                           },
                                           mc_rtc::gui::FormComboInput("Surface", true, robot->surfaces())));
        default:
          break;
      };
#undef LOG_ENTRY_GUI
    }
    if(!added_something)
    {
      gui.addElement(category, mc_rtc::gui::Label("Cannot display " + entry, []() { return ""; }));
    }
  }

  bool selectTime(const std::string & t)
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

  void run()
  {
    selectTime("t");

    server.reset(
        new mc_control::ControllerServer(dt, 10 * dt, {"ipc:///tmp/mc_rtc_pub.ipc"}, {"ipc:///tmp/mc_rtc_rep.ipc"}));

    pub_th = std::thread(std::bind(&LogPublisher::pubThread, this));
    rebuildGUI();
    pub_th.join();
  }

private:
  /* ROS */
  ros::NodeHandle & nh;
  std::shared_ptr<mc_rbdyn::RobotModule> mod;

  bool running = true;
  std::thread pub_th;

  /** Log data */
  mc_rtc::log::FlatLog log;

  /** Time parameters */
  double dt;
  double rate;
  ros::Rate rt;

  /** Play/pause playback */
  bool paused = false;
  /** Play in reverse */
  bool reversed = false;
  /** Playback speed numerator */
  unsigned int playback_num = 1;
  /** Playback speed denominator */
  unsigned int playback_den = 1;
  /* min/max/current playback index/time */
  double min_t = 0;
  size_t min_i = 0;
  double max_t = 0;
  size_t max_i = 0;
  double cur_t = 0;
  size_t cur_i = 0;
  /* Current time data */
  std::string t_data;
  /* Current extra entry menu currently selected */
  std::string extra_selected = "";

  std::unique_ptr<LogRobot> robot;
  std::unique_ptr<LogRobot> real_robot;

  /* UI related */
  mc_rtc::gui::StateBuilder gui;
  std::unique_ptr<mc_control::ControllerServer> server;
};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "log_visualizer", ros::init_options::NoSigintHandler);
  auto nh = mc_rtc::ROSBridge::get_node_handle();
  if(!nh)
  {
    LOG_ERROR_AND_THROW(std::runtime_error, "Failed to initialized node handle")
  }
  std::string log = "";
  {
    std::string param;
    nh->searchParam("log", param);
    nh->getParam(param, log);
  }
  mc_rbdyn::RobotModulePtr mod;
  {
    std::vector<std::string> robot_params;
    std::string param;
    nh->searchParam("robot_module", param);
    nh->getParam(param, robot_params);
    if(robot_params.size() == 1)
    {
      mod = mc_rbdyn::RobotLoader::get_robot_module(robot_params[0]);
    }
    else if(robot_params.size() == 2)
    {
      mod = mc_rbdyn::RobotLoader::get_robot_module(robot_params[0], robot_params[1]);
    }
    else if(robot_params.size() == 3)
    {
      mod = mc_rbdyn::RobotLoader::get_robot_module(robot_params[0], robot_params[1], robot_params[2]);
    }
    else
    {
      LOG_ERROR_AND_THROW(std::runtime_error, "log_visualization cannot handle the robot_params it was given")
    }
  }
  double dt = 0.005;
  {
    std::string param = "";
    nh->searchParam("dt", param);
    nh->getParam(param, dt);
  }

  LOG_INFO("Replaying log: " << log)
  LogPublisher appli(*nh, log, mod, dt);
  appli.run();

  return 0;
}
