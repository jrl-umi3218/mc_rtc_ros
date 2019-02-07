#include <mc_control/ControllerServer.h>
#include <mc_control/generic_gripper.h>
#include <mc_rbdyn/PolygonInterpolator.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/GUIState.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>

#include <RBDyn/CoM.h>
#include <RBDyn/FK.h>

#include <ros/ros.h>

#include "LogReader.h"
#include "util.h"
#include <fstream>
#include <memory>
#include <thread>

struct LogPublisher
{
public:
  LogPublisher(ros::NodeHandle & nh, const std::string & logfile) : nh(nh)
  {
    log.read(logfile);
  }

  void pubThread()
  {
    auto & robot = robots->robot();
    auto & real_robot = real_robots->robot();
    unsigned int pub_i = 0;
    auto ref_joint_order = mod->ref_joint_order();

    mc_rtc::RobotPublisher publisher("control/", rate, dt);
    mc_rtc::RobotPublisher real_publisher("real/", rate, dt);

    CoMPublisher com_pub = CoMPublisher(*mc_rtc::ROSBridge::get_node_handle());
    CoMPublisher target_com_pub = CoMPublisher(*mc_rtc::ROSBridge::get_node_handle(), "target_com_marker");
    PolygonPublisher poly_pub = PolygonPublisher(*mc_rtc::ROSBridge::get_node_handle());

    rt = ros::Rate(rate);
    while(running)
    {
      /* Publication */
      robot.mbc().q[0] = {log.at("ff_qw")[cur_i],  -log.at("ff_qx")[cur_i], -log.at("ff_qy")[cur_i],
                          -log.at("ff_qz")[cur_i], log.at("ff_tx")[cur_i],  log.at("ff_ty")[cur_i],
                          log.at("ff_tz")[cur_i]};
      real_robot.mbc().q[0] = {1, 0, 0, 0, 0, 0, 0};
      for(size_t j = 0; j < ref_joint_order.size(); ++j)
      {
        const auto & jn = ref_joint_order[j];
        std::stringstream ss;
        ss << "qOut_" << j;
        std::stringstream ss2;
        ss2 << "qIn_" << j;
        robot.mbc().q[robot.jointIndexByName(jn)][0] = log.at(ss.str())[cur_i];
        real_robot.mbc().q[robot.jointIndexByName(jn)][0] = log.at(ss2.str())[cur_i];
        for(const auto & g : grippers)
        {
          for(size_t k = 0; k < g.second->names.size(); ++k)
          {
            const auto & gJn = g.second->names[k];
            if(gJn == jn)
            {
              std::stringstream ss2;
              ss2 << "qIn_" << j;
              g.second->_q[k] = log.at(ss2.str())[cur_i];
            }
          }
        }
      }
      rbd::forwardKinematics(robot.mb(), robot.mbc());
      auto com = rbd::computeCoM(robot.mb(), robot.mbc());
      com_pub.publish_com(com);
      if(log.count("comt_x"))
      {
        Eigen::Vector3d comt = Eigen::Vector3d(log["comt_x"][cur_i], log["comt_y"][cur_i], log["comt_z"][cur_i]);
        target_com_pub.publish_com(comt);
      }

      Eigen::Quaterniond quat;
      Eigen::Vector3d trans;
      if(log.count("realRobot_posW_qw"))
      {
        auto X_0_real =
            sva::PTransformd(Eigen::Quaterniond(log.at("realRobot_posW_qw")[cur_i], log.at("realRobot_posW_qx")[cur_i],
                                                log.at("realRobot_posW_qy")[cur_i], log.at("realRobot_posW_qz")[cur_i])
                                 .normalized(),
                             Eigen::Vector3d(log.at("realRobot_posW_tx")[cur_i], log.at("realRobot_posW_ty")[cur_i],
                                             log.at("realRobot_posW_tz")[cur_i]));
        quat = Eigen::Quaterniond(X_0_real.rotation()).inverse();
        trans = X_0_real.translation();
      }
      else
      {
        rbd::forwardKinematics(real_robot.mb(), real_robot.mbc());
        auto rot_imu = sva::PTransformd(Eigen::Quaterniond(log.at("rpyIn_w")[cur_i], log.at("rpyIn_x")[cur_i],
                                                           log.at("rpyIn_y")[cur_i], log.at("rpyIn_z")[cur_i])
                                            .normalized());
        auto rot_imu0 = sva::PTransformd(
            Eigen::Quaterniond(log.at("rpyIn_w")[0], log.at("rpyIn_x")[0], log.at("rpyIn_y")[0], log.at("rpyIn_z")[0])
                .normalized());
        rot_imu = rot_imu * rot_imu0.inv();
        auto rootPos = real_robot.mbc().bodyPosW[0];
        auto imuPos = real_robot.mbc().bodyPosW[robot.bodyIndexByName(robot.bodySensor().parentBody())];
        quat = Eigen::Quaterniond((rootPos * imuPos.inv() * rot_imu).rotation()).inverse();
        trans = Eigen::Vector3d(robot.mbc().q[0][4], robot.mbc().q[0][5], robot.mbc().q[0][6]);
      }
      real_robot.mbc().q[0] = {quat.w(), quat.x(), quat.y(), quat.z(), trans.x(), trans.y(), trans.z()};
      rbd::forwardKinematics(real_robot.mb(), real_robot.mbc());

      publisher.update(dt, robot, grippers);
      real_publisher.update(dt, real_robot, grippers);

      if(log.count("stance_index") && log.count("polygonInterpolatorPercent"))
      {
        auto poly =
            interpolators[log.at("stance_index")[cur_i]].fast_interpolate(log.at("polygonInterpolatorPercent")[cur_i]);
        poly_pub.publish_poly(poly);
      }

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
      cur_t = log.at("t")[cur_i];
      server->handle_requests(gui);
      server->publish(gui);
      rt.sleep();
    }
  }

  void rebuildGUI()
  {
    std::vector<std::string> category = {"Log visualizer"};
    gui.removeCategory(category);

    auto makeTimeElement = [this]() {
      return mc_rtc::gui::NumberSlider("#Time", [this]() { return cur_t; },
                                       [this](double time) {
                                         const auto & t = log.at("t");
                                         size_t i = min_i;
                                         for(; i < max_i - 1; ++i)
                                         {
                                           if(t[i] <= time && t[i + 1] > time)
                                           {
                                             break;
                                           }
                                         }
                                         cur_i = i;
                                         cur_t = t[i];
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
    for(const auto & p : log)
    {
      const auto & k = p.first;
      if(k == "t" || (k.size() > 1 && k[0] == 't' && k[1] == '_'))
      {
        keys.push_back(k);
      }
    }
    gui.addElement(category, mc_rtc::gui::ComboInput("Time range", keys, [this]() { return t_data; },
                                                     [this](const std::string & t) { selectTime(t); }));
    gui.addElement(category, mc_rtc::gui::Button("Stop log replay", [this]() { running = false; }));
  }

  bool selectTime(const std::string & t)
  {
    const auto & data = log.at(t);
    size_t i = 0;
    while(i + 1 < data.size() && data[i] == data[i + 1])
    {
      i++;
    }
    if(i + 1 == data.size())
    {
      return false;
    }
    if(fabs(data[i + 1] - data[i] - dt) > 1e-6)
    {
      i = i + 1;
    }
    size_t start_i = i;
    while(i + 1 < data.size() && fabs(data[i + 1] - data[i] - dt) < 1e-6)
    {
      i++;
    }
    if(i == start_i)
    {
      return false;
    }
    min_i = start_i;
    cur_i = min_i;
    max_i = i;
    min_t = data[min_i];
    cur_t = min_t;
    max_t = data[max_i];
    t_data = t;
    rebuildGUI();
    return true;
  }

  void run()
  {
    selectTime("t");
    std::vector<std::string> robot_params;
    std::string param = "";
    nh.searchParam("robot_module", param);
    nh.getParam(param, robot_params);
    nh.searchParam("dt", param);
    nh.getParam(param, dt);
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

    server.reset(
        new mc_control::ControllerServer(dt, 10 * dt, {"ipc:///tmp/mc_rtc_pub.ipc"}, {"ipc:///tmp/mc_rtc_rep.ipc"}));

    robots = mc_rbdyn::loadRobot(*mod);
    real_robots = mc_rbdyn::loadRobot(*mod);

    std::string urdfPath = mod->urdf_path;
    std::ifstream ifs(urdfPath);
    std::stringstream urdf;
    if(ifs.is_open())
    {
      urdf << ifs.rdbuf();
    }
    for(auto & g : mod->grippers())
    {
      auto g_ptr = std::make_shared<mc_control::Gripper>(robots->robot(), g.joints, urdf.str(),
                                                         std::vector<double>(g.joints.size(), 0), dt, g.reverse_limits);
      grippers[g.name] = g_ptr;
    }

    pub_th = std::thread(std::bind(&LogPublisher::pubThread, this));
    rebuildGUI();
    pub_th.join();
  }

private:
  /* ROS */
  ros::NodeHandle & nh;

  bool running = true;
  std::thread pub_th;

  LogReader log;

  double dt = 0.005;
  double rate = 1 / dt;
  ros::Rate rt = ros::Rate(rate);

  /* Play/pause playback */
  bool paused = false;
  /* Play in reverse */
  bool reversed = false;
  /* Playback speed is playback_num/playback_den */
  unsigned int playback_num = 1;
  unsigned int playback_den = 1;
  /* Current and min/max playback time */
  double min_t = 0;
  size_t min_i = 0;
  double max_t = 0;
  size_t max_i = 0;
  double cur_t = 0;
  size_t cur_i = 0;
  /* Current time data */
  std::string t_data;

  std::shared_ptr<mc_rbdyn::RobotModule> mod;
  std::shared_ptr<mc_rbdyn::Robots> robots;
  std::shared_ptr<mc_rbdyn::Robots> real_robots;
  std::map<std::string, std::shared_ptr<mc_control::Gripper>> grippers;

  /* Store plan data */
  std::vector<mc_rbdyn::PolygonInterpolator> interpolators;

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
  LOG_INFO("Replaying log: " << log)
  LogPublisher appli(*nh, log);
  appli.run();

  return 0;
}
