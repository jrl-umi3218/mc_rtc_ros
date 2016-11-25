#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>

#include <mc_control/mc_global_controller.h>

#include <ros/ros.h>

#include <chrono>
#include <iostream>

#include <mc_tcp_msgs/close_grippers.h>
#include <mc_tcp_msgs/EnableController.h>
#include <mc_tcp_msgs/get_joint_pos.h>
#include <mc_tcp_msgs/open_grippers.h>
#include <mc_tcp_msgs/play_next_stance.h>
#include <mc_tcp_msgs/send_msg.h>
#include <mc_tcp_msgs/send_recv_msg.h>
#include <mc_tcp_msgs/set_gripper.h>
#include <mc_tcp_msgs/set_joint_pos.h>

#include "ContactForcePublisher.h"

namespace
{
  template<typename T>
  T getParam(ros::NodeHandle & n, const std::string & param)
  {
    T out;
    std::string true_param;
    n.searchParam(param, true_param);
    n.getParam(true_param, out);
    return out;
  }

  template<typename T>
  T getParam(ros::NodeHandle & n, const std::string & param, const T & def)
  {
    if(n.hasParam(param))
    {
      return getParam<T>(n, param);
    }
    return def;
  }

  struct ServiceHandler
  {
    ServiceHandler(mc_control::MCGlobalController & controller)
    : controller(controller)
    {
    }

    bool close_grippers(mc_tcp_msgs::close_grippersRequest &,
                        mc_tcp_msgs::close_grippersResponse & res)
    {
      controller.setGripperOpenPercent(0);
      res.success = true;
      return true;
    }

    bool EnableController(mc_tcp_msgs::EnableControllerRequest & req,
                        mc_tcp_msgs::EnableControllerResponse & res)
    {
      res.success = controller.EnableController(req.name);
      return res.success;
    }

    bool get_joint_pos(mc_tcp_msgs::get_joint_posRequest & req,
                        mc_tcp_msgs::get_joint_posResponse & res)
    {
      res.success = controller.get_joint_pos(req.jname, res.q);
      return res.success;
    }

    bool open_grippers(mc_tcp_msgs::open_grippersRequest &,
                        mc_tcp_msgs::open_grippersResponse & res)
    {
      controller.setGripperOpenPercent(1.);
      res.success = true;
      return res.success;
    }

    bool play_next_stance(mc_tcp_msgs::play_next_stanceRequest &,
                        mc_tcp_msgs::play_next_stanceResponse & res)
    {
      res.success = controller.play_next_stance();
      return res.success;
    }

    bool send_msg(mc_tcp_msgs::send_msgRequest & req,
                        mc_tcp_msgs::send_msgResponse & res)
    {
      res.success = controller.send_msg(req.msg);
      return res.success;
    }

    bool send_recv_msg(mc_tcp_msgs::send_recv_msgRequest & req,
                        mc_tcp_msgs::send_recv_msgResponse & res)
    {
      res.success = controller.send_recv_msg(req.msg, res.msg);
      return res.success;
    }

    bool set_gripper(mc_tcp_msgs::set_gripperRequest & req,
                        mc_tcp_msgs::set_gripperResponse & res)
    {
      controller.setGripperTargetQ(req.gname, req.values);
      res.success = true;
      return res.success;
    }

    bool set_joint_pos(mc_tcp_msgs::set_joint_posRequest & req,
                        mc_tcp_msgs::set_joint_posResponse & res)
    {
      res.success = controller.set_joint_pos(req.jname, req.q);
      return res.success;
    }
  private:
    mc_control::MCGlobalController & controller;
  };
}

#ifdef MC_RTC_HAS_ROS
int main()
{
  auto nh_p = mc_rtc::ROSBridge::get_node_handle();
  if(!nh_p)
  {
    return 1;
  }
  auto nh = *nh_p;


  std::string conf = "";
  if(nh.hasParam("mc_rtc_ticker/conf"))
  {
    conf = getParam<std::string>(nh, "mc_rtc_ticker/conf");
    LOG_INFO("Configuring mc_rtc with " << conf)
  }

  mc_control::MCGlobalController controller(conf);
  double dt = controller.timestep();

  std::vector<double> q;
  if(nh.hasParam("mc_rtc_ticker/init_state"))
  {
    q = getParam<std::vector<double>>(nh, "mc_rtc_ticker/init_state");
  }
  else
  {
    auto & mbc = controller.robot().mbc();
    const auto & rjo = controller.ref_joint_order();
    for(const auto & jn : rjo)
    {
      if(controller.robot().hasJoint(jn))
      {
        for(auto & qj : mbc.q[controller.robot().jointIndexByName(jn)])
        {
          q.push_back(qj);
        }
      }
    }
  }
  controller.setEncoderValues(q);
  if(nh.hasParam("mc_rtc_ticker/init_pos"))
  {
    LOG_INFO("Using initial pos from ROS param")
    std::vector<double> pos = getParam<std::vector<double>>(nh, "mc_rtc_ticker/init_pos");
    for(const auto & pi : pos)
    {
      std::cout << pi << " ";
    }
    std::cout << std::endl;
    std::array<double, 7> p;
    std::copy_n(pos.begin(), 7, p.begin());
    controller.init(q, p);
  }
  else
  {
    controller.init(q);
  }
  controller.running = true;

  const bool bench = getParam(nh, "mc_rtc_ticker/bench", false);
  std::chrono::time_point<std::chrono::system_clock> begin, end;
  std::vector<std::chrono::duration<double>> solve_dt(0);
  if(bench)
  {
    /* Reserve a million entry */
    solve_dt.reserve(1000000);
  }
  auto report_dt = [&solve_dt](bool partial)
  {
    std::cout << "Ran for " << solve_dt.size() << " iterations: ";
    size_t start_i = partial ? solve_dt.size() - 1000 : 0;
    auto min_t = solve_dt[start_i];
    auto max_t = solve_dt[start_i];
    auto second_max_t = solve_dt[start_i];
    std::chrono::duration<double> average_t(0);
    for(size_t i = start_i; i < solve_dt.size(); ++i)
    {
      const auto & t = solve_dt[i];
      if(t > max_t)
      {
        second_max_t = max_t;
        max_t = t;
      }
      if(t < min_t)
      {
        min_t = t;
      }
      average_t += t;
    }
    if(partial) { average_t = average_t / 1000; }
    else { average_t = average_t / solve_dt.size(); }
    std::cout << " avg: " << average_t.count()*1000 << "ms, min: " << min_t.count()*1000 << "ms, max: " << max_t.count()*1000 << "ms, second max: " << second_max_t.count()*1000 << std::endl;
  };

  /* Setup controller services */
  ServiceHandler sh(controller);
#define ADV_SVC(SVC)\
  ros::ServiceServer SVC##_srv = nh.advertiseService(#SVC, &ServiceHandler::SVC, &sh);
  ADV_SVC(close_grippers)
  ADV_SVC(EnableController)
  ADV_SVC(get_joint_pos)
  ADV_SVC(open_grippers)
  ADV_SVC(play_next_stance)
  ADV_SVC(send_msg)
  ADV_SVC(send_recv_msg)
  ADV_SVC(set_gripper)
  ADV_SVC(set_joint_pos)
#undef ADV_SVC

  const bool publish_contact_forces = getParam(nh, "mc_rtc_ticker/publish_contact_forces", true);
  std::unique_ptr<mc_rtc_ros::ContactForcePublisher> cfp_ptr = nullptr;
  if(publish_contact_forces)
  {
    cfp_ptr.reset(new mc_rtc_ros::ContactForcePublisher(nh,
                                                        controller));
  }

  ros::Rate rt(1/dt);
  while(ros::ok())
  {
    if(bench)
    {
      begin = std::chrono::system_clock::now();
    }
    if(controller.run())
    {
      std::map<std::string, std::vector<double>> gAQs; // will store only the active joints values
      std::map<std::string, std::vector<double>> gQs = controller.gripperQ();
      for(const auto & g : controller.gripperActiveJoints())
      {
        const auto & gn = g.first;
        const auto & gAJs = g.second;
        auto gJs = controller.gripperJoints()[gn];
        const auto & gQ = gQs[gn];
        gAQs[gn] = {};
        for(const auto & gAJ : gAJs)
        {
          for(size_t i = 0; i < gJs.size(); ++i)
          {
            if(gJs[i] == gAJ)
            {
              gAQs[gn].push_back(gQ[i]);
              break;
            }
          }
        }
      }
      controller.setActualGripperQ(gAQs);
    }
    if(bench)
    {
      end = std::chrono::system_clock::now();
      solve_dt.push_back(end-begin);
      if(solve_dt.size() % 1000 == 0)
      {
        report_dt(true);
      }
    }
    else
    {
      ros::spinOnce();
      rt.sleep();
    }
  }

  if(bench)
  {
    report_dt(false);
  }
  if(cfp_ptr)
  {
    cfp_ptr->stop();
  }

  return 0;
}
#else
int main()
{
  std::cerr << "mc_rtc was built without ROS support, aborting" << std::endl;
  return 1;
}
#endif
