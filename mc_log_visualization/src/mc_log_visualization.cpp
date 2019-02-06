#include <mc_control/generic_gripper.h>
#include <mc_rbdyn/PolygonInterpolator.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/Robots.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/logging.h>
#include <mc_rtc/ros.h>

#include <RBDyn/CoM.h>
#include <RBDyn/FK.h>

#include <ros/ros.h>

#include "IconsFontAwesome.h"
#include "LogReader.h"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "util.h"
#include <GLFW/glfw3.h>
#include <fstream>
#include <thread>

namespace
{
static const std::string FAWESOME_TTF = _DATA_PATH "/fontawesome-webfont.ttf";

static void error_callback(int error, const char * description)
{
  std::cerr << "Error " << error << ": " << description << std::endl;
}

void setStyle()
{
  ImGuiStyle & style = ImGui::GetStyle();

  // light style from Pacôme Danhiez (user itamago) https://github.com/ocornut/imgui/pull/511#issuecomment-175719267
  style.WindowRounding = 0.f;
  style.FrameRounding = 0.f;
  style.ItemSpacing = {10, 10};
  style.Colors[ImGuiCol_Text] = ImVec4(0.00f, 0.00f, 0.00f, 1.00f);
  style.Colors[ImGuiCol_TextDisabled] = ImVec4(0.60f, 0.60f, 0.60f, 1.00f);
  style.Colors[ImGuiCol_WindowBg] = ImVec4(0.94f, 0.94f, 0.94f, 1.00f);
  style.Colors[ImGuiCol_ChildWindowBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
  style.Colors[ImGuiCol_Border] = ImVec4(0.00f, 0.00f, 0.00f, 0.39f);
  style.Colors[ImGuiCol_BorderShadow] = ImVec4(1.00f, 1.00f, 1.00f, 0.10f);
  style.Colors[ImGuiCol_FrameBg] = ImVec4(1.00f, 1.00f, 1.00f, 1.00f);
  style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.26f, 0.59f, 0.98f, 0.40f);
  style.Colors[ImGuiCol_FrameBgActive] = ImVec4(0.26f, 0.59f, 0.98f, 0.67f);
  style.Colors[ImGuiCol_TitleBg] = ImVec4(0.96f, 0.96f, 0.96f, 1.00f);
  style.Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(1.00f, 1.00f, 1.00f, 0.51f);
  style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.82f, 0.82f, 0.82f, 1.00f);
  style.Colors[ImGuiCol_MenuBarBg] = ImVec4(0.86f, 0.86f, 0.86f, 1.00f);
  style.Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.98f, 0.98f, 0.98f, 0.53f);
  style.Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.69f, 0.69f, 0.69f, 0.80f);
  style.Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.49f, 0.49f, 0.49f, 0.80f);
  style.Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.49f, 0.49f, 0.49f, 1.00f);
  style.Colors[ImGuiCol_ComboBg] = ImVec4(0.86f, 0.86f, 0.86f, 0.99f);
  style.Colors[ImGuiCol_CheckMark] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
  style.Colors[ImGuiCol_SliderGrab] = ImVec4(0.26f, 0.59f, 0.98f, 0.78f);
  style.Colors[ImGuiCol_SliderGrabActive] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
  style.Colors[ImGuiCol_Button] = ImVec4(0.26f, 0.59f, 0.98f, 0.40f);
  style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
  style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.06f, 0.53f, 0.98f, 1.00f);
  style.Colors[ImGuiCol_Header] = ImVec4(0.26f, 0.59f, 0.98f, 0.31f);
  style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.26f, 0.59f, 0.98f, 0.80f);
  style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
  style.Colors[ImGuiCol_Column] = ImVec4(0.39f, 0.39f, 0.39f, 1.00f);
  style.Colors[ImGuiCol_ColumnHovered] = ImVec4(0.26f, 0.59f, 0.98f, 0.78f);
  style.Colors[ImGuiCol_ColumnActive] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
  style.Colors[ImGuiCol_ResizeGrip] = ImVec4(1.00f, 1.00f, 1.00f, 0.50f);
  style.Colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.26f, 0.59f, 0.98f, 0.67f);
  style.Colors[ImGuiCol_ResizeGripActive] = ImVec4(0.26f, 0.59f, 0.98f, 0.95f);
  style.Colors[ImGuiCol_CloseButton] = ImVec4(0.59f, 0.59f, 0.59f, 0.50f);
  style.Colors[ImGuiCol_CloseButtonHovered] = ImVec4(0.98f, 0.39f, 0.36f, 1.00f);
  style.Colors[ImGuiCol_CloseButtonActive] = ImVec4(0.98f, 0.39f, 0.36f, 1.00f);
  style.Colors[ImGuiCol_PlotLines] = ImVec4(0.39f, 0.39f, 0.39f, 1.00f);
  style.Colors[ImGuiCol_PlotLinesHovered] = ImVec4(1.00f, 0.43f, 0.35f, 1.00f);
  style.Colors[ImGuiCol_PlotHistogram] = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
  style.Colors[ImGuiCol_PlotHistogramHovered] = ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
  style.Colors[ImGuiCol_TextSelectedBg] = ImVec4(0.26f, 0.59f, 0.98f, 0.35f);
  style.Colors[ImGuiCol_ModalWindowDarkening] = ImVec4(0.20f, 0.20f, 0.20f, 0.35f);
}
} // namespace

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
        cur_i += playback_num;
        pub_i = 0;
      }
      if(cur_i >= max_i)
      {
        cur_i = min_i;
      }

      rt.sleep();
    }
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

    /* Start the UI */
    glfwSetErrorCallback(&error_callback);
    if(!glfwInit())
    {
      std::cerr << "Failed to init glfw" << std::endl;
      throw("Failed to init glfw");
    }
    auto window = glfwCreateWindow(300, 300, "Broadcaster UI", NULL, NULL);
    glfwMakeContextCurrent(window);

    // Setup ImGui binding
    ImGui_ImplGlfw_Init(window, true);
    ImGuiIO & io = ImGui::GetIO();
    io.Fonts->AddFontDefault();
    // merge in icons from Font Awesome
    static const ImWchar icons_ranges[] = {ICON_MIN_FA, ICON_MAX_FA, 0};
    ImFontConfig icons_config;
    icons_config.MergeMode = true;
    icons_config.PixelSnapH = true;
    io.Fonts->AddFontFromFileTTF(FAWESOME_TTF.c_str(), 13.0f, &icons_config, icons_ranges);

    io.IniFilename = nullptr;
    // Setup style
    setStyle();

    while(!glfwWindowShouldClose(window))
    {
      glfwPollEvents();
      ImGui_ImplGlfw_NewFrame();
      bool open = true;
      ImGui::Begin("mc_rtc log player", &open,
                   ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings
                       | ImGuiWindowFlags_AlwaysAutoResize);
      ImGui::SetWindowPos({10, 10});
      /* Time control */
      {
        if(paused)
        {
          if(ImGui::Button(ICON_FA_PLAY))
          {
            paused = false;
          }
        }
        else
        {
          if(ImGui::Button((ICON_FA_PAUSE)))
          {
            paused = true;
          }
        }
        ImGui::SameLine();
        int i = static_cast<int>(cur_i);
        char fmt[100];
        std::snprintf(fmt, 100, "%.3f/%.3f", log.at("t")[cur_i], max_t);
        if(ImGui::SliderInt("##timeslider", &i, min_i, max_i, fmt))
        {
          cur_i = static_cast<unsigned int>(i);
        }
      }
      /* Playback speed setter */
      {
        std::stringstream ss;
        ss << "Playback speed: x" << playback_num;
        if(playback_den != 1)
        {
          ss << "/" << playback_den;
        }
        auto ts = ImGui::CalcTextSize(ss.str().c_str());
        auto bs = ImVec2(ImGui::GetTextLineHeightWithSpacing(), ImGui::GetTextLineHeightWithSpacing());
        auto offset = (size.x - ts.x - 2 * bs.x - 20) / 2;
        ImGui::AlignFirstTextHeightToWidgets();
        ImGui::SetCursorPosX(offset);
        if(ImGui::Button(ICON_FA_MINUS, bs))
        {
          if(playback_num == 1)
          {
            playback_den *= 2;
          }
          else
          {
            playback_num /= 2;
          }
        }
        ImGui::SameLine();
        ImGui::TextUnformatted(ss.str().c_str());
        ImGui::SameLine();
        if(ImGui::Button(ICON_FA_PLUS, bs))
        {
          if(playback_den == 1)
          {
            playback_num *= 2;
          }
          else
          {
            playback_den /= 2;
          }
        }
      }
      /* Time select combo box */
      {
        auto getKeys = [](const LogReader & log) {
          std::vector<std::string> ret;
          for(const auto & e : log)
          {
            ret.push_back(e.first);
          }
          return ret;
        };
        static std::vector<std::string> comboKeys = getKeys(log);
        auto defaultIdx = [](const std::vector<std::string> & keys) {
          for(int i = 0; i < static_cast<int>(keys.size()); ++i)
          {
            if(keys[i] == "t")
            {
              return i;
            }
          }
          return -1;
        };
        static int comboIdx = defaultIdx(comboKeys);
        auto packKeys = [](const std::vector<std::string> & keys) {
          std::vector<char> out(0);
          for(const auto & k : keys)
          {
            size_t start = out.size();
            out.resize(out.size() + k.size() + 1);
            memcpy(out.data() + start, k.c_str(), k.size() + 1);
          }
          return out;
        };
        static std::vector<char> comboData = packKeys(comboKeys);
        int prevItem = comboIdx;
        if(ImGui::Combo("Time range", &comboIdx, comboData.data()))
        {
          if(!selectTime(comboKeys[comboIdx]))
          {
            comboIdx = prevItem;
          }
        }
      }
      /* Exit button */
      {
        ImGui::Separator();
        const char * btext = "Stop publisher";
        auto bsize = ImVec2(100, 25);
        ImGui::SetCursorPosX((size.x - bsize.x) / 2);
        if(ImGui::Button(btext, bsize))
        {
          glfwSetWindowShouldClose(window, GL_TRUE);
        }
      }
      prev_size = size;
      size = ImGui::GetWindowSize();
      ImGui::End();
      if(ImGui::IsKeyDown(ImGui::GetKeyIndex(ImGuiKey_Escape)))
      {
        glfwSetWindowShouldClose(window, GL_TRUE);
      }
      int display_w, display_h;
      glfwGetFramebufferSize(window, &display_w, &display_h);
      if(prev_size.x == size.x && prev_size.y == size.y)
      {
        glfwSetWindowSize(window, static_cast<int>(size.x + 20), static_cast<int>(size.y + 20));
      }
      glViewport(0, 0, display_w, display_h);
      glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
      glClear(GL_COLOR_BUFFER_BIT);
      ImGui::Render();
      glfwSwapBuffers(window);
    }

    ImGui_ImplGlfw_Shutdown();
    glfwTerminate();

    running = false;
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

  std::shared_ptr<mc_rbdyn::RobotModule> mod;
  std::shared_ptr<mc_rbdyn::Robots> robots;
  std::shared_ptr<mc_rbdyn::Robots> real_robots;
  std::map<std::string, std::shared_ptr<mc_control::Gripper>> grippers;

  /* Store plan data */
  std::vector<mc_rbdyn::PolygonInterpolator> interpolators;

  /* UI related */
  ImVec2 size;
  ImVec2 prev_size;
  ImVec4 clear_color = ImVec4(0.94f, 0.94f, 0.94f, 1.00f);
};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "log_visualizer");
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
