/*
 * Copyright 2016-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "Panel.h"
#include "PanelImpl.h"

#include "ConnectionDialog.h"

#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;

namespace mc_rtc_rviz
{

namespace
{

bfs::path getUserDirectory()
{
#ifndef WIN32
  return bfs::path(std::getenv("HOME")) / ".config";
#else
  // Should work for Windows Vista and up
  return bfs::path(std::getenv("APPDATA"));
#endif
}

bfs::path getConfigDirectory()
{
  return getUserDirectory() / "mc_rtc/rviz_panel";
}

bfs::path getConfigPath()
{
  bfs::path config = getConfigDirectory() / "rviz_panel.conf";
  if(bfs::exists(config)) { return config; }
  return getConfigDirectory() / "rviz_panel.yaml";
}

const mc_rtc::Configuration & loadPanelConfiguration()
{
  static mc_rtc::Configuration config = []()
  {
    mc_rtc::Configuration configOut;
    auto config_path = getConfigPath();
    if(bfs::exists(config_path) && bfs::is_regular(config_path)) { configOut.load(config_path.string()); }
    return configOut;
  }();
  return config;
}

void savePanelConfiguration(const mc_rtc::Configuration & config)
{
  auto config_directory = getConfigDirectory();
  if(!bfs::exists(config_directory)) { bfs::create_directories(config_directory); }
  if(!bfs::is_directory(config_directory))
  {
    mc_rtc::log::error("Cannot save configuration to {}, {} is not a directory", config_directory, config_directory);
    return;
  }
  auto config_path = getConfigPath();
  if(bfs::exists(config_path) && !bfs::is_regular(config_path))
  {
    mc_rtc::log::error("Cannot save configuration to {}, {} is not a regular file", config_path, config_path);
    return;
  }
  config.save(config_path.string());
}

const std::vector<ConnectionConfiguration> & getConnectionConfigurations()
{
  const auto & config = loadPanelConfiguration();
  static auto configs = config("URI", std::vector<ConnectionConfiguration>(1));
  return configs;
}

const ConnectionConfiguration & getConnectionConfiguration()
{
  return getConnectionConfigurations()[0];
}

double getTimeout()
{
  auto config = loadPanelConfiguration();
  return config("timeout", 2.0);
}

} // namespace

Panel::Panel(QWidget * parent)
: CategoryWidget(ClientWidgetParam{*this, parent, {{}, "ROOT"}}),
  mc_control::ControllerClient(getConnectionConfiguration().sub_uri(),
                               getConnectionConfiguration().push_uri(),
                               getTimeout()),
  impl_(new PanelImpl()), config_(loadPanelConfiguration()), connectionConfigs_(getConnectionConfigurations())
{
  qRegisterMetaType<uint64_t>("uint64_t");
  qRegisterMetaType<std::string>("std::string");
  qRegisterMetaType<std::vector<std::string>>("std::vector<std::string>");
  qRegisterMetaType<WidgetId>("WidgetId");
  qRegisterMetaType<Eigen::Vector3d>("Eigen::Vector3d");
  qRegisterMetaType<Eigen::VectorXd>("Eigen::VectorXd");
  qRegisterMetaType<sva::PTransformd>("sva::PTransformd");
  qRegisterMetaType<sva::ForceVecd>("sva::ForceVecd");
  qRegisterMetaType<mc_rtc::gui::ArrowConfig>("mc_rtc::gui::ArrowConfig");
  qRegisterMetaType<mc_rtc::gui::Color>("mc_rtc::gui::Color");
  qRegisterMetaType<mc_rtc::gui::ForceConfig>("mc_rtc::gui::ForceConfig");
  qRegisterMetaType<mc_rtc::gui::LineConfig>("mc_rtc::gui::LineConfig");
  qRegisterMetaType<mc_rtc::gui::PolyhedronConfig>("mc_rtc::gui::PolyhedronConfig");
  qRegisterMetaType<mc_rtc::gui::PointConfig>("mc_rtc::gui::PointConfig");
  qRegisterMetaType<mc_rtc::gui::plot::Range>("mc_rtc::gui::plot::Range");
  qRegisterMetaType<mc_rtc::gui::plot::Side>("mc_rtc::gui::plot::Side");
  qRegisterMetaType<mc_rtc::gui::plot::Style>("mc_rtc::gui::plot::Style");
  qRegisterMetaType<mc_rtc::gui::plot::PolygonDescription>("mc_rtc::gui::plot::PolygonDescription");
  qRegisterMetaType<std::vector<mc_rtc::gui::plot::PolygonDescription>>(
      "std::vector<mc_rtc::gui::plot::PolygonDescription>");
  qRegisterMetaType<std::vector<Eigen::Vector3d>>("std::vector<Eigen::Vector3d>");
  qRegisterMetaType<std::vector<mc_rtc::gui::Color>>("std::vector<mc_rtc::gui::Color>");
  qRegisterMetaType<std::vector<std::array<Eigen::Vector3d, 3>>>("std::vector<std::array<Eigen::Vector3d, 3>>");
  qRegisterMetaType<std::vector<std::array<mc_rtc::gui::Color, 3>>>("std::vector<std::array<mc_rtc::gui::Color, 3>>");
  qRegisterMetaType<std::vector<sva::PTransformd>>("std::vector<sva::PTransformd>");
  qRegisterMetaType<std::vector<std::vector<Eigen::Vector3d>>>("std::vector<std::vector<Eigen::Vector3d>>");
  qRegisterMetaType<std::vector<std::array<Eigen::Vector3d, 3>>>("std::vector<std::array<Eigen::Vector3d, 3>>");
  qRegisterMetaType<std::vector<std::vector<double>>>("std::vector<std::vector<double>>");
  qRegisterMetaType<rbd::parsers::Visual>("rbd::parsers::Visual");
  qRegisterMetaType<std::optional<std::vector<mc_rtc::Configuration>>>(
      "std::optional<std::vector<mc_rtc::Configuration>>");
  qRegisterMetaType<std::optional<std::pair<size_t, mc_rtc::Configuration>>>(
      "std::optional<std::pair<size_t, mc_rtc::Configuration>>");
  tree_.parent = this;
  setContextMenuPolicy(Qt::CustomContextMenu);
  connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(contextMenu(const QPoint &)));
  connect(this, SIGNAL(signal_start()), this, SLOT(got_start()));
  connect(this, SIGNAL(signal_stop()), this, SLOT(got_stop()));
#define CONNECT_SIGNAL_SLOT(...) connect(this, SIGNAL(signal_##__VA_ARGS__), this, SLOT(got_##__VA_ARGS__))
  CONNECT_SIGNAL_SLOT(category(const std::vector<std::string> &, const std::string &));
  CONNECT_SIGNAL_SLOT(label(const WidgetId &, const std::string &));
  CONNECT_SIGNAL_SLOT(array_label(const WidgetId &, const std::vector<std::string> &, const Eigen::VectorXd &));
  CONNECT_SIGNAL_SLOT(button(const WidgetId &));
  CONNECT_SIGNAL_SLOT(checkbox(const WidgetId &, bool));
  CONNECT_SIGNAL_SLOT(string_input(const WidgetId &, const std::string &));
  CONNECT_SIGNAL_SLOT(integer_input(const WidgetId &, int));
  CONNECT_SIGNAL_SLOT(number_input(const WidgetId &, double));
  CONNECT_SIGNAL_SLOT(number_slider(const WidgetId &, double, double, double));
  CONNECT_SIGNAL_SLOT(array_input(const WidgetId &, const std::vector<std::string> &, const Eigen::VectorXd &));
  CONNECT_SIGNAL_SLOT(combo_input(const WidgetId &, const std::vector<std::string> &, const std::string &));
  CONNECT_SIGNAL_SLOT(data_combo_input(const WidgetId &, const std::vector<std::string> &, const std::string &));
  CONNECT_SIGNAL_SLOT(
      point3d(const WidgetId &, const WidgetId &, bool, const Eigen::Vector3d &, const mc_rtc::gui::PointConfig &));
  CONNECT_SIGNAL_SLOT(
      trajectory(const WidgetId &, const std::vector<Eigen::Vector3d> &, const mc_rtc::gui::LineConfig &));
  CONNECT_SIGNAL_SLOT(
      trajectory(const WidgetId &, const std::vector<sva::PTransformd> &, const mc_rtc::gui::LineConfig &));
  CONNECT_SIGNAL_SLOT(trajectory(const WidgetId &, const Eigen::Vector3d &, const mc_rtc::gui::LineConfig &));
  CONNECT_SIGNAL_SLOT(trajectory(const WidgetId &, const sva::PTransformd &, const mc_rtc::gui::LineConfig &));
  CONNECT_SIGNAL_SLOT(
      polygon(const WidgetId &, const std::vector<std::vector<Eigen::Vector3d>> &, const mc_rtc::gui::LineConfig &));
  CONNECT_SIGNAL_SLOT(polyhedron(const WidgetId &, const std::vector<std::array<Eigen::Vector3d, 3>> &,
                                 const std::vector<std::array<mc_rtc::gui::Color, 3>> &,
                                 const mc_rtc::gui::PolyhedronConfig &));
  CONNECT_SIGNAL_SLOT(force(const WidgetId &, const WidgetId &, const sva::ForceVecd &, const sva::PTransformd &,
                            const mc_rtc::gui::ForceConfig &, bool));
  CONNECT_SIGNAL_SLOT(arrow(const WidgetId &, const WidgetId &, const Eigen::Vector3d &, const Eigen::Vector3d &,
                            const mc_rtc::gui::ArrowConfig &, bool));
  CONNECT_SIGNAL_SLOT(rotation(const WidgetId &, const WidgetId &, bool, const sva::PTransformd &));
  CONNECT_SIGNAL_SLOT(transform(const WidgetId &, const WidgetId &, bool, const sva::PTransformd &));
  CONNECT_SIGNAL_SLOT(xytheta(const WidgetId &, const WidgetId &, bool, const Eigen::Vector3d &, double));
  CONNECT_SIGNAL_SLOT(schema(const WidgetId &, const std::string &));
  CONNECT_SIGNAL_SLOT(table_start(const WidgetId &, const std::vector<std::string> &));
  CONNECT_SIGNAL_SLOT(table_row(const WidgetId &, const std::vector<std::string> &));
  CONNECT_SIGNAL_SLOT(table_end(const WidgetId &));
  CONNECT_SIGNAL_SLOT(robot(const WidgetId &, const std::vector<std::string> &,
                            const std::vector<std::vector<double>> &, const sva::PTransformd &));
  CONNECT_SIGNAL_SLOT(visual(const WidgetId &, const rbd::parsers::Visual &, const sva::PTransformd &));
  CONNECT_SIGNAL_SLOT(form(const WidgetId &));
  CONNECT_SIGNAL_SLOT(form_checkbox(const WidgetId &, const std::string &, bool, bool, bool));
  CONNECT_SIGNAL_SLOT(form_integer_input(const WidgetId &, const std::string &, bool, int, bool));
  CONNECT_SIGNAL_SLOT(form_number_input(const WidgetId &, const std::string &, bool, double, bool));
  CONNECT_SIGNAL_SLOT(form_string_input(const WidgetId &, const std::string &, bool, const std::string &, bool));
  CONNECT_SIGNAL_SLOT(
      form_array_input(const WidgetId &, const std::string &, bool, const Eigen::VectorXd &, bool, bool));
  CONNECT_SIGNAL_SLOT(
      form_combo_input(const WidgetId &, const std::string &, bool, const std::vector<std::string> &, bool, int));
  CONNECT_SIGNAL_SLOT(
      form_data_combo_input(const WidgetId &, const std::string &, bool, const std::vector<std::string> &, bool));
  CONNECT_SIGNAL_SLOT(
      form_point3d_input(const WidgetId &, const std::string &, bool, const Eigen::Vector3d &, bool, bool));
  CONNECT_SIGNAL_SLOT(
      form_rotation_input(const WidgetId &, const std::string &, bool, const sva::PTransformd &, bool, bool));
  CONNECT_SIGNAL_SLOT(
      form_transform_input(const WidgetId &, const std::string &, bool, const sva::PTransformd &, bool, bool));
  CONNECT_SIGNAL_SLOT(start_form_object_input(const std::string &, bool));
  CONNECT_SIGNAL_SLOT(end_form_object_input());
  CONNECT_SIGNAL_SLOT(
      start_form_generic_array_input(const std::string &, bool, std::optional<std::vector<mc_rtc::Configuration>>));
  CONNECT_SIGNAL_SLOT(end_form_generic_array_input());
  CONNECT_SIGNAL_SLOT(start_form_one_of_input(const std::string &, bool,
                                              const std::optional<std::pair<size_t, mc_rtc::Configuration>> &));
  CONNECT_SIGNAL_SLOT(end_form_one_of_input());
  CONNECT_SIGNAL_SLOT(start_plot(uint64_t, const std::string &));
  CONNECT_SIGNAL_SLOT(plot_setup_xaxis(uint64_t, const std::string &, const mc_rtc::gui::plot::Range &));
  CONNECT_SIGNAL_SLOT(plot_setup_yaxis_left(uint64_t, const std::string &, const mc_rtc::gui::plot::Range &));
  CONNECT_SIGNAL_SLOT(plot_setup_yaxis_right(uint64_t, const std::string &, const mc_rtc::gui::plot::Range &));
  CONNECT_SIGNAL_SLOT(plot_point(uint64_t, uint64_t, const std::string &, double, double, mc_rtc::gui::Color,
                                 mc_rtc::gui::plot::Style, mc_rtc::gui::plot::Side));
  CONNECT_SIGNAL_SLOT(plot_polygon(uint64_t, uint64_t, const std::string &,
                                   const mc_rtc::gui::plot::PolygonDescription &, mc_rtc::gui::plot::Side));
  CONNECT_SIGNAL_SLOT(plot_polygons(uint64_t, uint64_t, const std::string &,
                                    const std::vector<mc_rtc::gui::plot::PolygonDescription> &,
                                    mc_rtc::gui::plot::Side));
  CONNECT_SIGNAL_SLOT(end_plot(uint64_t));
#undef CONNECT_SIGNAL_SLOT
  mc_control::ControllerClient::start();
}

void Panel::started()
{
  Q_EMIT signal_start();
}

void Panel::got_start()
{
  tree_.start();
  latestWidget_ = nullptr;
}

void Panel::stopped()
{
  Q_EMIT signal_stop();
}

void Panel::got_stop()
{
  tree_.clean();
  this->clean();
  impl_->int_server_->applyChanges();
#ifdef MC_RTC_ROS_IS_ROS2
  impl_->marker_array_pub_->publish(impl_->marker_array_);
  if(rclcpp::ok()) { rclcpp::spin_some(impl_->nh_); }
#else
  impl_->marker_array_pub_.publish(impl_->marker_array_);
  if(ros::ok()) { ros::spinOnce(); }
#endif
  impl_->marker_array_.markers.clear();
}

Panel::~Panel()
{
  got_start();
  got_stop();
}

void Panel::category(const std::vector<std::string> & parent, const std::string & category)
{
  Q_EMIT signal_category(parent, category);
}

void Panel::got_category(const std::vector<std::string> & parent, const std::string & category)
{
  auto & tree = get_category(parent);
  if(tree.sub_trees_.count(category) == 0)
  {
    auto cat = new CategoryWidget(ClientWidgetParam{*this, tree.parent, {parent, category}});
    tree.parent->addWidget(cat);
    tree.sub_trees_[category].parent = cat;
  }
  tree.sub_trees_[category].parent->seen();
}

Panel::WidgetTree & Panel::get_category(const std::vector<std::string> & category)
{
  auto ret = &tree_;
  for(const auto & c : category) { ret = &(ret->sub_trees_[c]); }
  return *ret;
}

void Panel::WidgetTree::start()
{
  if(parent) { parent->resetSeen(); }
  for(auto & st : sub_trees_) { st.second.start(); }
}

void Panel::WidgetTree::clean()
{
  for(auto it = sub_trees_.begin(); it != sub_trees_.end();)
  {

    auto & t = it->second;
    t.clean();
    size_t rem = 0;
    if(t.parent) { rem = t.parent->clean(); }
    if(rem == 0 && t.sub_trees_.size() == 0)
    {
      if(parent && t.parent) { parent->removeWidget(t.parent); }
      sub_trees_.erase(it++);
    }
    else { ++it; }
  }
}

void Panel::contextMenu(const QPoint & pos)
{
  QMenu menu("Context menu", this);
  QAction edit("Edit connection parameters", this);
  connect(&edit, SIGNAL(triggered()), this, SLOT(contextMenu_editConnection()));
  menu.addAction(&edit);
  QAction reconnect("Reconnect", this);
  connect(&reconnect, SIGNAL(triggered()), this, SLOT(contextMenu_reconnect()));
  menu.addAction(&reconnect);
  menu.exec(mapToGlobal(pos));
}

void Panel::contextMenu_editConnection()
{
  auto configs = connectionConfigs_;
  double timeout = timeout_;
  ConnectionDialog dialog(configs, timeout, this);
  if(dialog.exec())
  {
    try
    {
      const auto & cfg = configs[0];
      reconnect(cfg.sub_uri(), cfg.push_uri());
    }
    catch(std::runtime_error & exc)
    {
      mc_rtc::log::error("Reconnection failed with provided URIs");
      exc.what();
      return;
    }
    connectionConfigs_ = configs;
    config_.add("URI", configs);
    config_.add("timeout", timeout);
    this->timeout(timeout);
    savePanelConfiguration(config_);
  }
}

void Panel::contextMenu_reconnect()
{
  reconnect(connectionConfigs_[0].sub_uri(), connectionConfigs_[0].push_uri());
}

bool Panel::visible(const WidgetId & id) const
{
  return config(id)("visible", true);
}

void Panel::visible(const WidgetId & id, bool visibility)
{
  config(id).add("visible", visibility);
  savePanelConfiguration(config_);
}

mc_rtc::Configuration Panel::config(const WidgetId & id) const
{
  if(!config_.has("widgets")) { config_.add("widgets"); }
  auto ret = config_("widgets");
  for(const auto & c : id.category)
  {
    if(!ret.has(c)) { ret.add(c); }
    ret = ret(c);
  }
  if(!ret.has(id.name)) { ret.add(id.name); }
  return ret(id.name);
}

Time Panel::now() const
{
#ifdef MC_RTC_ROS_IS_ROS2
  return impl_->nh_->now();
#else
  return ros::Time::now();
#endif
}

} // namespace mc_rtc_rviz
