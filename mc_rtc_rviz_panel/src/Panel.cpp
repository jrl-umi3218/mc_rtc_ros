/*
 * Copyright 2016-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "Panel.h"

#include "ArrayInputWidget.h"
#include "ArrayLabelWidget.h"
#include "ArrowInteractiveMarkerWidget.h"
#include "ButtonWidget.h"
#include "CheckboxWidget.h"
#include "ComboInputWidget.h"
#include "ConnectionDialog.h"
#include "DisplayTrajectoryWidget.h"
#include "ForceInteractiveMarkerWidget.h"
#include "FormElement.h"
#include "FormWidget.h"
#include "GenericInputWidget.h"
#include "InteractiveMarkerWidget.h"
#include "LabelWidget.h"
#include "NumberSliderWidget.h"
#include "PlotTabWidget.h"
#include "Point3DInteractiveMarkerWidget.h"
#include "PolygonMarkerWidget.h"
#include "PolyhedronMarkerWidget.h"
#include "RobotModelDisplay.h"
#include "SchemaWidget.h"
#include "TableWidget.h"
#include "TransformInteractiveMarkerWidget.h"
#include "VisualWidget.h"
#include "XYThetaInteractiveMarkerWidget.h"

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

struct PanelImpl
{
  PanelImpl()
  {
#ifdef MC_RTC_ROS_IS_ROS2
    nh_ = rclcpp::Node::make_shared("mc_rtc_rviz_panel");
    marker_array_pub_ = nh_->create_publisher<MarkerArray>("/mc_rtc_rviz", 0);
    int_server_ = std::make_shared<InteractiveMarkerServer>("mc_rtc_rviz_interactive_markers", nh_);
#else
    int_server_ = std::make_shared<InteractiveMarkerServer>("mc_rtc_rviz_interactive_markers");
    marker_array_pub_ = nh_.advertise<MarkerArray>("/mc_rtc_rviz", 0);
#endif
  }
#ifdef MC_RTC_ROS_IS_ROS2
  mc_rtc::NodeHandlePtr nh_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_array_pub_;
#else
  ros::NodeHandle nh_;
  ros::Publisher marker_array_pub_;
#endif
  std::shared_ptr<InteractiveMarkerServer> int_server_;
  MarkerArray marker_array_;
};

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

void Panel::label(const WidgetId & id, const std::string & dataIn)
{
  Q_EMIT signal_label(id, dataIn);
}

void Panel::array_label(const WidgetId & id, const std::vector<std::string> & labels, const Eigen::VectorXd & dataIn)
{
  Q_EMIT signal_array_label(id, labels, dataIn);
}

void Panel::button(const WidgetId & id)
{
  Q_EMIT signal_button(id);
}

void Panel::checkbox(const WidgetId & id, bool state)
{
  Q_EMIT signal_checkbox(id, state);
}

void Panel::string_input(const WidgetId & id, const std::string & dataIn)
{
  Q_EMIT signal_string_input(id, dataIn);
}

void Panel::integer_input(const WidgetId & id, int dataIn)
{
  Q_EMIT signal_integer_input(id, dataIn);
}

void Panel::number_input(const WidgetId & id, double dataIn)
{
  Q_EMIT signal_number_input(id, dataIn);
}

void Panel::number_slider(const WidgetId & id, double dataIn, double min, double max)
{
  Q_EMIT signal_number_slider(id, dataIn, min, max);
}

void Panel::array_input(const WidgetId & id, const std::vector<std::string> & inputs, const Eigen::VectorXd & dataIn)
{
  Q_EMIT signal_array_input(id, inputs, dataIn);
}

void Panel::combo_input(const WidgetId & id, const std::vector<std::string> & values, const std::string & dataIn)
{
  Q_EMIT signal_combo_input(id, values, dataIn);
}

void Panel::data_combo_input(const WidgetId & id, const std::vector<std::string> & ref, const std::string & dataIn)
{
  Q_EMIT signal_data_combo_input(id, ref, dataIn);
}

void Panel::point3d(const WidgetId & id,
                    const WidgetId & requestId,
                    bool ro,
                    const Eigen::Vector3d & pos,
                    const mc_rtc::gui::PointConfig & config)
{
  Q_EMIT signal_point3d(id, requestId, ro, pos, config);
}

void Panel::trajectory(const WidgetId & id,
                       const std::vector<Eigen::Vector3d> & points,
                       const mc_rtc::gui::LineConfig & config)
{
  Q_EMIT signal_trajectory(id, points, config);
}

void Panel::trajectory(const WidgetId & id,
                       const std::vector<sva::PTransformd> & points,
                       const mc_rtc::gui::LineConfig & config)
{
  Q_EMIT signal_trajectory(id, points, config);
}

void Panel::trajectory(const WidgetId & id, const Eigen::Vector3d & point, const mc_rtc::gui::LineConfig & config)
{
  Q_EMIT signal_trajectory(id, point, config);
}

void Panel::trajectory(const WidgetId & id, const sva::PTransformd & point, const mc_rtc::gui::LineConfig & config)
{
  Q_EMIT signal_trajectory(id, point, config);
}

void Panel::polygon(const WidgetId & id,
                    const std::vector<std::vector<Eigen::Vector3d>> & polygons,
                    const mc_rtc::gui::Color & color)
{
  polygon(id, polygons, mc_rtc::gui::LineConfig{color});
}

void Panel::polygon(const WidgetId & id,
                    const std::vector<std::vector<Eigen::Vector3d>> & polygons,
                    const mc_rtc::gui::LineConfig & config)
{
  Q_EMIT signal_polygon(id, polygons, config);
}

void Panel::polyhedron(const WidgetId & id,
                       const std::vector<std::array<Eigen::Vector3d, 3>> & triangles,
                       const std::vector<std::array<mc_rtc::gui::Color, 3>> & colors,
                       const mc_rtc::gui::PolyhedronConfig & c)
{
  Q_EMIT signal_polyhedron(id, triangles, colors, c);
}

void Panel::force(const WidgetId & id,
                  const WidgetId & requestId,
                  const sva::ForceVecd & force_,
                  const sva::PTransformd & start,
                  const mc_rtc::gui::ForceConfig & forceConfig,
                  bool ro)
{
  Q_EMIT signal_force(id, requestId, force_, start, forceConfig, ro);
}

void Panel::arrow(const WidgetId & id,
                  const WidgetId & requestId,
                  const Eigen::Vector3d & start,
                  const Eigen::Vector3d & end,
                  const mc_rtc::gui::ArrowConfig & config,
                  bool ro)
{
  Q_EMIT signal_arrow(id, requestId, start, end, config, ro);
}

void Panel::rotation(const WidgetId & id, const WidgetId & requestId, bool ro, const sva::PTransformd & pos)
{
  Q_EMIT signal_rotation(id, requestId, ro, pos);
}

void Panel::transform(const WidgetId & id, const WidgetId & requestId, bool ro, const sva::PTransformd & pos)
{
  Q_EMIT signal_transform(id, requestId, ro, pos);
}

void Panel::xytheta(const WidgetId & id,
                    const WidgetId & requestId,
                    bool ro,
                    const Eigen::Vector3d & vec,
                    double altitude)
{
  Q_EMIT signal_xytheta(id, requestId, ro, vec, altitude);
}

void Panel::schema(const WidgetId & id, const std::string & schema)
{
  Q_EMIT signal_schema(id, schema);
}

void Panel::table_start(const WidgetId & id, const std::vector<std::string> & header)
{
  Q_EMIT signal_table_start(id, header);
}

void Panel::table_row(const WidgetId & id, const std::vector<std::string> & dataIn)
{
  Q_EMIT signal_table_row(id, dataIn);
}

void Panel::table_end(const WidgetId & id)
{
  Q_EMIT signal_table_end(id);
}

void Panel::robot(const WidgetId & id,
                  const std::vector<std::string> & parameters,
                  const std::vector<std::vector<double>> & q,
                  const sva::PTransformd & posW)
{
  Q_EMIT signal_robot(id, parameters, q, posW);
}

void Panel::visual(const WidgetId & id, const rbd::parsers::Visual & visual, const sva::PTransformd & pos)
{
  Q_EMIT signal_visual(id, visual, pos);
}

void Panel::form(const WidgetId & id)
{
  Q_EMIT signal_form(id);
}

void Panel::form_checkbox(const WidgetId & formId, const std::string & name, bool required, bool def, bool def_from_user)
{
  Q_EMIT signal_form_checkbox(formId, name, required, def, def_from_user);
}

void Panel::form_integer_input(const WidgetId & formId,
                               const std::string & name,
                               bool required,
                               int def,
                               bool def_from_user)
{
  Q_EMIT signal_form_integer_input(formId, name, required, def, def_from_user);
}

void Panel::form_number_input(const WidgetId & formId,
                              const std::string & name,
                              bool required,
                              double def,
                              bool def_from_user)
{
  Q_EMIT signal_form_number_input(formId, name, required, def, def_from_user);
}

void Panel::form_string_input(const WidgetId & formId,
                              const std::string & name,
                              bool required,
                              const std::string & def,
                              bool def_from_user)
{
  Q_EMIT signal_form_string_input(formId, name, required, def, def_from_user);
}

void Panel::form_array_input(const WidgetId & formId,
                             const std::string & name,
                             bool required,
                             const Eigen::VectorXd & def,
                             bool fixed_size,
                             bool def_from_user)
{
  Q_EMIT signal_form_array_input(formId, name, required, def, fixed_size, def_from_user);
}

void Panel::form_combo_input(const WidgetId & formId,
                             const std::string & name,
                             bool required,
                             const std::vector<std::string> & values,
                             bool send_index,
                             int def)
{
  Q_EMIT signal_form_combo_input(formId, name, required, values, send_index, def);
}

void Panel::form_data_combo_input(const WidgetId & formId,
                                  const std::string & name,
                                  bool required,
                                  const std::vector<std::string> & ref,
                                  bool send_index)
{
  Q_EMIT signal_form_data_combo_input(formId, name, required, ref, send_index);
}

void Panel::form_point3d_input(const WidgetId & formId,
                               const std::string & name,
                               bool required,
                               const Eigen::Vector3d & default_,
                               bool default_from_user,
                               bool interactive)
{
  Q_EMIT signal_form_point3d_input(formId, name, required, default_, default_from_user, interactive);
}

void Panel::form_rotation_input(const WidgetId & formId,
                                const std::string & name,
                                bool required,
                                const sva::PTransformd & default_,
                                bool default_from_user,
                                bool interactive)
{
  Q_EMIT signal_form_rotation_input(formId, name, required, default_, default_from_user, interactive);
}

void Panel::form_transform_input(const WidgetId & formId,
                                 const std::string & name,
                                 bool required,
                                 const sva::PTransformd & default_,
                                 bool default_from_user,
                                 bool interactive)
{
  Q_EMIT signal_form_transform_input(formId, name, required, default_, default_from_user, interactive);
}

void Panel::start_form_object_input(const std::string & name, bool required)
{
  Q_EMIT signal_start_form_object_input(name, required);
}

void Panel::end_form_object_input()
{
  Q_EMIT signal_end_form_object_input();
}

void Panel::start_form_generic_array_input(const std::string & name,
                                           bool required,
                                           std::optional<std::vector<mc_rtc::Configuration>> data)
{
  Q_EMIT signal_start_form_generic_array_input(name, required, data);
}

void Panel::end_form_generic_array_input()
{
  Q_EMIT signal_end_form_generic_array_input();
}

void Panel::start_form_one_of_input(const std::string & name,
                                    bool required,
                                    const std::optional<std::pair<size_t, mc_rtc::Configuration>> & data)
{
  Q_EMIT signal_start_form_one_of_input(name, required, data);
}

void Panel::end_form_one_of_input()
{
  Q_EMIT signal_end_form_one_of_input();
}

void Panel::start_plot(uint64_t id, const std::string & title)
{
  Q_EMIT signal_start_plot(id, title);
}

void Panel::plot_setup_xaxis(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  Q_EMIT signal_plot_setup_xaxis(id, legend, range);
}

void Panel::plot_setup_yaxis_left(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  Q_EMIT signal_plot_setup_yaxis_left(id, legend, range);
}

void Panel::plot_setup_yaxis_right(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  Q_EMIT signal_plot_setup_yaxis_right(id, legend, range);
}

void Panel::plot_point(uint64_t id,
                       uint64_t did,
                       const std::string & legend,
                       double x,
                       double y,
                       mc_rtc::gui::Color color,
                       mc_rtc::gui::plot::Style style,
                       mc_rtc::gui::plot::Side side)
{
  Q_EMIT signal_plot_point(id, did, legend, x, y, color, style, side);
}

void Panel::plot_polygon(uint64_t id,
                         uint64_t did,
                         const std::string & legend,
                         const mc_rtc::gui::plot::PolygonDescription & polygon,
                         mc_rtc::gui::plot::Side side)
{
  Q_EMIT signal_plot_polygon(id, did, legend, polygon, side);
}

void Panel::plot_polygons(uint64_t id,
                          uint64_t did,
                          const std::string & legend,
                          const std::vector<mc_rtc::gui::plot::PolygonDescription> & polygons,
                          mc_rtc::gui::plot::Side side)
{
  Q_EMIT signal_plot_polygons(id, did, legend, polygons, side);
}

void Panel::end_plot(uint64_t id)
{
  Q_EMIT signal_end_plot(id);
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

void Panel::got_label(const WidgetId & id, const std::string & dataIn)
{
  auto & w = get_widget<LabelWidget>(id);
  w.update(dataIn);
}

void Panel::got_array_label(const WidgetId & id,
                            const std::vector<std::string> & labels,
                            const Eigen::VectorXd & dataIn)
{
  auto & w = get_widget<ArrayLabelWidget>(id, labels);
  w.update(dataIn);
}

void Panel::got_button(const WidgetId & id)
{
  get_widget<ButtonWidget>(id);
}

void Panel::got_checkbox(const WidgetId & id, bool state)
{
  auto & w = get_widget<CheckboxWidget>(id);
  w.update(state);
}

void Panel::got_string_input(const WidgetId & id, const std::string & dataIn)
{
  auto & w = get_widget<StringInputWidget>(id);
  w.update(dataIn);
}

void Panel::got_integer_input(const WidgetId & id, int dataIn)
{
  auto & w = get_widget<IntegerInputWidget>(id);
  w.update(dataIn);
}

void Panel::got_number_input(const WidgetId & id, double dataIn)
{
  auto & w = get_widget<NumberInputWidget>(id);
  w.update(dataIn);
}

void Panel::got_number_slider(const WidgetId & id, double dataIn, double min, double max)
{
  auto & w = get_widget<NumberSliderWidget>(id, min, max);
  w.update(dataIn, min, max);
}

void Panel::got_array_input(const WidgetId & id,
                            const std::vector<std::string> & inputs,
                            const Eigen::VectorXd & dataIn)
{
  auto & w = get_widget<ArrayInputWidget>(id, inputs);
  w.update(dataIn);
}

void Panel::got_combo_input(const WidgetId & id, const std::vector<std::string> & values, const std::string & dataIn)
{
  auto & w = get_widget<ComboInputWidget>(id, values);
  w.update(dataIn, values);
}

void Panel::got_data_combo_input(const WidgetId & id,
                                 const std::vector<std::string> & values,
                                 const std::string & dataIn)
{
  auto & w = get_widget<ComboInputWidget>(id, data_, values);
  w.update(dataIn, data_, values);
}

void Panel::got_point3d(const WidgetId & id,
                        const WidgetId & requestId,
                        bool ro,
                        const Eigen::Vector3d & pos,
                        const mc_rtc::gui::PointConfig & config)
{
  auto label = latestWidget_;
  auto & w = get_widget<Point3DInteractiveMarkerWidget>(id, requestId, impl_->int_server_, config, !ro, label);
  w.update(pos);
}

void Panel::got_rotation(const WidgetId & id, const WidgetId & requestId, bool ro, const sva::PTransformd & pos)
{
  auto label = latestWidget_;
  auto & w = get_widget<TransformInteractiveMarkerWidget>(id, requestId, impl_->int_server_, !ro, false, label);
  w.update(pos);
}

void Panel::got_transform(const WidgetId & id, const WidgetId & requestId, bool ro, const sva::PTransformd & pos)
{
  auto label = latestWidget_;
  auto & w = get_widget<TransformInteractiveMarkerWidget>(id, requestId, impl_->int_server_, !ro, !ro, label);
  w.update(pos);
}

void Panel::got_xytheta(const WidgetId & id,
                        const WidgetId & requestId,
                        bool ro,
                        const Eigen::Vector3d & vec,
                        double altitude)
{
  auto label = latestWidget_;
  auto & w = get_widget<XYThetaInteractiveMarkerWidget>(id, requestId, impl_->int_server_, sva::PTransformd::Identity(),
                                                        !ro, !ro, label);
  w.update(vec, altitude);
}

void Panel::got_trajectory(const WidgetId & id,
                           const std::vector<Eigen::Vector3d> & points,
                           const mc_rtc::gui::LineConfig & config)
{
  auto & w = get_widget<DisplayTrajectoryWidget>(id, impl_->marker_array_);
  w.update(points, config);
}

void Panel::got_trajectory(const WidgetId & id,
                           const std::vector<sva::PTransformd> & points,
                           const mc_rtc::gui::LineConfig & config)
{
  auto & w = get_widget<DisplayTrajectoryWidget>(id, impl_->marker_array_);
  w.update(points, config);
}

void Panel::got_trajectory(const WidgetId & id, const Eigen::Vector3d & point, const mc_rtc::gui::LineConfig & config)
{
  auto & w = get_widget<DisplayTrajectoryWidget>(id, impl_->marker_array_);
  w.update(point, config);
}

void Panel::got_trajectory(const WidgetId & id, const sva::PTransformd & point, const mc_rtc::gui::LineConfig & config)
{
  auto & w = get_widget<DisplayTrajectoryWidget>(id, impl_->marker_array_);
  w.update(point, config);
}

void Panel::got_polygon(const WidgetId & id,
                        const std::vector<std::vector<Eigen::Vector3d>> & polygons,
                        const mc_rtc::gui::LineConfig & c)
{
  auto & w = get_widget<PolygonMarkerWidget>(id, impl_->marker_array_);
  w.update(polygons, c);
}

void Panel::got_polyhedron(const WidgetId & id,
                           const std::vector<std::array<Eigen::Vector3d, 3>> & triangles,
                           const std::vector<std::array<mc_rtc::gui::Color, 3>> & colors,
                           const mc_rtc::gui::PolyhedronConfig & c)
{
  auto & w = get_widget<PolyhedronMarkerWidget>(id, impl_->marker_array_, c);
  w.update(triangles, colors);
}

void Panel::got_force(const WidgetId & id,
                      const WidgetId & requestId,
                      const sva::ForceVecd & forcep,
                      const sva::PTransformd & surface,
                      const mc_rtc::gui::ForceConfig & forceConfig,
                      bool ro)
{
  auto label = latestWidget_;
  auto & w = get_widget<ForceInteractiveMarkerWidget>(id, requestId, impl_->int_server_, impl_->marker_array_, surface,
                                                      forcep, forceConfig, ro, label);
  w.update(surface, forcep, forceConfig);
}

void Panel::got_arrow(const WidgetId & id,
                      const WidgetId & requestId,
                      const Eigen::Vector3d & start,
                      const Eigen::Vector3d & end,
                      const mc_rtc::gui::ArrowConfig & config,
                      bool ro)
{
  auto label = latestWidget_;
  auto & w = get_widget<ArrowInteractiveMarkerWidget>(id, requestId, impl_->int_server_, impl_->marker_array_, start,
                                                      end, config, !ro, !ro, label);
  w.update(start, end, config);
}

void Panel::got_schema(const WidgetId & id, const std::string & schema)
{
  get_widget<SchemaWidget>(id, schema, data_);
}

void Panel::got_table_start(const WidgetId & id, const std::vector<std::string> & header)
{
  auto & w = get_widget<TableWidget>(id);
  w.header(header);
}

void Panel::got_table_row(const WidgetId & id, const std::vector<std::string> & dataIn)
{
  auto & w = get_widget<TableWidget>(id);
  w.row(dataIn);
}

void Panel::got_table_end(const WidgetId & id)
{
  auto & w = get_widget<TableWidget>(id);
  w.finalize();
}

void Panel::got_robot(const WidgetId & id,
                      const std::vector<std::string> & parameters,
                      const std::vector<std::vector<double>> & /*q*/,
                      const sva::PTransformd & /*posW*/)
{
  auto & w = get_widget<RobotModelDisplay>(id, displayContext(), displayGroup());
  w.update(parameters[0]);
}

void Panel::got_visual(const WidgetId & id, const rbd::parsers::Visual & visual, const sva::PTransformd & pose)
{
  auto & w = get_widget<VisualWidget>(id, impl_->marker_array_);
  w.update(visual, pose);
}

void Panel::got_form(const WidgetId & id)
{
  auto & form = get_widget<FormWidget>(id);
  activeForm_ = form.container();
  form.update();
}

void Panel::got_form_checkbox(const WidgetId & formId,
                              const std::string & name,
                              bool required,
                              bool def,
                              bool def_from_user)
{
  activeForm_->element<form::Checkbox>(name, required, def, def_from_user);
}

void Panel::got_form_integer_input(const WidgetId & formId,
                                   const std::string & name,
                                   bool required,
                                   int def,
                                   bool def_from_user)
{
  activeForm_->element<form::IntegerInput>(name, required, def, def_from_user);
}

void Panel::got_form_number_input(const WidgetId & formId,
                                  const std::string & name,
                                  bool required,
                                  double def,
                                  bool def_from_user)
{
  activeForm_->element<form::NumberInput>(name, required, def, def_from_user);
}

void Panel::got_form_string_input(const WidgetId & formId,
                                  const std::string & name,
                                  bool required,
                                  const std::string & def,
                                  bool def_from_user)
{
  activeForm_->element<form::StringInput>(name, required, def, def_from_user);
}

void Panel::got_form_array_input(const WidgetId & formId,
                                 const std::string & name,
                                 bool required,
                                 const Eigen::VectorXd & def,
                                 bool fixed_size,
                                 bool def_from_user)
{
  activeForm_->element<form::NumberArrayInput>(name, required, def, fixed_size, def_from_user);
}

void Panel::got_form_combo_input(const WidgetId & formId,
                                 const std::string & name,
                                 bool required,
                                 const std::vector<std::string> & values,
                                 bool send_index,
                                 int def)
{
  activeForm_->element<form::ComboInput>(name, required, values, send_index, def);
}

void Panel::got_form_data_combo_input(const WidgetId & formId,
                                      const std::string & name,
                                      bool required,
                                      const std::vector<std::string> & ref,
                                      bool send_index)
{
  activeForm_->element<form::DataComboInput>(name, required, data_, ref, send_index);
}

void Panel::got_form_point3d_input(const WidgetId & formId,
                                   const std::string & name,
                                   bool required,
                                   const Eigen::Vector3d & default_,
                                   bool default_from_user,
                                   bool interactive)
{
  activeForm_->element<form::Point3DInput>(name, required, default_, default_from_user, interactive,
                                           impl_->int_server_);
}

void Panel::got_form_rotation_input(const WidgetId & formId,
                                    const std::string & name,
                                    bool required,
                                    const sva::PTransformd & default_,
                                    bool default_from_user,
                                    bool interactive)
{
  activeForm_->element<form::RotationInput>(name, required, default_, default_from_user, interactive,
                                            impl_->int_server_);
}

void Panel::got_form_transform_input(const WidgetId & formId,
                                     const std::string & name,
                                     bool required,
                                     const sva::PTransformd & default_,
                                     bool default_from_user,
                                     bool interactive)
{
  activeForm_->element<form::TransformInput>(name, required, default_, default_from_user, interactive,
                                             impl_->int_server_);
}

void Panel::got_start_form_object_input(const std::string & name, bool required)
{
  activeForm_ = activeForm_->element<form::Object>(name, required, activeForm_)->container();
}

void Panel::got_end_form_object_input()
{
  activeForm_->update();
  activeForm_ = activeForm_->parentForm();
}

void Panel::got_start_form_generic_array_input(const std::string & name,
                                               bool required,
                                               std::optional<std::vector<mc_rtc::Configuration>> data)
{
  auto array = activeForm_->element<form::GenericArray>(name, required, data, activeForm_);
  activeForm_ = array->container();
}

void Panel::got_end_form_generic_array_input()
{
  activeForm_->update();
  activeForm_ = activeForm_->parentForm();
}

void Panel::got_start_form_one_of_input(const std::string & name,
                                        bool required,
                                        const std::optional<std::pair<size_t, mc_rtc::Configuration>> & data)
{
  auto oneof = activeForm_->element<form::OneOf>(name, required, data, activeForm_);
  oneof->update();
  activeForm_ = oneof->container();
}

void Panel::got_end_form_one_of_input()
{
  activeForm_->update();
  activeForm_ = activeForm_->parentForm();
}

void Panel::got_start_plot(uint64_t id, const std::string & title)
{
  got_category({}, "Plots");
  auto & w = get_widget<PlotTabWidget>({{"Plots"}, ""});
  w.start_plot(id, title);
}

void Panel::got_plot_setup_xaxis(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  auto & w = get_widget<PlotTabWidget>({{"Plots"}, ""});
  w.plot_setup_xaxis(id, legend, range);
}

void Panel::got_plot_setup_yaxis_left(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  auto & w = get_widget<PlotTabWidget>({{"Plots"}, ""});
  w.plot_setup_yaxis_left(id, legend, range);
}

void Panel::got_plot_setup_yaxis_right(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range)
{
  auto & w = get_widget<PlotTabWidget>({{"Plots"}, ""});
  w.plot_setup_yaxis_right(id, legend, range);
}

void Panel::got_plot_point(uint64_t id,
                           uint64_t did,
                           const std::string & legend,
                           double x,
                           double y,
                           mc_rtc::gui::Color color,
                           mc_rtc::gui::plot::Style style,
                           mc_rtc::gui::plot::Side side)
{
  auto & w = get_widget<PlotTabWidget>({{"Plots"}, ""});
  w.plot_point(id, did, legend, x, y, color, style, side);
}

void Panel::got_plot_polygon(uint64_t id,
                             uint64_t did,
                             const std::string & legend,
                             const mc_rtc::gui::plot::PolygonDescription & polygon,
                             mc_rtc::gui::plot::Side side)
{
  auto & w = get_widget<PlotTabWidget>({{"Plots"}, ""});
  w.plot_polygon(id, did, legend, polygon, side);
}

void Panel::got_plot_polygons(uint64_t id,
                              uint64_t did,
                              const std::string & legend,
                              const std::vector<mc_rtc::gui::plot::PolygonDescription> & polygons,
                              mc_rtc::gui::plot::Side side)
{
  auto & w = get_widget<PlotTabWidget>({{"Plots"}, ""});
  w.plot_polygons(id, did, legend, polygons, side);
}

void Panel::got_end_plot(uint64_t id)
{
  auto & w = get_widget<PlotTabWidget>({{"Plots"}, ""});
  w.end_plot(id);
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
