/*
 * Copyright 2016-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "Panel.h"

#include "ArrayInputWidget.h"
#include "ArrayLabelWidget.h"
#include "ButtonWidget.h"
#include "CheckboxWidget.h"
#include "ComboInputWidget.h"
#include "FormElement.h"
#include "FormWidget.h"
#include "GenericInputWidget.h"
#ifndef DISABLE_ROS
#  include "ArrowInteractiveMarkerWidget.h"
#  include "DisplayTrajectoryWidget.h"
#  include "ForceInteractiveMarkerWidget.h"
#  include "InteractiveMarkerWidget.h"
#  include "Point3DInteractiveMarkerWidget.h"
#  include "PolygonMarkerWidget.h"
#  include "TransformInteractiveMarkerWidget.h"
#  include "XYThetaInteractiveMarkerWidget.h"
#endif
#include "ConnectionDialog.h"
#include "LabelWidget.h"
#include "NumberSliderWidget.h"
#include "PlotTabWidget.h"
#include "SchemaWidget.h"

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
  if(bfs::exists(config))
  {
    return config;
  }
  return getConfigDirectory() / "rviz_panel.yaml";
}

const mc_rtc::Configuration & loadPanelConfiguration()
{
  static mc_rtc::Configuration config = []() {
    mc_rtc::Configuration config;
    auto config_path = getConfigPath();
    if(bfs::exists(config_path) && bfs::is_regular(config_path))
    {
      config.load(config_path.string());
    }
    return config;
  }();
  return config;
}

void savePanelConfiguration(const mc_rtc::Configuration & config)
{
  auto config_directory = getConfigDirectory();
  if(!bfs::exists(config_directory))
  {
    bfs::create_directories(config_directory);
  }
  if(!bfs::is_directory(config_directory))
  {
    LOG_ERROR("Cannot save configuration to " << config_directory << ", " << config_directory << " is not a directory")
    return;
  }
  auto config_path = getConfigPath();
  if(bfs::exists(config_path) && !bfs::is_regular(config_path))
  {
    LOG_ERROR("Cannot save configuration to " << config_path << ", " << config_path << " is not a regular file")
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

#ifndef DISABLE_ROS
struct PanelImpl
{
  PanelImpl()
  : nh_(),
    int_server_(std::make_shared<interactive_markers::InteractiveMarkerServer>("mc_rtc_rviz_interactive_markers"))
  {
    marker_array_pub_ =
        mc_rtc::ROSBridge::get_node_handle()->advertise<visualization_msgs::MarkerArray>("/mc_rtc_rviz", 0);
  }
  ros::NodeHandle nh_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> int_server_;
  visualization_msgs::MarkerArray marker_array_;
  ros::Publisher marker_array_pub_;
};
#else
struct PanelImpl
{
};
#endif

Panel::Panel(QWidget * parent)
: CategoryWidget(ClientWidgetParam{*this, parent, {{}, "ROOT"}}), mc_control::ControllerClient(
                                                                      getConnectionConfiguration().sub_uri(),
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
  qRegisterMetaType<mc_rtc::gui::PointConfig>("mc_rtc::gui::PointConfig");
  qRegisterMetaType<mc_rtc::gui::plot::Range>("mc_rtc::gui::plot::Range");
  qRegisterMetaType<mc_rtc::gui::plot::Side>("mc_rtc::gui::plot::Side");
  qRegisterMetaType<mc_rtc::gui::plot::Style>("mc_rtc::gui::plot::Style");
  qRegisterMetaType<mc_rtc::gui::plot::PolygonDescription>("mc_rtc::gui::plot::PolygonDescription");
  qRegisterMetaType<std::vector<mc_rtc::gui::plot::PolygonDescription>>(
      "std::vector<mc_rtc::gui::plot::PolygonDescription>");
  qRegisterMetaType<std::vector<Eigen::Vector3d>>("std::vector<Eigen::Vector3d>");
  qRegisterMetaType<std::vector<sva::PTransformd>>("std::vector<sva::PTransformd>");
  qRegisterMetaType<std::vector<std::vector<Eigen::Vector3d>>>("std::vector<std::vector<Eigen::Vector3d>>");
  tree_.parent = this;
  setContextMenuPolicy(Qt::CustomContextMenu);
  connect(this, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(contextMenu(const QPoint &)));
  connect(this, SIGNAL(signal_start()), this, SLOT(got_start()));
  connect(this, SIGNAL(signal_stop()), this, SLOT(got_stop()));
  connect(this, SIGNAL(signal_category(const std::vector<std::string> &, const std::string &)), this,
          SLOT(got_category(const std::vector<std::string> &, const std::string &)));
  connect(this, SIGNAL(signal_label(const WidgetId &, const std::string &)), this,
          SLOT(got_label(const WidgetId &, const std::string &)));
  connect(this, SIGNAL(signal_array_label(const WidgetId &, const std::vector<std::string> &, const Eigen::VectorXd &)),
          this, SLOT(got_array_label(const WidgetId &, const std::vector<std::string> &, const Eigen::VectorXd &)));
  connect(this, SIGNAL(signal_button(const WidgetId &)), this, SLOT(got_button(const WidgetId &)));
  connect(this, SIGNAL(signal_checkbox(const WidgetId &, bool)), this, SLOT(got_checkbox(const WidgetId &, bool)));
  connect(this, SIGNAL(signal_string_input(const WidgetId &, const std::string &)), this,
          SLOT(got_string_input(const WidgetId &, const std::string &)));
  connect(this, SIGNAL(signal_integer_input(const WidgetId &, int)), this,
          SLOT(got_integer_input(const WidgetId &, int)));
  connect(this, SIGNAL(signal_number_input(const WidgetId &, double)), this,
          SLOT(got_number_input(const WidgetId &, double)));
  connect(this, SIGNAL(signal_number_slider(const WidgetId &, double, double, double)), this,
          SLOT(got_number_slider(const WidgetId &, double, double, double)));
  connect(this, SIGNAL(signal_array_input(const WidgetId &, const std::vector<std::string> &, const Eigen::VectorXd &)),
          this, SLOT(got_array_input(const WidgetId &, const std::vector<std::string> &, const Eigen::VectorXd &)));
  connect(this, SIGNAL(signal_combo_input(const WidgetId &, const std::vector<std::string> &, const std::string &)),
          this, SLOT(got_combo_input(const WidgetId &, const std::vector<std::string> &, const std::string &)));
  connect(this,
          SIGNAL(signal_data_combo_input(const WidgetId &, const std::vector<std::string> &, const std::string &)),
          this, SLOT(got_data_combo_input(const WidgetId &, const std::vector<std::string> &, const std::string &)));
  connect(this,
          SIGNAL(signal_point3d(const WidgetId &, const WidgetId &, bool, const Eigen::Vector3d &,
                                const mc_rtc::gui::PointConfig &)),
          this,
          SLOT(got_point3d(const WidgetId &, const WidgetId &, bool, const Eigen::Vector3d &,
                           const mc_rtc::gui::PointConfig &)));
  connect(
      this,
      SIGNAL(
          signal_trajectory(const WidgetId &, const std::vector<Eigen::Vector3d> &, const mc_rtc::gui::LineConfig &)),
      this,
      SLOT(got_trajectory(const WidgetId &, const std::vector<Eigen::Vector3d> &, const mc_rtc::gui::LineConfig &)));
  connect(
      this,
      SIGNAL(
          signal_trajectory(const WidgetId &, const std::vector<sva::PTransformd> &, const mc_rtc::gui::LineConfig &)),
      this,
      SLOT(got_trajectory(const WidgetId &, const std::vector<sva::PTransformd> &, const mc_rtc::gui::LineConfig &)));
  connect(this, SIGNAL(signal_trajectory(const WidgetId &, const Eigen::Vector3d &, const mc_rtc::gui::LineConfig &)),
          this, SLOT(got_trajectory(const WidgetId &, const Eigen::Vector3d &, const mc_rtc::gui::LineConfig &)));
  connect(this, SIGNAL(signal_trajectory(const WidgetId &, const sva::PTransformd &, const mc_rtc::gui::LineConfig &)),
          this, SLOT(got_trajectory(const WidgetId &, const sva::PTransformd &, const mc_rtc::gui::LineConfig &)));
  connect(this,
          SIGNAL(signal_polygon(const WidgetId &, const std::vector<std::vector<Eigen::Vector3d>> &,
                                const mc_rtc::gui::Color &)),
          this,
          SLOT(got_polygon(const WidgetId &, const std::vector<std::vector<Eigen::Vector3d>> &,
                           const mc_rtc::gui::Color &)));
  connect(this,
          SIGNAL(signal_force(const WidgetId &, const WidgetId &, const sva::ForceVecd &, const sva::PTransformd &,
                              const mc_rtc::gui::ForceConfig &, bool)),
          this,
          SLOT(got_force(const WidgetId &, const WidgetId &, const sva::ForceVecd &, const sva::PTransformd &,
                         const mc_rtc::gui::ForceConfig &, bool)));
  connect(this,
          SIGNAL(signal_arrow(const WidgetId &, const WidgetId &, const Eigen::Vector3d &, const Eigen::Vector3d &,
                              const mc_rtc::gui::ArrowConfig &, bool)),
          this,
          SLOT(got_arrow(const WidgetId &, const WidgetId &, const Eigen::Vector3d &, const Eigen::Vector3d &,
                         const mc_rtc::gui::ArrowConfig &, bool)));
  connect(this, SIGNAL(signal_rotation(const WidgetId &, const WidgetId &, bool, const sva::PTransformd &)), this,
          SLOT(got_rotation(const WidgetId &, const WidgetId &, bool, const sva::PTransformd &)));
  connect(this, SIGNAL(signal_transform(const WidgetId &, const WidgetId &, bool, const sva::PTransformd &)), this,
          SLOT(got_transform(const WidgetId &, const WidgetId &, bool, const sva::PTransformd &)));
  connect(this, SIGNAL(signal_xytheta(const WidgetId &, const WidgetId &, bool, const Eigen::Vector3d &, double)), this,
          SLOT(got_xytheta(const WidgetId &, const WidgetId &, bool, const Eigen::Vector3d &, double)));
  connect(this, SIGNAL(signal_schema(const WidgetId &, const std::string &)), this,
          SLOT(got_schema(const WidgetId &, const std::string &)));
  connect(this, SIGNAL(signal_form(const WidgetId &)), this, SLOT(got_form(const WidgetId &)));
  connect(this, SIGNAL(signal_form_checkbox(const WidgetId &, const std::string &, bool, bool)), this,
          SLOT(got_form_checkbox(const WidgetId &, const std::string &, bool, bool)));
  connect(this, SIGNAL(signal_form_integer_input(const WidgetId &, const std::string &, bool, int)), this,
          SLOT(got_form_integer_input(const WidgetId &, const std::string &, bool, int)));
  connect(this, SIGNAL(signal_form_number_input(const WidgetId &, const std::string &, bool, double)), this,
          SLOT(got_form_number_input(const WidgetId &, const std::string &, bool, double)));
  connect(this, SIGNAL(signal_form_string_input(const WidgetId &, const std::string &, bool, const std::string &)),
          this, SLOT(got_form_string_input(const WidgetId &, const std::string &, bool, const std::string &)));
  connect(this,
          SIGNAL(signal_form_array_input(const WidgetId &, const std::string &, bool, const Eigen::VectorXd &, bool)),
          this, SLOT(got_form_array_input(const WidgetId &, const std::string &, bool, const Eigen::VectorXd &, bool)));
  connect(
      this,
      SIGNAL(
          signal_form_combo_input(const WidgetId &, const std::string &, bool, const std::vector<std::string> &, bool)),
      this,
      SLOT(got_form_combo_input(const WidgetId &, const std::string &, bool, const std::vector<std::string> &, bool)));
  connect(this,
          SIGNAL(signal_form_data_combo_input(const WidgetId &, const std::string &, bool,
                                              const std::vector<std::string> &, bool)),
          this,
          SLOT(got_form_data_combo_input(const WidgetId &, const std::string &, bool, const std::vector<std::string> &,
                                         bool)));
  connect(this, SIGNAL(signal_start_plot(uint64_t, const std::string &)), this,
          SLOT(got_start_plot(uint64_t, const std::string &)));
  connect(this, SIGNAL(signal_plot_setup_xaxis(uint64_t, const std::string &, const mc_rtc::gui::plot::Range &)), this,
          SLOT(got_plot_setup_xaxis(uint64_t, const std::string &, const mc_rtc::gui::plot::Range &)));
  connect(this, SIGNAL(signal_plot_setup_yaxis_left(uint64_t, const std::string &, const mc_rtc::gui::plot::Range &)),
          this, SLOT(got_plot_setup_yaxis_left(uint64_t, const std::string &, const mc_rtc::gui::plot::Range &)));
  connect(this, SIGNAL(signal_plot_setup_yaxis_right(uint64_t, const std::string &, const mc_rtc::gui::plot::Range &)),
          this, SLOT(got_plot_setup_yaxis_right(uint64_t, const std::string &, const mc_rtc::gui::plot::Range &)));
  connect(this,
          SIGNAL(signal_plot_point(uint64_t, uint64_t, const std::string &, double, double, mc_rtc::gui::Color,
                                   mc_rtc::gui::plot::Style, mc_rtc::gui::plot::Side)),
          this,
          SLOT(got_plot_point(uint64_t, uint64_t, const std::string &, double, double, mc_rtc::gui::Color,
                              mc_rtc::gui::plot::Style, mc_rtc::gui::plot::Side)));
  connect(this,
          SIGNAL(signal_plot_polygon(uint64_t, uint64_t, const std::string &,
                                     const mc_rtc::gui::plot::PolygonDescription &, mc_rtc::gui::plot::Side)),
          this,
          SLOT(got_plot_polygon(uint64_t, uint64_t, const std::string &, const mc_rtc::gui::plot::PolygonDescription &,
                                mc_rtc::gui::plot::Side)));
  connect(
      this,
      SIGNAL(signal_plot_polygons(uint64_t, uint64_t, const std::string &,
                                  const std::vector<mc_rtc::gui::plot::PolygonDescription> &, mc_rtc::gui::plot::Side)),
      this,
      SLOT(got_plot_polygons(uint64_t, uint64_t, const std::string &,
                             const std::vector<mc_rtc::gui::plot::PolygonDescription> &, mc_rtc::gui::plot::Side)));
  connect(this, SIGNAL(signal_end_plot(uint64_t)), this, SLOT(got_end_plot(uint64_t)));
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
#ifndef DISABLE_ROS
  impl_->int_server_->applyChanges();
  impl_->marker_array_pub_.publish(impl_->marker_array_);
  impl_->marker_array_.markers.clear();
  if(ros::ok())
  {
    ros::spinOnce();
  }
#endif
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

void Panel::label(const WidgetId & id, const std::string & data)
{
  Q_EMIT signal_label(id, data);
}

void Panel::array_label(const WidgetId & id, const std::vector<std::string> & labels, const Eigen::VectorXd & data)
{
  Q_EMIT signal_array_label(id, labels, data);
}

void Panel::button(const WidgetId & id)
{
  Q_EMIT signal_button(id);
}

void Panel::checkbox(const WidgetId & id, bool state)
{
  Q_EMIT signal_checkbox(id, state);
}

void Panel::string_input(const WidgetId & id, const std::string & data)
{
  Q_EMIT signal_string_input(id, data);
}

void Panel::integer_input(const WidgetId & id, int data)
{
  Q_EMIT signal_integer_input(id, data);
}

void Panel::number_input(const WidgetId & id, double data)
{
  Q_EMIT signal_number_input(id, data);
}

void Panel::number_slider(const WidgetId & id, double data, double min, double max)
{
  Q_EMIT signal_number_slider(id, data, min, max);
}

void Panel::array_input(const WidgetId & id, const std::vector<std::string> & inputs, const Eigen::VectorXd & data)
{
  Q_EMIT signal_array_input(id, inputs, data);
}

void Panel::combo_input(const WidgetId & id, const std::vector<std::string> & values, const std::string & data)
{
  Q_EMIT signal_combo_input(id, values, data);
}

void Panel::data_combo_input(const WidgetId & id, const std::vector<std::string> & ref, const std::string & data)
{
  Q_EMIT signal_data_combo_input(id, ref, data);
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
  Q_EMIT signal_polygon(id, polygons, color);
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

void Panel::form(const WidgetId & id)
{
  Q_EMIT signal_form(id);
}

void Panel::form_checkbox(const WidgetId & formId, const std::string & name, bool required, bool def)
{
  Q_EMIT signal_form_checkbox(formId, name, required, def);
}

void Panel::form_integer_input(const WidgetId & formId, const std::string & name, bool required, int def)
{
  Q_EMIT signal_form_integer_input(formId, name, required, def);
}

void Panel::form_number_input(const WidgetId & formId, const std::string & name, bool required, double def)
{
  Q_EMIT signal_form_number_input(formId, name, required, def);
}

void Panel::form_string_input(const WidgetId & formId, const std::string & name, bool required, const std::string & def)
{
  Q_EMIT signal_form_string_input(formId, name, required, def);
}

void Panel::form_array_input(const WidgetId & formId,
                             const std::string & name,
                             bool required,
                             const Eigen::VectorXd & def,
                             bool fixed_size)
{
  Q_EMIT signal_form_array_input(formId, name, required, def, fixed_size);
}

void Panel::form_combo_input(const WidgetId & formId,
                             const std::string & name,
                             bool required,
                             const std::vector<std::string> & values,
                             bool send_index)
{
  Q_EMIT signal_form_combo_input(formId, name, required, values, send_index);
}

void Panel::form_data_combo_input(const WidgetId & formId,
                                  const std::string & name,
                                  bool required,
                                  const std::vector<std::string> & ref,
                                  bool send_index)
{
  Q_EMIT signal_form_data_combo_input(formId, name, required, ref, send_index);
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

void Panel::got_label(const WidgetId & id, const std::string & data)
{
  auto & w = get_widget<LabelWidget>(id);
  w.update(data);
}

void Panel::got_array_label(const WidgetId & id, const std::vector<std::string> & labels, const Eigen::VectorXd & data)
{
  auto & w = get_widget<ArrayLabelWidget>(id, labels);
  w.update(data);
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

void Panel::got_string_input(const WidgetId & id, const std::string & data)
{
  auto & w = get_widget<StringInputWidget>(id);
  w.update(data);
}

void Panel::got_integer_input(const WidgetId & id, int data)
{
  auto & w = get_widget<IntegerInputWidget>(id);
  w.update(data);
}

void Panel::got_number_input(const WidgetId & id, double data)
{
  auto & w = get_widget<NumberInputWidget>(id);
  w.update(data);
}

void Panel::got_number_slider(const WidgetId & id, double data, double min, double max)
{
  auto & w = get_widget<NumberSliderWidget>(id, min, max);
  w.update(data, min, max);
}

void Panel::got_array_input(const WidgetId & id, const std::vector<std::string> & inputs, const Eigen::VectorXd & data)
{
  auto & w = get_widget<ArrayInputWidget>(id, inputs);
  w.update(data);
}

void Panel::got_combo_input(const WidgetId & id, const std::vector<std::string> & values, const std::string & data)
{
  auto & w = get_widget<ComboInputWidget>(id, values);
  w.update(data, values);
}

void Panel::got_data_combo_input(const WidgetId & id, const std::vector<std::string> & values, const std::string & data)
{
  auto & w = get_widget<ComboInputWidget>(id, data_, values);
  w.update(data, data_, values);
}

void Panel::got_point3d(const WidgetId & id,
                        const WidgetId & requestId,
                        bool ro,
                        const Eigen::Vector3d & pos,
                        const mc_rtc::gui::PointConfig & config)
{
#ifndef DISABLE_ROS
  auto label = latestWidget_;
  auto & w = get_widget<Point3DInteractiveMarkerWidget>(id, requestId, impl_->int_server_, config, !ro, label);
  w.update(pos);
#endif
}

void Panel::got_rotation(const WidgetId & id, const WidgetId & requestId, bool ro, const sva::PTransformd & pos)
{
#ifndef DISABLE_ROS
  auto label = latestWidget_;
  auto & w = get_widget<TransformInteractiveMarkerWidget>(id, requestId, impl_->int_server_, !ro, false, label);
  w.update(pos);
#endif
}

void Panel::got_transform(const WidgetId & id, const WidgetId & requestId, bool ro, const sva::PTransformd & pos)
{
#ifndef DISABLE_ROS
  auto label = latestWidget_;
  auto & w = get_widget<TransformInteractiveMarkerWidget>(id, requestId, impl_->int_server_, !ro, !ro, label);
  w.update(pos);
#endif
}

void Panel::got_xytheta(const WidgetId & id,
                        const WidgetId & requestId,
                        bool ro,
                        const Eigen::Vector3d & vec,
                        double altitude)
{
#ifndef DISABLE_ROS
  auto label = latestWidget_;
  auto & w = get_widget<XYThetaInteractiveMarkerWidget>(id, requestId, impl_->int_server_, sva::PTransformd::Identity(),
                                                        !ro, !ro, label);
  w.update(vec, altitude);
#endif
}

void Panel::got_trajectory(const WidgetId & id,
                           const std::vector<Eigen::Vector3d> & points,
                           const mc_rtc::gui::LineConfig & config)
{
#ifndef DISABLE_ROS
  auto & w = get_widget<DisplayTrajectoryWidget>(id, impl_->marker_array_);
  w.update(points, config);
#endif
}

void Panel::got_trajectory(const WidgetId & id,
                           const std::vector<sva::PTransformd> & points,
                           const mc_rtc::gui::LineConfig & config)
{
#ifndef DISABLE_ROS
  auto & w = get_widget<DisplayTrajectoryWidget>(id, impl_->marker_array_);
  w.update(points, config);
#endif
}

void Panel::got_trajectory(const WidgetId & id, const Eigen::Vector3d & point, const mc_rtc::gui::LineConfig & config)
{
#ifndef DISABLE_ROS
  auto & w = get_widget<DisplayTrajectoryWidget>(id, impl_->marker_array_);
  w.update(point, config);
#endif
}

void Panel::got_trajectory(const WidgetId & id, const sva::PTransformd & point, const mc_rtc::gui::LineConfig & config)
{
#ifndef DISABLE_ROS
  auto & w = get_widget<DisplayTrajectoryWidget>(id, impl_->marker_array_);
  w.update(point, config);
#endif
}

void Panel::got_polygon(const WidgetId & id,
                        const std::vector<std::vector<Eigen::Vector3d>> & polygons,
                        const mc_rtc::gui::Color & c)
{
#ifndef DISABLE_ROS
  auto & w = get_widget<PolygonMarkerWidget>(id, impl_->marker_array_);
  w.update(polygons, c);
#endif
}

void Panel::got_force(const WidgetId & id,
                      const WidgetId & requestId,
                      const sva::ForceVecd & forcep,
                      const sva::PTransformd & surface,
                      const mc_rtc::gui::ForceConfig & forceConfig,
                      bool ro)
{
#ifndef DISABLE_ROS
  auto label = latestWidget_;
  auto & w = get_widget<ForceInteractiveMarkerWidget>(id, requestId, impl_->int_server_, surface, forcep, forceConfig,
                                                      ro, label);
  w.update(surface, forcep, forceConfig);
#endif
}

void Panel::got_arrow(const WidgetId & id,
                      const WidgetId & requestId,
                      const Eigen::Vector3d & start,
                      const Eigen::Vector3d & end,
                      const mc_rtc::gui::ArrowConfig & config,
                      bool ro)
{
#ifndef DISABLE_ROS
  auto label = latestWidget_;
  auto & w =
      get_widget<ArrowInteractiveMarkerWidget>(id, requestId, impl_->int_server_, start, end, config, !ro, !ro, label);
  w.update(start, end, config);
#endif
}

void Panel::got_schema(const WidgetId & id, const std::string & schema)
{
  get_widget<SchemaWidget>(id, schema, data_);
}

void Panel::got_form(const WidgetId & id)
{
  auto & form = get_widget<FormWidget>(id);
  form.update();
}

void Panel::got_form_checkbox(const WidgetId & formId, const std::string & name, bool required, bool def)
{
  auto & form = get_widget<FormWidget>(formId);
  form.element<form::Checkbox>(name, required, def);
}

void Panel::got_form_integer_input(const WidgetId & formId, const std::string & name, bool required, int def)
{
  auto & form = get_widget<FormWidget>(formId);
  form.element<form::IntegerInput>(name, required, def);
}

void Panel::got_form_number_input(const WidgetId & formId, const std::string & name, bool required, double def)
{
  auto & form = get_widget<FormWidget>(formId);
  form.element<form::NumberInput>(name, required, def);
}

void Panel::got_form_string_input(const WidgetId & formId,
                                  const std::string & name,
                                  bool required,
                                  const std::string & def)
{
  auto & form = get_widget<FormWidget>(formId);
  form.element<form::StringInput>(name, required, def);
}

void Panel::got_form_array_input(const WidgetId & formId,
                                 const std::string & name,
                                 bool required,
                                 const Eigen::VectorXd & def,
                                 bool fixed_size)
{
  auto & form = get_widget<FormWidget>(formId);
  form.element<form::NumberArrayInput>(name, required, def, fixed_size);
}

void Panel::got_form_combo_input(const WidgetId & formId,
                                 const std::string & name,
                                 bool required,
                                 const std::vector<std::string> & values,
                                 bool send_index)
{
  auto & form = get_widget<FormWidget>(formId);
  form.element<form::ComboInput>(name, required, values, send_index);
}

void Panel::got_form_data_combo_input(const WidgetId & formId,
                                      const std::string & name,
                                      bool required,
                                      const std::vector<std::string> & ref,
                                      bool send_index)
{
  auto & form = get_widget<FormWidget>(formId);
  form.element<form::DataComboInput>(name, required, data_, ref, send_index);
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
  for(const auto & c : category)
  {
    ret = &(ret->sub_trees_[c]);
  }
  return *ret;
}

void Panel::WidgetTree::start()
{
  if(parent)
  {
    parent->resetSeen();
  }
  for(auto & st : sub_trees_)
  {
    st.second.start();
  }
}

void Panel::WidgetTree::clean()
{
  for(auto it = sub_trees_.begin(); it != sub_trees_.end();)
  {

    auto & t = it->second;
    t.clean();
    int rem = 0;
    if(t.parent)
    {
      rem = t.parent->clean();
    }
    if(rem == 0 && t.sub_trees_.size() == 0)
    {
      if(parent && t.parent)
      {
        parent->removeWidget(t.parent);
      }
      sub_trees_.erase(it++);
    }
    else
    {
      ++it;
    }
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
      LOG_ERROR("Reconnection failed with provided URIs")
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
  if(!config_.has("widgets"))
  {
    config_.add("widgets");
  }
  auto ret = config_("widgets");
  for(const auto & c : id.category)
  {
    if(!ret.has(c))
    {
      ret.add(c);
    }
    ret = ret(c);
  }
  if(!ret.has(id.name))
  {
    ret.add(id.name);
  }
  return ret(id.name);
}

} // namespace mc_rtc_rviz
