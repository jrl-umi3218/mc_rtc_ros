/*
 * Copyright 2016-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#ifndef Q_MOC_RUN

#  include "CategoryWidget.h"
#  include "ConnectionConfiguration.h"

#  include <memory>
#  include <unordered_map>

#endif

#include <mc_rtc_ros/ros.h>

namespace mc_rtc_rviz
{

struct FormElementContainer;

/** Hide ROS details in pimpl */
struct PanelImpl;

class Panel : public CategoryWidget, public mc_control::ControllerClient
{
  Q_OBJECT
public:
  Panel(QWidget * parent = 0);

  ~Panel();

  using CategoryWidget::connect;

  /** Returns the current ROS time */
  Time now() const;

  /** Returns true if the provided widget should be visible
   *
   * If the widget has never been seen, returns true
   *
   */
  bool visible(const WidgetId & id) const;

  /** Save visibility setting of a specific widget */
  void visible(const WidgetId & id, bool visibility);

protected:
  void started() override;

  void stopped() override;

  struct WidgetTree
  {
    ClientWidget * parent = nullptr;
    void start();
    void clean();
    std::map<std::string, WidgetTree> sub_trees_;
  };
  WidgetTree tree_;

  WidgetTree & get_category(const std::vector<std::string> & category);

#define DEFINE_OVERRIDE(...)          \
  Q_SLOT void got_##__VA_ARGS__;      \
                                      \
  Q_SIGNAL void signal_##__VA_ARGS__; \
                                      \
protected:                            \
  void __VA_ARGS__ override;

  DEFINE_OVERRIDE(category(const std::vector<std::string> & parent, const std::string & category))

  DEFINE_OVERRIDE(label(const WidgetId & id, const std::string & data))

  DEFINE_OVERRIDE(array_label(const WidgetId & id,
                              const std::vector<std::string> & labels,
                              const Eigen::VectorXd & data))

  DEFINE_OVERRIDE(button(const WidgetId & id))

  DEFINE_OVERRIDE(checkbox(const WidgetId & id, bool state))

  DEFINE_OVERRIDE(string_input(const WidgetId & id, const std::string & data))

  DEFINE_OVERRIDE(integer_input(const WidgetId & id, int data))

  DEFINE_OVERRIDE(number_input(const WidgetId & id, double data))

  DEFINE_OVERRIDE(number_slider(const WidgetId & id, double data, double min, double max))

  DEFINE_OVERRIDE(array_input(const WidgetId & id,
                              const std::vector<std::string> & labels,
                              const Eigen::VectorXd & data))

  DEFINE_OVERRIDE(combo_input(const WidgetId & id, const std::vector<std::string> & values, const std::string & data))

  DEFINE_OVERRIDE(data_combo_input(const WidgetId & id, const std::vector<std::string> & ref, const std::string & data))

  DEFINE_OVERRIDE(point3d(const WidgetId & id,
                          const WidgetId & requestId,
                          bool ro,
                          const Eigen::Vector3d & pos,
                          const mc_rtc::gui::PointConfig & config))

  DEFINE_OVERRIDE(trajectory(const WidgetId & id,
                             const std::vector<Eigen::Vector3d> & pos,
                             const mc_rtc::gui::LineConfig & config))

  DEFINE_OVERRIDE(trajectory(const WidgetId & id,
                             const std::vector<sva::PTransformd> & pos,
                             const mc_rtc::gui::LineConfig & config))

  DEFINE_OVERRIDE(trajectory(const WidgetId & id,
                             const Eigen::Vector3d & point,
                             const mc_rtc::gui::LineConfig & config))

  DEFINE_OVERRIDE(trajectory(const WidgetId & id,
                             const sva::PTransformd & point,
                             const mc_rtc::gui::LineConfig & config))

  void polygon(const WidgetId & id,
               const std::vector<std::vector<Eigen::Vector3d>> & pos,
               const mc_rtc::gui::Color & color) override;

  DEFINE_OVERRIDE(polygon(const WidgetId & id,
                          const std::vector<std::vector<Eigen::Vector3d>> & pos,
                          const mc_rtc::gui::LineConfig & config))

  DEFINE_OVERRIDE(polyhedron(const WidgetId & id,
                             const std::vector<std::array<Eigen::Vector3d, 3>> & triangles,
                             const std::vector<std::array<mc_rtc::gui::Color, 3>> & colors,
                             const mc_rtc::gui::PolyhedronConfig & c))

  DEFINE_OVERRIDE(force(const WidgetId & id,
                        const WidgetId & requestId,
                        const sva::ForceVecd & force,
                        const sva::PTransformd & surface,
                        const mc_rtc::gui::ForceConfig & forceConfig,
                        bool ro))

  DEFINE_OVERRIDE(arrow(const WidgetId & id,
                        const WidgetId & requestId,
                        const Eigen::Vector3d & start,
                        const Eigen::Vector3d & end,
                        const mc_rtc::gui::ArrowConfig & config,
                        bool ro))

  DEFINE_OVERRIDE(rotation(const WidgetId & id, const WidgetId & requestId, bool ro, const sva::PTransformd & pos))

  DEFINE_OVERRIDE(transform(const WidgetId & id, const WidgetId & requestId, bool ro, const sva::PTransformd & pos))

  DEFINE_OVERRIDE(
      xytheta(const WidgetId & id, const WidgetId & requestId, bool ro, const Eigen::Vector3d & vec, double altitude))

  DEFINE_OVERRIDE(schema(const WidgetId & id, const std::string & schema))

  DEFINE_OVERRIDE(table_start(const WidgetId & id, const std::vector<std::string> & header))

  DEFINE_OVERRIDE(table_row(const WidgetId & id, const std::vector<std::string> & data))

  DEFINE_OVERRIDE(table_end(const WidgetId & id))

  DEFINE_OVERRIDE(robot(const WidgetId & id,
                        const std::vector<std::string> & parameters,
                        const std::vector<std::vector<double>> & q,
                        const sva::PTransformd & posW))

  DEFINE_OVERRIDE(visual(const WidgetId & id, const rbd::parsers::Visual & visual, const sva::PTransformd & pose))

  DEFINE_OVERRIDE(form(const WidgetId & id))

  DEFINE_OVERRIDE(
      form_checkbox(const WidgetId & formId, const std::string & name, bool required, bool def, bool def_from_user))

  DEFINE_OVERRIDE(
      form_integer_input(const WidgetId & formId, const std::string & name, bool required, int def, bool def_from_user))

  DEFINE_OVERRIDE(form_number_input(const WidgetId & formId,
                                    const std::string & name,
                                    bool required,
                                    double def,
                                    bool def_from_user))

  DEFINE_OVERRIDE(form_string_input(const WidgetId & formId,
                                    const std::string & name,
                                    bool required,
                                    const std::string & def,
                                    bool def_from_user))

  DEFINE_OVERRIDE(form_array_input(const WidgetId & formId,
                                   const std::string & name,
                                   bool required,
                                   const Eigen::VectorXd & def,
                                   bool fixed_size,
                                   bool def_from_user))

  DEFINE_OVERRIDE(form_combo_input(const WidgetId & formId,
                                   const std::string & name,
                                   bool required,
                                   const std::vector<std::string> & values,
                                   bool send_index,
                                   int def))

  DEFINE_OVERRIDE(form_data_combo_input(const WidgetId & formId,
                                        const std::string & name,
                                        bool required,
                                        const std::vector<std::string> & ref,
                                        bool send_index))

  DEFINE_OVERRIDE(form_point3d_input(const WidgetId & formId,
                                     const std::string & name,
                                     bool required,
                                     const Eigen::Vector3d & default_,
                                     bool default_from_user,
                                     bool interactive))

  DEFINE_OVERRIDE(form_rotation_input(const WidgetId & formId,
                                      const std::string & name,
                                      bool required,
                                      const sva::PTransformd & default_,
                                      bool default_from_user,
                                      bool interactive))

  DEFINE_OVERRIDE(form_transform_input(const WidgetId & formId,
                                       const std::string & name,
                                       bool required,
                                       const sva::PTransformd & default_,
                                       bool default_from_user,
                                       bool interactive))

  DEFINE_OVERRIDE(start_form_object_input(const std::string & name, bool required))
  DEFINE_OVERRIDE(end_form_object_input())

  DEFINE_OVERRIDE(start_form_generic_array_input(const std::string & name,
                                                 bool required,
                                                 std::optional<std::vector<mc_rtc::Configuration>> data))
  DEFINE_OVERRIDE(end_form_generic_array_input())

  DEFINE_OVERRIDE(start_form_one_of_input(const std::string & name,
                                          bool required,
                                          const std::optional<std::pair<size_t, mc_rtc::Configuration>> & data))
  DEFINE_OVERRIDE(end_form_one_of_input())

  DEFINE_OVERRIDE(start_plot(uint64_t id, const std::string & title))
  DEFINE_OVERRIDE(plot_setup_xaxis(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range))
  DEFINE_OVERRIDE(plot_setup_yaxis_left(uint64_t id,
                                        const std::string & legend,
                                        const mc_rtc::gui::plot::Range & range))
  DEFINE_OVERRIDE(plot_setup_yaxis_right(uint64_t id,
                                         const std::string & legend,
                                         const mc_rtc::gui::plot::Range & range))
  DEFINE_OVERRIDE(plot_point(uint64_t id,
                             uint64_t did,
                             const std::string & legend,
                             double x,
                             double y,
                             mc_rtc::gui::Color color,
                             mc_rtc::gui::plot::Style style,
                             mc_rtc::gui::plot::Side side))
  DEFINE_OVERRIDE(plot_polygon(uint64_t id,
                               uint64_t did,
                               const std::string & legend,
                               const mc_rtc::gui::plot::PolygonDescription & polygon,
                               mc_rtc::gui::plot::Side side))
  DEFINE_OVERRIDE(plot_polygons(uint64_t id,
                                uint64_t did,
                                const std::string & legend,
                                const std::vector<mc_rtc::gui::plot::PolygonDescription> & polygon,
                                mc_rtc::gui::plot::Side side))
  DEFINE_OVERRIDE(end_plot(uint64_t id))

#undef DEFINE_OVERRIDE

  template<typename T, typename... Args>
  T & get_widget(const WidgetId & id, Args &&... args)
  {
    auto parent = get_category(id.category).parent;
    assert(parent);
    auto w_ptr = parent->widget(id.name);
    auto actual_ptr = dynamic_cast<T *>(w_ptr);
    if(w_ptr && !actual_ptr)
    {
      parent->removeWidget(w_ptr);
      w_ptr = nullptr;
    }
    if(!w_ptr)
    {
      w_ptr = new T(ClientWidgetParam{*this, parent, id}, std::forward<Args>(args)...);
      parent->addWidget(w_ptr);
    }
    auto & w = static_cast<T &>(*w_ptr);
    w.seen();
    latestWidget_ = w_ptr;
    return w;
  }

  /** Returns the configuration of a given widget */
  mc_rtc::Configuration config(const WidgetId & id) const;

private:
  std::unique_ptr<PanelImpl> impl_;
  /** Latest widget added */
  ClientWidget * latestWidget_ = nullptr;
  /** Active form */
  FormElementContainer * activeForm_ = nullptr;
  /** Configuration */
  mutable mc_rtc::Configuration config_;
  mutable std::vector<ConnectionConfiguration> connectionConfigs_;
private slots:
  void contextMenu(const QPoint & pos);
  void contextMenu_editConnection();
  void contextMenu_reconnect();
  void got_start();
  void got_stop();
signals:
  void signal_start();
  void signal_stop();
};

} // namespace mc_rtc_rviz
