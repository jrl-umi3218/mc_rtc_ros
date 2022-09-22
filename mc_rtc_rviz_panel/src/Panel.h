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

#ifndef DISABLE_ROS

#  include <ros/ros.h>
#  include <visualization_msgs/MarkerArray.h>

#  include <interactive_markers/interactive_marker_server.h>
#  include <tf/tfMessage.h>
#  include <tf/transform_listener.h>

#endif

namespace mc_rtc_rviz
{

/** Hide ROS details in pimpl */
struct PanelImpl;

class Panel : public CategoryWidget, public mc_control::ControllerClient
{
  Q_OBJECT
public:
  Panel(QWidget * parent = 0);

  ~Panel();

  using CategoryWidget::connect;

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

  void category(const std::vector<std::string> & parent, const std::string & category) override;

  void label(const WidgetId & id, const std::string & data) override;

  void array_label(const WidgetId & id, const std::vector<std::string> & labels, const Eigen::VectorXd & data) override;

  void button(const WidgetId & id) override;

  void checkbox(const WidgetId & id, bool state) override;

  void string_input(const WidgetId & id, const std::string & data) override;

  void integer_input(const WidgetId & id, int data) override;

  void number_input(const WidgetId & id, double data) override;

  void number_slider(const WidgetId & id, double data, double min, double max) override;

  void array_input(const WidgetId & id, const std::vector<std::string> & labels, const Eigen::VectorXd & data) override;

  void combo_input(const WidgetId & id, const std::vector<std::string> & values, const std::string & data) override;

  void data_combo_input(const WidgetId & id, const std::vector<std::string> & ref, const std::string & data) override;

  void point3d(const WidgetId & id,
               const WidgetId & requestId,
               bool ro,
               const Eigen::Vector3d & pos,
               const mc_rtc::gui::PointConfig & config) override;

  void trajectory(const WidgetId & id,
                  const std::vector<Eigen::Vector3d> & pos,
                  const mc_rtc::gui::LineConfig & config) override;

  void trajectory(const WidgetId & id,
                  const std::vector<sva::PTransformd> & pos,
                  const mc_rtc::gui::LineConfig & config) override;

  void trajectory(const WidgetId & id, const Eigen::Vector3d & point, const mc_rtc::gui::LineConfig & config) override;

  void trajectory(const WidgetId & id, const sva::PTransformd & point, const mc_rtc::gui::LineConfig & config) override;

  void polygon(const WidgetId & id,
               const std::vector<std::vector<Eigen::Vector3d>> & pos,
               const mc_rtc::gui::Color & color) override;

  void polygon(const WidgetId & id,
               const std::vector<std::vector<Eigen::Vector3d>> & pos,
               const mc_rtc::gui::LineConfig & config) override;

  void polyhedron(const WidgetId & id,
               const std::vector<std::array<Eigen::Vector3d,3>> & triangles,
               const std::vector<std::array<mc_rtc::gui::Color,3>> & colors,
               const mc_rtc::gui::PolyhedronConfig & c) override;

  void force(const WidgetId & id,
             const WidgetId & requestId,
             const sva::ForceVecd & force,
             const sva::PTransformd & surface,
             const mc_rtc::gui::ForceConfig & forceConfig,
             bool ro) override;

  void arrow(const WidgetId & id,
             const WidgetId & requestId,
             const Eigen::Vector3d & start,
             const Eigen::Vector3d & end,
             const mc_rtc::gui::ArrowConfig & config,
             bool ro) override;

  void rotation(const WidgetId & id, const WidgetId & requestId, bool ro, const sva::PTransformd & pos) override;

  void transform(const WidgetId & id, const WidgetId & requestId, bool ro, const sva::PTransformd & pos) override;

  void xytheta(const WidgetId & id,
               const WidgetId & requestId,
               bool ro,
               const Eigen::Vector3d & vec,
               double altitude) override;

  void schema(const WidgetId & id, const std::string & schema) override;

  void table_start(const WidgetId & id, const std::vector<std::string> & header) override;

  void table_row(const WidgetId & id, const std::vector<std::string> & data) override;

  void table_end(const WidgetId & id) override;

  void robot(const WidgetId & id,
             const std::vector<std::string> & parameters,
             const std::vector<std::vector<double>> & q,
             const sva::PTransformd & posW) override;

  void visual(const WidgetId & id, const rbd::parsers::Visual & visual, const sva::PTransformd & pose) override;

  void form(const WidgetId & id) override;

  void form_checkbox(const WidgetId & formId,
                     const std::string & name,
                     bool required,
                     bool def,
                     bool def_from_user) override;

  void form_integer_input(const WidgetId & formId,
                          const std::string & name,
                          bool required,
                          int def,
                          bool def_from_user) override;

  void form_number_input(const WidgetId & formId,
                         const std::string & name,
                         bool required,
                         double def,
                         bool def_from_user) override;

  void form_string_input(const WidgetId & formId,
                         const std::string & name,
                         bool required,
                         const std::string & def,
                         bool def_from_user) override;

  void form_array_input(const WidgetId & formId,
                        const std::string & name,
                        bool required,
                        const Eigen::VectorXd & def,
                        bool fixed_size,
                        bool def_from_user) override;

  void form_combo_input(const WidgetId & formId,
                        const std::string & name,
                        bool required,
                        const std::vector<std::string> & values,
                        bool send_index,
                        int def) override;

  void form_data_combo_input(const WidgetId & formId,
                             const std::string & name,
                             bool required,
                             const std::vector<std::string> & ref,
                             bool send_index) override;

  void start_plot(uint64_t id, const std::string & title) override;
  void plot_setup_xaxis(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range) override;
  void plot_setup_yaxis_left(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range) override;
  void plot_setup_yaxis_right(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range) override;
  void plot_point(uint64_t id,
                  uint64_t did,
                  const std::string & legend,
                  double x,
                  double y,
                  mc_rtc::gui::Color color,
                  mc_rtc::gui::plot::Style style,
                  mc_rtc::gui::plot::Side side) override;
  void plot_polygon(uint64_t id,
                    uint64_t did,
                    const std::string & legend,
                    const mc_rtc::gui::plot::PolygonDescription & polygon,
                    mc_rtc::gui::plot::Side side) override;
  void plot_polygons(uint64_t id,
                     uint64_t did,
                     const std::string & legend,
                     const std::vector<mc_rtc::gui::plot::PolygonDescription> & polygon,
                     mc_rtc::gui::plot::Side side) override;
  void end_plot(uint64_t id) override;

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
  /** Configuration */
  mutable mc_rtc::Configuration config_;
  mutable std::vector<ConnectionConfiguration> connectionConfigs_;
private slots:
  void contextMenu(const QPoint & pos);
  void contextMenu_editConnection();
  void contextMenu_reconnect();
  void got_start();
  void got_stop();
  void got_category(const std::vector<std::string> & parent, const std::string & category);
  void got_label(const WidgetId & id, const std::string & data);
  void got_array_label(const WidgetId & id, const std::vector<std::string> & labels, const Eigen::VectorXd & data);
  void got_button(const WidgetId & id);
  void got_checkbox(const WidgetId & id, bool state);
  void got_string_input(const WidgetId & id, const std::string & data);
  void got_integer_input(const WidgetId & id, int data);
  void got_number_input(const WidgetId & id, double data);
  void got_number_slider(const WidgetId & id, double data, double min, double max);
  void got_array_input(const WidgetId & id, const std::vector<std::string> & labels, const Eigen::VectorXd & data);
  void got_combo_input(const WidgetId & id, const std::vector<std::string> & values, const std::string & data);
  void got_data_combo_input(const WidgetId & id, const std::vector<std::string> & values, const std::string & data);
  void got_point3d(const WidgetId & id,
                   const WidgetId & requestId,
                   bool ro,
                   const Eigen::Vector3d & pos,
                   const mc_rtc::gui::PointConfig &);
  void got_rotation(const WidgetId & id, const WidgetId & requestId, bool ro, const sva::PTransformd & pos);
  void got_transform(const WidgetId & id, const WidgetId & requestId, bool ro, const sva::PTransformd & pos);
  void got_xytheta(const WidgetId & id,
                   const WidgetId & requestId,
                   bool ro,
                   const Eigen::Vector3d & vec,
                   double altitude);
  void got_trajectory(const WidgetId & id,
                      const std::vector<Eigen::Vector3d> & points,
                      const mc_rtc::gui::LineConfig & config);
  void got_trajectory(const WidgetId & id,
                      const std::vector<sva::PTransformd> & points,
                      const mc_rtc::gui::LineConfig & config);
  void got_trajectory(const WidgetId & id, const Eigen::Vector3d & point, const mc_rtc::gui::LineConfig & config);
  void got_trajectory(const WidgetId & id, const sva::PTransformd & point, const mc_rtc::gui::LineConfig & config);
  void got_polygon(const WidgetId & id,
                   const std::vector<std::vector<Eigen::Vector3d>> & polygons,
                   const mc_rtc::gui::LineConfig & c);
  void got_polyhedron(const WidgetId & id,
                   const std::vector<std::array<Eigen::Vector3d, 3>> & triangles,
                   const std::vector<std::array<mc_rtc::gui::Color, 3>> & colors,
                   const mc_rtc::gui::PolyhedronConfig & c);
  void got_force(const WidgetId & id,
                 const WidgetId & requestId,
                 const sva::ForceVecd & force,
                 const sva::PTransformd & surface,
                 const mc_rtc::gui::ForceConfig & forceConfig,
                 bool ro);
  void got_arrow(const WidgetId & id,
                 const WidgetId & requestId,
                 const Eigen::Vector3d & force,
                 const Eigen::Vector3d & surface,
                 const mc_rtc::gui::ArrowConfig & config,
                 bool ro);
  void got_schema(const WidgetId & id, const std::string & schema);
  void got_table_start(const WidgetId & id, const std::vector<std::string> & header);
  void got_table_row(const WidgetId & id, const std::vector<std::string> & data);
  void got_table_end(const WidgetId & id);
  void got_robot(const WidgetId & id,
                 const std::vector<std::string> & parameters,
                 const std::vector<std::vector<double>> & q,
                 const sva::PTransformd & posW);
  void got_visual(const WidgetId & id, const rbd::parsers::Visual & visual, const sva::PTransformd & pose);
  void got_form(const WidgetId & id);
  void got_form_checkbox(const WidgetId & formId, const std::string & name, bool required, bool def, bool def_from_user);
  void got_form_integer_input(const WidgetId & formId,
                              const std::string & name,
                              bool required,
                              int def,
                              bool def_from_user);
  void got_form_number_input(const WidgetId & formId,
                             const std::string & name,
                             bool required,
                             double def,
                             bool def_from_user);
  void got_form_string_input(const WidgetId & formId,
                             const std::string & name,
                             bool required,
                             const std::string & def,
                             bool def_from_user);
  void got_form_array_input(const WidgetId & formId,
                            const std::string & name,
                            bool required,
                            const Eigen::VectorXd & def,
                            bool fixed_size,
                            bool def_from_user);
  void got_form_combo_input(const WidgetId & formId,
                            const std::string & name,
                            bool required,
                            const std::vector<std::string> & values,
                            bool send_index,
                            int def);
  void got_form_data_combo_input(const WidgetId & formId,
                                 const std::string & name,
                                 bool required,
                                 const std::vector<std::string> & ref,
                                 bool send_index);
  void got_start_plot(uint64_t id, const std::string & title);
  void got_plot_setup_xaxis(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range);
  void got_plot_setup_yaxis_left(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range);
  void got_plot_setup_yaxis_right(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range);
  void got_plot_point(uint64_t id,
                      uint64_t did,
                      const std::string & legend,
                      double x,
                      double y,
                      mc_rtc::gui::Color color,
                      mc_rtc::gui::plot::Style style,
                      mc_rtc::gui::plot::Side side);
  void got_plot_polygon(uint64_t id,
                        uint64_t did,
                        const std::string & legend,
                        const mc_rtc::gui::plot::PolygonDescription & polygon,
                        mc_rtc::gui::plot::Side side);
  void got_plot_polygons(uint64_t id,
                         uint64_t did,
                         const std::string & legend,
                         const std::vector<mc_rtc::gui::plot::PolygonDescription> & polygon,
                         mc_rtc::gui::plot::Side side);
  void got_end_plot(uint64_t id);
signals:
  void signal_start();
  void signal_stop();
  void signal_category(const std::vector<std::string> & parent, const std::string & category);
  void signal_label(const WidgetId & id, const std::string & data);
  void signal_array_label(const WidgetId & id, const std::vector<std::string> & labels, const Eigen::VectorXd & data);
  void signal_button(const WidgetId & id);
  void signal_checkbox(const WidgetId & id, bool state);
  void signal_string_input(const WidgetId & id, const std::string & data);
  void signal_integer_input(const WidgetId & id, int data);
  void signal_number_input(const WidgetId & id, double data);
  void signal_number_slider(const WidgetId & id, double data, double min, double max);
  void signal_array_input(const WidgetId & id, const std::vector<std::string> & labels, const Eigen::VectorXd & data);
  void signal_combo_input(const WidgetId & id, const std::vector<std::string> & values, const std::string & data);
  void signal_data_combo_input(const WidgetId & id, const std::vector<std::string> & values, const std::string & data);
  void signal_point3d(const WidgetId & id,
                      const WidgetId & requestId,
                      bool ro,
                      const Eigen::Vector3d & pos,
                      const mc_rtc::gui::PointConfig &);
  void signal_trajectory(const WidgetId & id,
                         const std::vector<Eigen::Vector3d> & pos,
                         const mc_rtc::gui::LineConfig & config);
  void signal_trajectory(const WidgetId & id,
                         const std::vector<sva::PTransformd> & pos,
                         const mc_rtc::gui::LineConfig & config);
  void signal_trajectory(const WidgetId & id, const Eigen::Vector3d & point, const mc_rtc::gui::LineConfig & config);
  void signal_trajectory(const WidgetId & id, const sva::PTransformd & point, const mc_rtc::gui::LineConfig & config);
  void signal_polygon(const WidgetId & id,
                      const std::vector<std::vector<Eigen::Vector3d>> & polygons,
                      const mc_rtc::gui::LineConfig & c);
  void signal_polyhedron(const WidgetId & id,
                   const std::vector<std::array<Eigen::Vector3d, 3>> & triangles,
                   const std::vector<std::array<mc_rtc::gui::Color, 3>> & colors,
                   const mc_rtc::gui::PolyhedronConfig & c);
  void signal_force(const WidgetId & id,
                    const WidgetId & requestId,
                    const sva::ForceVecd & force,
                    const sva::PTransformd & surface,
                    const mc_rtc::gui::ForceConfig & forceConfig,
                    bool ro);
  void signal_arrow(const WidgetId & id,
                    const WidgetId & requestId,
                    const Eigen::Vector3d & force,
                    const Eigen::Vector3d & surface,
                    const mc_rtc::gui::ArrowConfig & config,
                    bool ro);
  void signal_rotation(const WidgetId & id, const WidgetId & requestId, bool ro, const sva::PTransformd & pos);
  void signal_transform(const WidgetId & id, const WidgetId & requestId, bool ro, const sva::PTransformd & pos);
  void signal_xytheta(const WidgetId & id,
                      const WidgetId & requestId,
                      bool ro,
                      const Eigen::Vector3d & vec,
                      double altitude);
  void signal_schema(const WidgetId & id, const std::string & schema);
  void signal_table_start(const WidgetId & id, const std::vector<std::string> & header);
  void signal_table_row(const WidgetId & id, const std::vector<std::string> & data);
  void signal_table_end(const WidgetId & id);
  void signal_robot(const WidgetId & id,
                    const std::vector<std::string> & parameters,
                    const std::vector<std::vector<double>> & q,
                    const sva::PTransformd & posW);
  void signal_visual(const WidgetId & id, const rbd::parsers::Visual & visual, const sva::PTransformd & pose);
  void signal_form(const WidgetId & id);
  void signal_form_checkbox(const WidgetId & formId,
                            const std::string & name,
                            bool required,
                            bool def,
                            bool def_from_user);
  void signal_form_integer_input(const WidgetId & formId,
                                 const std::string & name,
                                 bool required,
                                 int def,
                                 bool def_from_user);
  void signal_form_number_input(const WidgetId & formId,
                                const std::string & name,
                                bool required,
                                double def,
                                bool def_from_user);
  void signal_form_string_input(const WidgetId & formId,
                                const std::string & name,
                                bool required,
                                const std::string & def,
                                bool def_from_user);
  void signal_form_array_input(const WidgetId & formId,
                               const std::string & name,
                               bool required,
                               const Eigen::VectorXd & def,
                               bool fixed_size,
                               bool def_from_user);
  void signal_form_combo_input(const WidgetId & formId,
                               const std::string & name,
                               bool required,
                               const std::vector<std::string> & values,
                               bool send_index,
                               int def);
  void signal_form_data_combo_input(const WidgetId & formId,
                                    const std::string & name,
                                    bool required,
                                    const std::vector<std::string> & ref,
                                    bool send_index);
  void signal_start_plot(uint64_t id, const std::string & title);
  void signal_plot_setup_xaxis(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range);
  void signal_plot_setup_yaxis_left(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range);
  void signal_plot_setup_yaxis_right(uint64_t id, const std::string & legend, const mc_rtc::gui::plot::Range & range);
  void signal_plot_point(uint64_t id,
                         uint64_t did,
                         const std::string & legend,
                         double x,
                         double y,
                         mc_rtc::gui::Color color,
                         mc_rtc::gui::plot::Style style,
                         mc_rtc::gui::plot::Side side);
  void signal_plot_polygon(uint64_t id,
                           uint64_t did,
                           const std::string & legend,
                           const mc_rtc::gui::plot::PolygonDescription & polygon,
                           mc_rtc::gui::plot::Side side);
  void signal_plot_polygons(uint64_t id,
                            uint64_t did,
                            const std::string & legend,
                            const std::vector<mc_rtc::gui::plot::PolygonDescription> & polygon,
                            mc_rtc::gui::plot::Side side);
  void signal_end_plot(uint64_t id);
};

} // namespace mc_rtc_rviz
