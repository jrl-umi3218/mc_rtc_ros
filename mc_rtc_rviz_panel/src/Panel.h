#pragma once

#ifndef Q_MOC_RUN

#include <memory>
#include <unordered_map>

#include "CategoryWidget.h"

#endif

#ifndef DISABLE_ROS

  #include <interactive_markers/interactive_marker_server.h>
  #include <visualization_msgs/MarkerArray.h>
  #include <ros/ros.h>
  #include <rviz/panel.h>
  #include <tf/tfMessage.h>
  #include <tf/transform_listener.h>

#else

namespace ros
{
  struct NodeHandle {};
  struct Publisher {};
  inline void spinOnce() {}
  inline bool ok() { return true; }
  inline void init(int argc, char * argv[], const std::string &) {}
}

namespace interactive_markers
{
  struct InteractiveMarkerServer
  {
    struct FeedbackCallback {};
    InteractiveMarkerServer(const std::string &) {}
    void applyChanges() {}
  };
}

namespace visualization_msgs
{
  struct MarkerArray {};
}

#endif

namespace mc_rtc_rviz
{

class Panel: public CategoryWidget, public mc_control::ControllerClient
{
Q_OBJECT
public:
  Panel(QWidget* parent = 0);

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
    void clean();
    std::map<std::string, WidgetTree> sub_trees_;
  };
  WidgetTree tree_;

  WidgetTree & get_category(const std::vector<std::string> & category);

  void category(const std::vector<std::string> & parent, const std::string & category) override;

  void label(const WidgetId & id, const std::string & data) override;

  void array_label(const WidgetId & id,
                   const std::vector<std::string> & labels,
                   const Eigen::VectorXd & data) override;

  void button(const WidgetId & id) override;

  void checkbox(const WidgetId & id, bool state) override;

  void string_input(const WidgetId & id, const std::string & data) override;

  void integer_input(const WidgetId & id, int data) override;

  void number_input(const WidgetId & id, double data) override;

  void number_slider(const WidgetId & id, double data, double min, double max) override;

  void array_input(const WidgetId & id,
                   const std::vector<std::string> & labels,
                   const Eigen::VectorXd & data) override;

  void combo_input(const WidgetId & id,
                   const std::vector<std::string> & values,
                   const std::string & data) override;

  void data_combo_input(const WidgetId & id,
                        const std::vector<std::string> & ref,
                        const std::string & data) override;

  void point3d(const WidgetId & id,
               const WidgetId & requestId,
               bool ro, const Eigen::Vector3d & pos,
               const mc_rtc::gui::PointConfig & config) override;

  void trajectory(const WidgetId & id,
                  const std::vector<Eigen::Vector3d> & pos,
                  const mc_rtc::gui::LineConfig & config) override;

  void trajectory(const WidgetId & id,
                  const std::vector<sva::PTransformd> & pos,
                  const mc_rtc::gui::LineConfig & config) override;

  void trajectory(const WidgetId & id,
                  const Eigen::Vector3d & point,
                  const mc_rtc::gui::LineConfig & config) override;

  void trajectory(const WidgetId & id,
                  const sva::PTransformd & point,
                  const mc_rtc::gui::LineConfig & config) override;

  void polygon(const WidgetId & id,
               const std::vector<std::vector<Eigen::Vector3d>> & pos,
               const mc_rtc::gui::Color& color) override;

  void force(const WidgetId & id,
             const WidgetId & requestId,
             const sva::ForceVecd & force,
             const sva::PTransformd & surface,
             const mc_rtc::gui::ForceConfig & forceConfig) override;

  void arrow(const WidgetId & id,
             const Eigen::Vector3d& start,
             const Eigen::Vector3d& end,
             const mc_rtc::gui::ArrowConfig & config) override;

  void rotation(const WidgetId & id,
                const WidgetId & requestId,
                bool ro, const sva::PTransformd & pos) override;

  void transform(const WidgetId & id,
                 const WidgetId & requestId,
                 bool ro, const sva::PTransformd & pos) override;

  void schema(const WidgetId & id, const std::string & schema) override;

  void form(const WidgetId & id) override;

  void form_checkbox(const WidgetId & formId,
                     const std::string & name,
                     bool required,
                     bool def) override;

  void form_integer_input(const WidgetId & formId,
                          const std::string & name,
                          bool required,
                          int def) override;

  void form_number_input(const WidgetId & formId,
                         const std::string & name,
                         bool required,
                         double def) override;

  void form_string_input(const WidgetId & formId,
                         const std::string & name,
                         bool required,
                         const std::string & def) override;

  void form_array_input(const WidgetId & formId,
                        const std::string & name,
                        bool required,
                        const Eigen::VectorXd & def,
                        bool fixed_size) override;

  void form_combo_input(const WidgetId & formId,
                        const std::string & name,
                        bool required,
                        const std::vector<std::string> & values,
                        bool send_index) override;

  void form_data_combo_input(const WidgetId & formId,
                             const std::string & name,
                             bool required,
                             const std::vector<std::string> & ref,
                             bool send_index) override;

  template<typename T, typename ... Args>
  T & get_widget(const WidgetId & id, Args && ... args)
  {
    auto parent = get_category(id.category).parent;
    assert(parent);
    auto w_ptr = parent->widget(id.name);
    if(!w_ptr)
    {
      w_ptr = new T(ClientWidgetParam{*this, parent, id}, std::forward<Args>(args)...);
      parent->addWidget(w_ptr);
    }
    auto & w = static_cast<T&>(*w_ptr);
    w.seen(true);
    return w;
  }

  /** Returns the configuration of a given widget */
  mc_rtc::Configuration config(const WidgetId & id) const;
private:
  /** ROS stuff */
  ros::NodeHandle nh_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> int_server_;
  visualization_msgs::MarkerArray marker_array_;
  ros::Publisher marker_array_pub_;
  /** Configuration */
  mutable mc_rtc::Configuration config_;
  std::string sub_uri_;
  std::string push_uri_;
private slots:
  void contextMenu(const QPoint & pos);
  void contextMenu_editConnection();
  void contextMenu_reconnect();
  void got_start();
  void got_stop();
  void got_category(const std::vector<std::string> & parent, const std::string & category);
  void got_label(const WidgetId & id, const std::string & data);
  void got_array_label(const WidgetId & id,
                       const std::vector<std::string> & labels,
                       const Eigen::VectorXd & data);
  void got_button(const WidgetId & id);
  void got_checkbox(const WidgetId & id, bool state);
  void got_string_input(const WidgetId & id, const std::string & data);
  void got_integer_input(const WidgetId & id, int data);
  void got_number_input(const WidgetId & id, double data);
  void got_number_slider(const WidgetId & id, double data, double min, double max);
  void got_array_input(const WidgetId & id,
                       const std::vector<std::string> & labels,
                       const Eigen::VectorXd & data);
  void got_combo_input(const WidgetId & id,
                       const std::vector<std::string> & values,
                       const std::string & data);
  void got_data_combo_input(const WidgetId & id,
                            const std::vector<std::string> & values,
                            const std::string & data);
  void got_point3d(const WidgetId & id,
                   const WidgetId & requestId,
                   bool ro, const Eigen::Vector3d & pos,
                   const mc_rtc::gui::PointConfig&);
  void got_rotation(const WidgetId & id,
                    const WidgetId & requestId,
                    bool ro, const sva::PTransformd & pos);
  void got_transform(const WidgetId & id,
                     const WidgetId & requestId,
                     bool ro, const sva::PTransformd & pos);
  void got_trajectory(const WidgetId & id,
                      const std::vector<Eigen::Vector3d> & points,
                      const mc_rtc::gui::LineConfig & config);
  void got_trajectory(const WidgetId & id,
                      const std::vector<sva::PTransformd> & points,
                      const mc_rtc::gui::LineConfig & config);
  void got_trajectory(const WidgetId & id,
                      const Eigen::Vector3d & point,
                      const mc_rtc::gui::LineConfig & config);
  void got_trajectory(const WidgetId & id,
                      const sva::PTransformd & point,
                      const mc_rtc::gui::LineConfig & config);
  void got_polygon(const WidgetId & id,
                   const std::vector<std::vector<Eigen::Vector3d>> & polygons,
                   const mc_rtc::gui::Color & c);
  void got_force(const WidgetId & id,
                 const WidgetId & requestId,
                 const sva::ForceVecd & force,
                 const sva::PTransformd & surface,
                 const mc_rtc::gui::ForceConfig & forceConfig);
  void got_arrow(const WidgetId & id,
                 const Eigen::Vector3d & force,
                 const Eigen::Vector3d & surface,
                 const mc_rtc::gui::ArrowConfig & config);
  void got_schema(const WidgetId & id, const std::string & schema);
  void got_form(const WidgetId & id);
  void got_form_checkbox(const WidgetId & formId,
                         const std::string & name,
                         bool required,
                         bool def);
  void got_form_integer_input(const WidgetId & formId,
                              const std::string & name,
                              bool required,
                              int def);
  void got_form_number_input(const WidgetId & formId,
                             const std::string & name,
                             bool required,
                             double def);
  void got_form_string_input(const WidgetId & formId,
                             const std::string & name,
                             bool required,
                             const std::string & def);
  void got_form_array_input(const WidgetId & formId,
                            const std::string & name,
                            bool required,
                            const Eigen::VectorXd & def,
                            bool fixed_size);
  void got_form_combo_input(const WidgetId & formId,
                            const std::string & name,
                            bool required,
                            const std::vector<std::string> & values,
                            bool send_index);
  void got_form_data_combo_input(const WidgetId & formId,
                                 const std::string & name,
                                 bool required,
                                 const std::vector<std::string> & ref,
                                 bool send_index);
signals:
  void signal_start();
  void signal_stop();
  void signal_category(const std::vector<std::string> & parent, const std::string & category);
  void signal_label(const WidgetId & id, const std::string & data);
  void signal_array_label(const WidgetId & id,
                          const std::vector<std::string> & labels,
                          const Eigen::VectorXd & data);
  void signal_button(const WidgetId & id);
  void signal_checkbox(const WidgetId & id, bool state);
  void signal_string_input(const WidgetId & id, const std::string & data);
  void signal_integer_input(const WidgetId & id, int data);
  void signal_number_input(const WidgetId & id, double data);
  void signal_number_slider(const WidgetId & id, double data, double min, double max);
  void signal_array_input(const WidgetId & id,
                          const std::vector<std::string> & labels,
                          const Eigen::VectorXd & data);
  void signal_combo_input(const WidgetId & id,
                          const std::vector<std::string> & values,
                          const std::string & data);
  void signal_data_combo_input(const WidgetId & id,
                               const std::vector<std::string> & values,
                               const std::string & data);
  void signal_point3d(const WidgetId & id,
                      const WidgetId & requestId,
                      bool ro, const Eigen::Vector3d & pos,
                      const mc_rtc::gui::PointConfig&);
  void signal_trajectory(const WidgetId & id,
                         const std::vector<Eigen::Vector3d> & pos,
                         const mc_rtc::gui::LineConfig & config);
  void signal_trajectory(const WidgetId & id,
                         const std::vector<sva::PTransformd> & pos,
                         const mc_rtc::gui::LineConfig & config);
  void signal_trajectory(const WidgetId & id,
                         const Eigen::Vector3d & point,
                         const mc_rtc::gui::LineConfig & config);
  void signal_trajectory(const WidgetId & id,
                         const sva::PTransformd & point,
                         const mc_rtc::gui::LineConfig & config);
  void signal_polygon(const WidgetId & id,
                      const std::vector<std::vector<Eigen::Vector3d>> & polygons,
                      const mc_rtc::gui::Color & c);
 void signal_force(const WidgetId & id,
                   const WidgetId & requestId,
                   const sva::ForceVecd & force,
                   const sva::PTransformd & surface,
                   const mc_rtc::gui::ForceConfig & forceConfig);
  void signal_arrow(const WidgetId & id,
                    const Eigen::Vector3d & force,
                    const Eigen::Vector3d & surface,
                    const mc_rtc::gui::ArrowConfig & config);
  void signal_rotation(const WidgetId & id,
                       const WidgetId & requestId,
                       bool ro, const sva::PTransformd & pos);
  void signal_transform(const WidgetId & id,
                        const WidgetId & requestId,
                        bool ro, const sva::PTransformd & pos);
  void signal_schema(const WidgetId & id, const std::string & schema);
  void signal_form(const WidgetId & id);
  void signal_form_checkbox(const WidgetId & formId,
                            const std::string & name,
                            bool required,
                            bool def);
  void signal_form_integer_input(const WidgetId & formId,
                                 const std::string & name,
                                 bool required,
                                 int def);
  void signal_form_number_input(const WidgetId & formId,
                                const std::string & name,
                                bool required,
                                double def);
  void signal_form_string_input(const WidgetId & formId,
                                const std::string & name,
                                bool required,
                                const std::string & def);
  void signal_form_array_input(const WidgetId & formId,
                               const std::string & name,
                               bool required,
                               const Eigen::VectorXd & def,
                            bool fixed_size);
  void signal_form_combo_input(const WidgetId & formId,
                               const std::string & name,
                               bool required,
                               const std::vector<std::string> & values,
                               bool send_index);
  void signal_form_data_combo_input(const WidgetId & formId,
                                    const std::string & name,
                                    bool required,
                                    const std::vector<std::string> & ref,
                                    bool send_index);
};

}
