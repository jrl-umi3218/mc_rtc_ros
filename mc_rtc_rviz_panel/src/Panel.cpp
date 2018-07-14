#include "Panel.h"

#include "ArrayLabelWidget.h"
#include "ArrayInputWidget.h"
#include "ButtonWidget.h"
#include "CheckboxWidget.h"
#include "ComboInputWidget.h"
#include "FormWidget.h"
#include "FormElement.h"
#include "GenericInputWidget.h"
#ifndef DISABLE_ROS
#include "InteractiveMarkerWidget.h"
#include "DisplayTrajectoryWidget.h"
#include "PolygonMarkerWidget.h"
#endif
#include "LabelWidget.h"
#include "NumberSliderWidget.h"
#include "SchemaWidget.h"

namespace mc_rtc_rviz
{

Panel::Panel(QWidget * parent)
: mc_control::ControllerClient("ipc:///tmp/mc_rtc_pub.ipc", "ipc:///tmp/mc_rtc_rep.ipc", 2),
  CategoryWidget(ClientWidgetParam{*this, parent, {{},"ROOT"}}),
  nh_(),
  int_server_(std::make_shared<interactive_markers::InteractiveMarkerServer>("mc_rtc_rviz_interactive_markers"))
{
  qRegisterMetaType<std::string>("std::string");
  qRegisterMetaType<std::vector<std::string>>("std::vector<std::string>");
  qRegisterMetaType<WidgetId>("WidgetId");
  qRegisterMetaType<Eigen::Vector3d>("Eigen::Vector3d");
  qRegisterMetaType<Eigen::VectorXd>("Eigen::VectorXd");
  qRegisterMetaType<sva::PTransformd>("sva::PTransformd");
  qRegisterMetaType<std::vector<Eigen::Vector3d>>("std::vector<Eigen::Vector3d>");
  qRegisterMetaType<std::vector<sva::PTransformd>>("std::vector<sva::PTransformd>");
  tree_.parent = this;
  connect(this, SIGNAL(signal_start()),
          this, SLOT(got_start()));
  connect(this, SIGNAL(signal_stop()),
          this, SLOT(got_stop()));
  connect(this, SIGNAL(signal_category(const std::vector<std::string>&, const std::string&)),
          this, SLOT(got_category(const std::vector<std::string>&, const std::string&)));
  connect(this, SIGNAL(signal_label(const WidgetId&, const std::string&)),
          this, SLOT(got_label(const WidgetId&, const std::string&)));
  connect(this, SIGNAL(signal_array_label(const WidgetId&, const std::vector<std::string>&, const Eigen::VectorXd&)),
          this, SLOT(got_array_label(const WidgetId&, const std::vector<std::string>&, const Eigen::VectorXd&)));
  connect(this, SIGNAL(signal_button(const WidgetId&)),
          this, SLOT(got_button(const WidgetId&)));
  connect(this, SIGNAL(signal_checkbox(const WidgetId&, bool)),
          this, SLOT(got_checkbox(const WidgetId&, bool)));
  connect(this, SIGNAL(signal_string_input(const WidgetId&, const std::string&)),
          this, SLOT(got_string_input(const WidgetId&, const std::string&)));
  connect(this, SIGNAL(signal_integer_input(const WidgetId&, int)),
          this, SLOT(got_integer_input(const WidgetId&, int)));
  connect(this, SIGNAL(signal_number_input(const WidgetId&, double)),
          this, SLOT(got_number_input(const WidgetId&, double)));
  connect(this, SIGNAL(signal_number_slider(const WidgetId&, double, double, double)),
          this, SLOT(got_number_slider(const WidgetId&, double, double, double)));
  connect(this, SIGNAL(signal_array_input(const WidgetId&, const std::vector<std::string>&, const Eigen::VectorXd&)),
          this, SLOT(got_array_input(const WidgetId&, const std::vector<std::string>&, const Eigen::VectorXd&)));
  connect(this, SIGNAL(signal_combo_input(const WidgetId&, const std::vector<std::string>&, const std::string&)),
          this, SLOT(got_combo_input(const WidgetId&, const std::vector<std::string>&, const std::string&)));
  connect(this, SIGNAL(signal_data_combo_input(const WidgetId&, const std::vector<std::string>&, const std::string&)),
          this, SLOT(got_data_combo_input(const WidgetId&, const std::vector<std::string>&, const std::string&)));
  connect(this, SIGNAL(signal_point3d(const WidgetId&, const WidgetId&, bool, const Eigen::Vector3d&)),
          this, SLOT(got_point3d(const WidgetId&, const WidgetId&, bool, const Eigen::Vector3d&)));
  connect(this, SIGNAL(signal_displayTrajectory(const WidgetId&,  const WidgetId&, const std::vector<Eigen::Vector3d>&)),
          this, SLOT(got_displayTrajectory(const WidgetId&, const WidgetId&, const std::vector<Eigen::Vector3d>&)));
  connect(this, SIGNAL(signal_displayTrajectory(const WidgetId&,  const WidgetId&, const std::vector<sva::PTransformd>&)),
          this, SLOT(got_displayTrajectory(const WidgetId&, const WidgetId&, const std::vector<sva::PTransformd>&)));
  connect(this, SIGNAL(signal_displayPolygon(const WidgetId&,  const WidgetId&, const std::vector<Eigen::Vector3d>&)),
          this, SLOT(got_displayPolygon(const WidgetId&, const WidgetId&, const std::vector<Eigen::Vector3d>&)));
  connect(this, SIGNAL(signal_rotation(const WidgetId&, const WidgetId&, bool, const sva::PTransformd&)),
          this, SLOT(got_rotation(const WidgetId&, const WidgetId&, bool, const sva::PTransformd&)));
  connect(this, SIGNAL(signal_transform(const WidgetId&, const WidgetId&, bool, const sva::PTransformd&)),
          this, SLOT(got_transform(const WidgetId&, const WidgetId&, bool, const sva::PTransformd&)));
  connect(this, SIGNAL(signal_schema(const WidgetId&, const std::string&)),
          this, SLOT(got_schema(const WidgetId&, const std::string&)));
  connect(this, SIGNAL(signal_form(const WidgetId&)),
          this, SLOT(got_form(const WidgetId&)));
  connect(this, SIGNAL(signal_form_checkbox(const WidgetId&, const std::string&, bool, bool)),
          this, SLOT(got_form_checkbox(const WidgetId&, const std::string&, bool, bool)));
  connect(this, SIGNAL(signal_form_integer_input(const WidgetId&, const std::string&, bool, int)),
          this, SLOT(got_form_integer_input(const WidgetId&, const std::string&, bool, int)));
  connect(this, SIGNAL(signal_form_number_input(const WidgetId&, const std::string&, bool, double)),
          this, SLOT(got_form_number_input(const WidgetId&, const std::string&, bool, double)));
  connect(this, SIGNAL(signal_form_string_input(const WidgetId&, const std::string&, bool, const std::string&)),
          this, SLOT(got_form_string_input(const WidgetId&, const std::string&, bool, const std::string&)));
  connect(this, SIGNAL(signal_form_array_input(const WidgetId&, const std::string&, bool, const Eigen::VectorXd&, bool)),
          this, SLOT(got_form_array_input(const WidgetId&, const std::string&, bool, const Eigen::VectorXd&, bool)));
  connect(this, SIGNAL(signal_form_combo_input(const WidgetId&, const std::string&, bool, const std::vector<std::string>&, bool)),
          this, SLOT(got_form_combo_input(const WidgetId&, const std::string&, bool, const std::vector<std::string>&, bool)));
  connect(this, SIGNAL(signal_form_data_combo_input(const WidgetId&, const std::string&, bool, const std::vector<std::string>&, bool)),
          this, SLOT(got_form_data_combo_input(const WidgetId&, const std::string&, bool, const std::vector<std::string>&, bool)));
  mc_control::ControllerClient::start();
}

void Panel::started()
{
  Q_EMIT signal_start();
}

void Panel::got_start()
{
}

void Panel::stopped()
{
  Q_EMIT signal_stop();
}

void Panel::got_stop()
{
  tree_.clean();
  int_server_->applyChanges();
  if(ros::ok()) { ros::spinOnce(); }
}

void Panel::category(const std::vector<std::string> & parent, const std::string & category)
{
  Q_EMIT signal_category(parent, category);
}

void Panel::label(const WidgetId & id, const std::string & data)
{
  Q_EMIT signal_label(id, data);
}

void Panel::array_label(const WidgetId & id,
                        const std::vector<std::string> & labels,
                        const Eigen::VectorXd & data)
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

void Panel::array_input(const WidgetId & id,
                        const std::vector<std::string> & inputs,
                        const Eigen::VectorXd & data)
{
  Q_EMIT signal_array_input(id, inputs, data);
}

void Panel::combo_input(const WidgetId & id,
                        const std::vector<std::string> & values,
                        const std::string & data)
{
  Q_EMIT signal_combo_input(id, values, data);
}

void Panel::data_combo_input(const WidgetId & id,
                             const std::vector<std::string> & ref,
                             const std::string & data)
{
  Q_EMIT signal_data_combo_input(id, ref, data);
}

void Panel::point3d(const WidgetId & id,
                    const WidgetId & requestId,
                    bool ro, const Eigen::Vector3d & pos)
{
  Q_EMIT signal_point3d(id, requestId, ro, pos);
}

void Panel::displayTrajectory(const WidgetId & id,
                              const WidgetId & requestId,
                              const std::vector<Eigen::Vector3d> & points)
{
  Q_EMIT signal_displayTrajectory(id, requestId, points);
}

void Panel::displayTrajectory(const WidgetId & id,
                              const WidgetId & requestId,
                              const std::vector<sva::PTransformd> & points)
{
  Q_EMIT signal_displayTrajectory(id, requestId, points);
}

void Panel::displayPolygon(const WidgetId & id,
                           const WidgetId & requestId,
                           const std::vector<Eigen::Vector3d> & points)
{
  Q_EMIT signal_displayPolygon(id, requestId, points);
}

void Panel::rotation(const WidgetId & id,
                     const WidgetId & requestId,
                     bool ro, const sva::PTransformd & pos)
{
  Q_EMIT signal_rotation(id, requestId, ro, pos);
}

void Panel::transform(const WidgetId & id,
                      const WidgetId & requestId,
                      bool ro, const sva::PTransformd & pos)
{
  Q_EMIT signal_transform(id, requestId, ro, pos);
}

void Panel::schema(const WidgetId & id, const std::string & schema)
{
  Q_EMIT signal_schema(id, schema);
}

void Panel::form(const WidgetId & id)
{
  Q_EMIT signal_form(id);
}

void Panel::form_checkbox(const WidgetId & formId,
                          const std::string & name,
                          bool required,
                          bool def)
{
  Q_EMIT signal_form_checkbox(formId, name, required, def);
}

void Panel::form_integer_input(const WidgetId & formId,
                               const std::string & name,
                               bool required,
                               int def)
{
  Q_EMIT signal_form_integer_input(formId, name, required, def);
}

void Panel::form_number_input(const WidgetId & formId,
                              const std::string & name,
                              bool required,
                              double def)
{
  Q_EMIT signal_form_number_input(formId, name, required, def);
}

void Panel::form_string_input(const WidgetId & formId,
                              const std::string & name,
                              bool required,
                              const std::string & def)
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

void Panel::got_category(const std::vector<std::string> & parent, const std::string & category)
{
  auto & tree = get_category(parent);
  if(tree.sub_trees_.count(category) == 0)
  {
    auto cat = new CategoryWidget(ClientWidgetParam{*this, tree.parent, {parent, category}});
    tree.parent->addWidget(cat);
    tree.sub_trees_[category].parent = cat;
  }
  tree.sub_trees_[category].parent->seen(true);
}

void Panel::got_label(const WidgetId & id, const std::string & data)
{
  auto & w = get_widget<LabelWidget>(id);
  w.update(data);
}

void Panel::got_array_label(const WidgetId & id,
                        const std::vector<std::string> & labels,
                        const Eigen::VectorXd & data)
{
  auto & w = get_widget<ArrayLabelWidget>(id, labels);
  w.update(data);
}

void Panel::got_button(const WidgetId & id)
{
  auto & w = get_widget<ButtonWidget>(id);
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
  w.update(data);
}

void Panel::got_array_input(const WidgetId & id,
                        const std::vector<std::string> & inputs,
                        const Eigen::VectorXd & data)
{
  auto & w = get_widget<ArrayInputWidget>(id, inputs);
  w.update(data);
}

void Panel::got_combo_input(const WidgetId & id,
                            const std::vector<std::string> & values,
                            const std::string & data)
{
  auto & w = get_widget<ComboInputWidget>(id, values);
  w.update(data);
}

void Panel::got_data_combo_input(const WidgetId & id,
                            const std::vector<std::string> & values,
                            const std::string & data)
{
  auto & w = get_widget<ComboInputWidget>(id, data_, values);
  w.update(data);
}

void Panel::got_point3d(const WidgetId & id,
                        const WidgetId & requestId,
                        bool ro, const Eigen::Vector3d & pos)
{
#ifndef DISABLE_ROS
  auto & w = get_widget<InteractiveMarkerWidget>(id, *int_server_, requestId, sva::PTransformd{pos}, false, !ro);
  w.update(pos);
#endif
}

void Panel::got_rotation(const WidgetId & id,
                         const WidgetId & requestId,
                         bool ro, const sva::PTransformd & pos)
{
#ifndef DISABLE_ROS
  auto & w = get_widget<InteractiveMarkerWidget>(id, *int_server_, requestId, pos, !ro, false);
  w.update(pos);
#endif
}

void Panel::got_transform(const WidgetId & id,
                          const WidgetId & requestId,
                          bool ro, const sva::PTransformd & pos)
{
#ifndef DISABLE_ROS
  auto & w = get_widget<InteractiveMarkerWidget>(id, *int_server_, requestId, pos, !ro, !ro);
  w.update(pos);
#endif
}

void Panel::got_displayTrajectory(const WidgetId & id,
                                  const WidgetId & requestId,
                                  const std::vector<Eigen::Vector3d> & points)
{
  #ifndef DISABLE_ROS
  auto & w = get_widget<DisplayTrajectoryWidget>(id, requestId);
  w.update(points);
  #endif
}

void Panel::got_displayTrajectory(const WidgetId & id,
                                  const WidgetId & requestId,
                                  const std::vector<sva::PTransformd> & points)
{
  #ifndef DISABLE_ROS
  auto & w = get_widget<DisplayTrajectoryWidget>(id, requestId);
  w.update(points);
  #endif
}

void Panel::got_displayPolygon(const WidgetId & id,
                               const WidgetId & requestId,
                               const std::vector<Eigen::Vector3d> & points)
{
  #ifndef DISABLE_ROS
  auto & w = get_widget<PolygonMarkerWidget>(id, requestId);
  w.update(points);
  #endif
}

void Panel::got_schema(const WidgetId & id, const std::string & schema)
{
  auto & w = get_widget<SchemaWidget>(id, schema, data_);
}

void Panel::got_form(const WidgetId & id)
{
  auto & w = get_widget<FormWidget>(id);
}

void Panel::got_form_checkbox(const WidgetId & formId,
                       const std::string & name,
                       bool required,
                       bool def)
{
  auto & form = get_widget<FormWidget>(formId);
  form.element<form::Checkbox>(name, required, def);
}

void Panel::got_form_integer_input(const WidgetId & formId,
                            const std::string & name,
                            bool required,
                            int def)
{
  auto & form = get_widget<FormWidget>(formId);
  form.element<form::IntegerInput>(name, required, def);
}

void Panel::got_form_number_input(const WidgetId & formId,
                           const std::string & name,
                           bool required,
                           double def)
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


Panel::WidgetTree & Panel::get_category(const std::vector<std::string> & category)
{
  auto ret = &tree_;
  for(const auto & c : category)
  {
    ret = &(ret->sub_trees_[c]);
  }
  return *ret;
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
      if(parent && t.parent) { parent->removeWidget(t.parent); }
      sub_trees_.erase(it++);
    }
    else
    {
      ++it;
    }
  }
}

} // namespace mc_rtc_rviz
