/*
 * Copyright 2016-2025 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "FormElement.h"
#include "FormWidget.h"
#include "Panel.h"
#include "PanelImpl.h"

namespace mc_rtc_rviz
{

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

} // namespace mc_rtc_rviz
