/*
 * Copyright 2016-2025 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "ArrayInputWidget.h"
#include "ArrayLabelWidget.h"
#include "ButtonWidget.h"
#include "CheckboxWidget.h"
#include "ComboInputWidget.h"
#include "GenericInputWidget.h"
#include "LabelWidget.h"
#include "NumberSliderWidget.h"
#include "Panel.h"
#include "SchemaWidget.h"
#include "TableWidget.h"

namespace mc_rtc_rviz
{

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

} // namespace mc_rtc_rviz
