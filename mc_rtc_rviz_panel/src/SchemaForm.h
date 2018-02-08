#pragma once

#include <mc_rtc/Configuration.h>

#include <QFormLayout>
#include <QLineEdit>

#include <string>

struct SchemaFormItem;

struct SchemaForm : public QWidget
{
  SchemaForm(const std::string & schema, const mc_rtc::Configuration & ctl_data);

  /** Returns an empty string if the form is ready, an error message otherwise */
  std::string ready();

  /** Generate JSON message from the form */
  mc_rtc::Configuration send();

  std::string title;

  void addSpecialItem(const std::string & name, SchemaFormItem * item);

  bool hasSpecialItem(const std::string & name) const;

  SchemaFormItem & getSpecialItem(const std::string & name);

  QFormLayout * layout = new QFormLayout();
  std::vector<std::unique_ptr<SchemaFormItem>> items_;
private:
  std::map<std::string, SchemaFormItem*> special_items_;
};
