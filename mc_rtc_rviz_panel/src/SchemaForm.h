#pragma once

#include <mc_rtc/Configuration.h>

#include <QFormLayout>
#include <QLineEdit>

#include <string>

struct SchemaFormItem;

struct SchemaForm : public QWidget
{
  SchemaForm(const std::string & schema);

  /** Returns an empty string if the form is ready, an error message otherwise */
  std::string ready();

  /** Generate JSON message from the form */
  mc_rtc::Configuration send();

  std::string title;

  QFormLayout * layout = new QFormLayout();
  std::vector<std::unique_ptr<SchemaFormItem>> items_;
};
